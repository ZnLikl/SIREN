#include <plan_manage/phantom_attack_generator.h>
#include <cmath>
#include <limits>
#include <algorithm>
#include <vector>
#include <bspline_opt/lbfgs.hpp>
#include <bspline_opt/uniform_bspline.h>

namespace ego_planner
{

namespace
{
struct PhantomShapeParams
{
  bool enable = true;
  double attack_strength_gain = 1.8;
  double victim_pull_gain = 1.25;
  double max_ctrl_offset = 4.5;
  double target_weight = 25.0;
  double smoothness_weight = 2.0;
  double velocity_weight = 4.0;
  double acceleration_weight = 3.0;
  double victim_weight = 0.0;
  double preserve_fake_ratio = 0.7;
  int max_iterations = 80;
  int obstacle_backoff_steps = 12;
};

PhantomShapeParams g_shape;

struct ShapeOptContext
{
  Eigen::MatrixXd original;
  Eigen::MatrixXd target;
  std::vector<Eigen::Vector3d> victim_pts;
  double dt = 0.1;
  double max_vel = 3.0;
  double max_acc = 4.0;
  int first_free = 3;
};

static Eigen::MatrixXd msgToMatrix(const traj_utils::Bspline &msg)
{
  Eigen::MatrixXd pts(3, static_cast<int>(msg.pos_pts.size()));
  for (size_t i = 0; i < msg.pos_pts.size(); ++i)
  {
    pts(0, static_cast<int>(i)) = msg.pos_pts[i].x;
    pts(1, static_cast<int>(i)) = msg.pos_pts[i].y;
    pts(2, static_cast<int>(i)) = msg.pos_pts[i].z;
  }
  return pts;
}

static void matrixToMsg(const Eigen::MatrixXd &pts, traj_utils::Bspline &msg)
{
  msg.pos_pts.resize(static_cast<size_t>(pts.cols()));
  for (int i = 0; i < pts.cols(); ++i)
  {
    msg.pos_pts[static_cast<size_t>(i)].x = pts(0, i);
    msg.pos_pts[static_cast<size_t>(i)].y = pts(1, i);
    msg.pos_pts[static_cast<size_t>(i)].z = pts(2, i);
  }
}
static Eigen::Vector3d estimateAppliedOffset(
    const traj_utils::Bspline &real_bspline,
    const traj_utils::Bspline &fake_bspline,
    int skip_ctrl_pts)
{
  const int n = std::min(
      static_cast<int>(real_bspline.pos_pts.size()),
      static_cast<int>(fake_bspline.pos_pts.size()));

  if (n <= 0) return Eigen::Vector3d::Zero();

  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  int count = 0;

  const int start = std::max(0, std::min(skip_ctrl_pts, n - 1));

  for (int i = start; i < n; ++i)
  {
    const Eigen::Vector3d real_p(
        real_bspline.pos_pts[i].x,
        real_bspline.pos_pts[i].y,
        real_bspline.pos_pts[i].z);

    const Eigen::Vector3d fake_p(
        fake_bspline.pos_pts[i].x,
        fake_bspline.pos_pts[i].y,
        fake_bspline.pos_pts[i].z);

    sum += fake_p - real_p;
    ++count;
  }

  if (count > 0)
  {
    return sum / static_cast<double>(count);
  }

  return Eigen::Vector3d::Zero();
}
static void preserveFakeOffset(
    const Eigen::MatrixXd &original,
    const Eigen::MatrixXd &target,
    Eigen::MatrixXd &optimized,
    int first_free,
    double preserve_ratio)
{
  const int cols = original.cols();
  const double ratio = std::max(0.0, std::min(1.0, preserve_ratio));

  for (int i = std::max(0, first_free); i < cols; ++i)
  {
    Eigen::Vector3d ref_delta = target.col(i) - original.col(i);
    const double ref_norm = ref_delta.norm();

    if (ref_norm < 1e-6) continue;

    Eigen::Vector3d dir = ref_delta / ref_norm;
    Eigen::Vector3d opt_delta = optimized.col(i) - original.col(i);

    const double projected = opt_delta.dot(dir);
    const double required = ratio * ref_norm;

    if (projected < required)
    {
      optimized.col(i) += (required - projected) * dir;
    }
  }
}

static bool controlPointsFree(const Eigen::MatrixXd &pts, const GridMap::Ptr &grid_map)
{
  if (grid_map == nullptr) return true;
  for (int i = 0; i < pts.cols(); ++i)
    if (grid_map->getInflateOccupancy(pts.col(i))) return false;
  return true;
}

static bool curveSamplesFree(const Eigen::MatrixXd &pts, const traj_utils::Bspline &ref, const GridMap::Ptr &grid_map)
{
  if (grid_map == nullptr || pts.cols() < 4) return true;
  const double interval = ref.knots.size() >= 2 ? std::max(0.02, ref.knots[1] - ref.knots[0]) : 0.1;
  ego_planner::UniformBspline traj(pts, std::max(3, ref.order), interval);
  if (ref.knots.size() == static_cast<size_t>(pts.cols() + ref.order + 1))
  {
    Eigen::VectorXd knots(static_cast<int>(ref.knots.size()));
    for (size_t i = 0; i < ref.knots.size(); ++i) knots(static_cast<int>(i)) = ref.knots[i];
    traj.setKnot(knots);
  }
  const double duration = traj.getTimeSum();
  const double step = std::max(0.03, std::min(0.10, interval));
  for (double t = 0.0; t <= duration + 1e-4; t += step)
    if (grid_map->getInflateOccupancy(traj.evaluateDeBoorT(std::min(t, duration)))) return false;
  return true;
}

static double shapeCost(void *func_data, const double *x, double *grad, const int n)
{
  ShapeOptContext *ctx = reinterpret_cast<ShapeOptContext *>(func_data);
  const int cols = ctx->original.cols();
  Eigen::MatrixXd q = ctx->original;
  Eigen::MatrixXd g = Eigen::MatrixXd::Zero(3, cols);

  for (int i = ctx->first_free; i < cols && 3 * (i - ctx->first_free) + 2 < n; ++i)
    q.col(i) = Eigen::Vector3d(x[3 * (i - ctx->first_free)], x[3 * (i - ctx->first_free) + 1], x[3 * (i - ctx->first_free) + 2]);

  double cost = 0.0;

  for (int i = ctx->first_free; i < cols; ++i)
  {
    Eigen::Vector3d d = q.col(i) - ctx->target.col(i);
    cost += g_shape.target_weight * d.squaredNorm();
    g.col(i) += 2.0 * g_shape.target_weight * d;

    // if (i < static_cast<int>(ctx->victim_pts.size()) && ctx->victim_pts[i].allFinite())
    // {
    //   Eigen::Vector3d to_victim = q.col(i) - ctx->victim_pts[i];
    //   const double dist = std::max(1e-3, to_victim.norm());
    //   const double inv = 1.0 / dist;
    //   cost += g_shape.victim_weight * inv;
    //   g.col(i) += -g_shape.victim_weight * to_victim / (dist * dist * dist);
    // }
  }

  for (int i = 1; i + 1 < cols; ++i)
  {
    Eigen::Vector3d a = q.col(i - 1) - 2.0 * q.col(i) + q.col(i + 1);
    cost += g_shape.smoothness_weight * a.squaredNorm();
    g.col(i - 1) += 2.0 * g_shape.smoothness_weight * a;
    g.col(i) += -4.0 * g_shape.smoothness_weight * a;
    g.col(i + 1) += 2.0 * g_shape.smoothness_weight * a;
  }

  const double dt = std::max(0.02, ctx->dt);
  for (int i = 1; i < cols; ++i)
  {
    Eigen::Vector3d v = (q.col(i) - q.col(i - 1)) / dt;
    const double excess = v.norm() - ctx->max_vel;
    if (excess > 0.0)
    {
      Eigen::Vector3d dir = v.normalized();
      cost += g_shape.velocity_weight * excess * excess;
      g.col(i) += 2.0 * g_shape.velocity_weight * excess * dir / dt;
      g.col(i - 1) -= 2.0 * g_shape.velocity_weight * excess * dir / dt;
    }
  }

  for (int i = 2; i < cols; ++i)
  {
    Eigen::Vector3d acc = (q.col(i) - 2.0 * q.col(i - 1) + q.col(i - 2)) / (dt * dt);
    const double excess = acc.norm() - ctx->max_acc;
    if (excess > 0.0)
    {
      Eigen::Vector3d dir = acc.normalized();
      const Eigen::Vector3d term = 2.0 * g_shape.acceleration_weight * excess * dir / (dt * dt);
      cost += g_shape.acceleration_weight * excess * excess;
      g.col(i) += term;
      g.col(i - 1) -= 2.0 * term;
      g.col(i - 2) += term;
    }
  }

  if (grad != nullptr)
  {
    for (int i = ctx->first_free; i < cols && 3 * (i - ctx->first_free) + 2 < n; ++i)
    {
      grad[3 * (i - ctx->first_free)] = g(0, i);
      grad[3 * (i - ctx->first_free) + 1] = g(1, i);
      grad[3 * (i - ctx->first_free) + 2] = g(2, i);
    }
  }

  return cost;
}
}


PhantomAttackGenerator::PhantomAttackGenerator()
{
  std::random_device rd;
  rng_.seed(rd());
}

void PhantomAttackGenerator::init(ros::NodeHandle &nh)
{
  /*
   * 参数兼容：
   * 推荐使用 attack/xxx 形式；同时兼容 attack_xxx 形式，避免 launch 中只写 arg
   * 或旧参数名导致攻击没有生效。
   */
  nh.param("attack/enable", attack_enable_, false);
  nh.param("attack_enable", attack_enable_, attack_enable_);

  int method_id = 0;
  nh.param("attack/method_id", method_id, 0);
  nh.param("attack_method_id", method_id, method_id);

  std::string method_name;
  nh.param<std::string>("attack/method", method_name, std::string("benign"));
  nh.param<std::string>("attack_method", method_name, method_name);
  attack_method_ = parseMethod(method_name, method_id);

  nh.param("attack/malicious_drone_id", malicious_drone_id_, -1);
  nh.param("attack_malicious_drone_id", malicious_drone_id_, malicious_drone_id_);
  nh.param("attack/victim_drone_id", victim_drone_id_, -1);
  nh.param("attack_victim_drone_id", victim_drone_id_, victim_drone_id_);

  std::string scenario_name;
  nh.param<std::string>("attack/scenario", scenario_name, std::string("random"));
  nh.param<std::string>("attack_scenario", scenario_name, scenario_name);
  if (scenario_name == "herding")
    attack_scenario_ = SCENARIO_HERDING;
  else if (scenario_name == "trap")
    attack_scenario_ = SCENARIO_TRAP;
  else if (scenario_name == "oscillation")
    attack_scenario_ = SCENARIO_OSCILLATION;
  else
    attack_scenario_ = SCENARIO_RANDOM;

  nh.param("attack/scenario_weight", scenario_weight_, scenario_weight_);
  nh.param("attack_scenario_weight", scenario_weight_, scenario_weight_);

  nh.param("attack/herding_weight", herding_weight_, herding_weight_);
  nh.param("attack_herding_weight", herding_weight_, herding_weight_);
  nh.param("attack/herding_direction_x", herding_direction_.x(), herding_direction_.x());
  nh.param("attack_herding_direction_x", herding_direction_.x(), herding_direction_.x());
  nh.param("attack/herding_direction_y", herding_direction_.y(), herding_direction_.y());
  nh.param("attack_herding_direction_y", herding_direction_.y(), herding_direction_.y());
  nh.param("attack/herding_direction_z", herding_direction_.z(), herding_direction_.z());
  nh.param("attack_herding_direction_z", herding_direction_.z(), herding_direction_.z());

  nh.param("attack/trap_weight", trap_weight_, trap_weight_);
  nh.param("attack_trap_weight", trap_weight_, trap_weight_);
  nh.param("attack/trap_exit_x", trap_exit_.x(), trap_exit_.x());
  nh.param("attack_trap_exit_x", trap_exit_.x(), trap_exit_.x());
  nh.param("attack/trap_exit_y", trap_exit_.y(), trap_exit_.y());
  nh.param("attack_trap_exit_y", trap_exit_.y(), trap_exit_.y());
  nh.param("attack/trap_exit_z", trap_exit_.z(), trap_exit_.z());
  nh.param("attack_trap_exit_z", trap_exit_.z(), trap_exit_.z());

  nh.param("attack/oscillation_weight", oscillation_weight_, oscillation_weight_);
  nh.param("attack_oscillation_weight", oscillation_weight_, oscillation_weight_);
  nh.param("attack/oscillation_period", oscillation_period_, oscillation_period_);
  nh.param("attack_oscillation_period", oscillation_period_, oscillation_period_);
  nh.param("attack/oscillation_amplitude", oscillation_amplitude_, oscillation_amplitude_);
  nh.param("attack_oscillation_amplitude", oscillation_amplitude_, oscillation_amplitude_);
  nh.param("attack/oscillation_approach_offset",
           oscillation_approach_offset_, oscillation_approach_offset_);
  nh.param("attack_oscillation_approach_offset",
           oscillation_approach_offset_, oscillation_approach_offset_);
  nh.param("attack/oscillation_use_sine", oscillation_use_sine_, oscillation_use_sine_);
  nh.param("attack_oscillation_use_sine", oscillation_use_sine_, oscillation_use_sine_);

  nh.param("attack/window_enable", attack_window_enable_, false);
  nh.param("attack_window_enable", attack_window_enable_, attack_window_enable_);
  nh.param("attack/start_time", attack_start_time_, 0.0);
  nh.param("attack_start_time", attack_start_time_, attack_start_time_);
  nh.param("attack/duration", attack_duration_, 1e9);
  nh.param("attack_duration", attack_duration_, attack_duration_);

  nh.param("attack/fixed_offset_x", fixed_offset_x_, 1.5);
  nh.param("attack_fixed_offset_x", fixed_offset_x_, fixed_offset_x_);
  nh.param("attack/fixed_offset_y", fixed_offset_y_, 2.0);
  nh.param("attack_fixed_offset_y", fixed_offset_y_, fixed_offset_y_);
  nh.param("attack/fixed_offset_z", fixed_offset_z_, 0.0);
  nh.param("attack_fixed_offset_z", fixed_offset_z_, fixed_offset_z_);

  nh.param("attack/max_offset_radius", max_offset_radius_, 2.5);
  nh.param("attack_max_offset_radius", max_offset_radius_, max_offset_radius_);
  nh.param("attack/min_offset_radius", min_offset_radius_, 0.2);
  nh.param("attack_min_offset_radius", min_offset_radius_, min_offset_radius_);
  nh.param("attack/direction_samples", direction_samples_, 16);
  nh.param("attack_direction_samples", direction_samples_, direction_samples_);
  nh.param("attack/radius_samples", radius_samples_, 5);
  nh.param("attack_radius_samples", radius_samples_, radius_samples_);
  nh.param("attack/random_max_trials", random_max_trials_, 50);
  nh.param("attack_random_max_trials", random_max_trials_, random_max_trials_);

  nh.param("attack/horizon", horizon_, 5.0);
  nh.param("attack_horizon", horizon_, horizon_);
  nh.param("attack/dt", dt_, 0.1);
  nh.param("attack_dt", dt_, dt_);
  nh.param("attack/clearance", clearance_, 0.5);
  nh.param("attack_clearance", clearance_, clearance_);
  nh.param("attack/impact_distance_weight", impact_distance_weight_, 0.05);
  nh.param("attack/offset_penalty_weight", offset_penalty_weight_, 0.02);

  nh.param("attack/continuity_enable", continuity_enable_, true);
  nh.param("attack_continuity_enable", continuity_enable_, continuity_enable_);
  nh.param("attack/smoothing_time", smoothing_time_, 1.0);
  nh.param("attack_smoothing_time", smoothing_time_, smoothing_time_);
  nh.param("attack/max_offset_rate", max_offset_rate_, 0.8);
  nh.param("attack_max_offset_rate", max_offset_rate_, max_offset_rate_);
  nh.param("attack/ramp_ctrl_pts", ramp_ctrl_pts_, 5);
  nh.param("attack_ramp_ctrl_pts", ramp_ctrl_pts_, ramp_ctrl_pts_);
  nh.param("attack/ramp_power", ramp_power_, 1.0);
  nh.param("attack_ramp_power", ramp_power_, ramp_power_);

  nh.param("attack/max_vel_check", max_vel_check_, 3.0);
  nh.param("attack_max_vel_check", max_vel_check_, max_vel_check_);
  nh.param("attack/max_acc_check", max_acc_check_, 4.0);
  nh.param("attack_max_acc_check", max_acc_check_, max_acc_check_);
  nh.param("attack/max_jerk_check", max_jerk_check_, 8.0);
  nh.param("attack_max_jerk_check", max_jerk_check_, max_jerk_check_);


  nh.param("attack/shape_opt_enable", g_shape.enable, g_shape.enable);
  nh.param("attack_shape_opt_enable", g_shape.enable, g_shape.enable);
  nh.param("attack/strength_gain", g_shape.attack_strength_gain, g_shape.attack_strength_gain);
  nh.param("attack_strength_gain", g_shape.attack_strength_gain, g_shape.attack_strength_gain);
  nh.param("attack/victim_pull_gain", g_shape.victim_pull_gain, g_shape.victim_pull_gain);
  nh.param("attack_victim_pull_gain", g_shape.victim_pull_gain, g_shape.victim_pull_gain);
  nh.param("attack/max_ctrl_offset", g_shape.max_ctrl_offset, g_shape.max_ctrl_offset);
  nh.param("attack_max_ctrl_offset", g_shape.max_ctrl_offset, g_shape.max_ctrl_offset);
  nh.param("attack/shape_target_weight", g_shape.target_weight, g_shape.target_weight);
  nh.param("attack/shape_smoothness_weight", g_shape.smoothness_weight, g_shape.smoothness_weight);
  nh.param("attack/shape_velocity_weight", g_shape.velocity_weight, g_shape.velocity_weight);
  nh.param("attack/shape_acceleration_weight", g_shape.acceleration_weight, g_shape.acceleration_weight);
  nh.param("attack/shape_victim_weight", g_shape.victim_weight, g_shape.victim_weight);
  nh.param("attack/shape_opt_max_iterations", g_shape.max_iterations, g_shape.max_iterations);
  nh.param("attack/obstacle_backoff_steps", g_shape.obstacle_backoff_steps, g_shape.obstacle_backoff_steps);

  nh.param("attack/preserve_fake_ratio", g_shape.preserve_fake_ratio, g_shape.preserve_fake_ratio);
  nh.param("attack_preserve_fake_ratio", g_shape.preserve_fake_ratio, g_shape.preserve_fake_ratio);
  ROS_WARN_STREAM("[PhantomAttackGenerator] enable=" << attack_enable_
                  << " method=" << static_cast<int>(attack_method_)
                  << " scenario=" << scenario_name
                  << " malicious=" << malicious_drone_id_
                  << " victim=" << victim_drone_id_
                  << " max_offset_radius=" << max_offset_radius_
                  << " continuity=" << continuity_enable_
                  << " scenario_weight=" << scenario_weight_
                  << " herding_dir=" << herding_direction_.transpose()
                  << " trap_exit=" << trap_exit_.transpose()
                  << " oscillation_period=" << oscillation_period_);
}

PhantomAttackGenerator::AttackMethod PhantomAttackGenerator::parseMethod(const std::string &name, int id) const
{
  if (name == "benign") return BENIGN;
  if (name == "random_fake") return RANDOM_FAKE;
  if (name == "no_temporal_refresh") return NO_TEMPORAL_REFRESH;
  if (name == "no_impact_objective") return NO_IMPACT_OBJECTIVE;
  if (name == "full_attack") return FULL_ATTACK;
  if (name == "naive_infeasible") return NAIVE_INFEASIBLE;
  if (id < BENIGN || id > NAIVE_INFEASIBLE) return BENIGN;
  return static_cast<AttackMethod>(id);
}

bool PhantomAttackGenerator::shouldAttack(int drone_id) const
{
  return attack_enable_ && drone_id == malicious_drone_id_ && attack_method_ != BENIGN;
}

bool PhantomAttackGenerator::isWithinAttackWindow(const ros::Time &now) const
{
  if (!attack_window_enable_) return true;
  const double t = now.toSec();
  return t >= attack_start_time_ && t <= attack_start_time_ + attack_duration_;
}

traj_utils::Bspline PhantomAttackGenerator::generate(
    const traj_utils::Bspline &real_bspline,
    const SwarmTrajData &swarm_trajs,
    const Eigen::Vector3d &odom_pos,
    const Eigen::Vector3d &odom_vel,
    int self_drone_id,
    const ros::Time &now,
    const GridMap::Ptr &grid_map)
{
  if (!shouldAttack(self_drone_id) || !isWithinAttackWindow(now))
  {
    last_score_ = 0.0;
    target_offset_.setZero();
    applied_offset_.setZero();
    last_physical_check_pass_ = true;
    return real_bspline;
  }

  if (attack_scenario_ == SCENARIO_OSCILLATION)
  {
    if (last_oscillation_switch_time_.isZero())
    {
      last_oscillation_switch_time_ = now;
    }

    if ((now - last_oscillation_switch_time_).toSec() >= std::max(0.1, oscillation_period_))
    {
      oscillation_side_ *= -1;
      last_oscillation_switch_time_ = now;
    }
  }

  /*
   * No Temporal Refresh：第一次生成后缓存，后续不随 receding horizon 更新。
   * 注意：缓存轨迹仍然保持原始 drone_id，不能伪造 fake id。
   */
  if (attack_method_ == NO_TEMPORAL_REFRESH && no_refresh_cached_)
  {
    traj_utils::Bspline cached = no_refresh_cache_;
    cached.start_time = now;
    cached.drone_id = real_bspline.drone_id;
    cached.traj_id = real_bspline.traj_id;

    target_offset_ = no_refresh_target_offset_;
    applied_offset_ = no_refresh_applied_offset_;

    last_physical_check_pass_ = physicalCheck(cached);
    last_score_ = scoreOffsetAgainstVictim(real_bspline, swarm_trajs, target_offset_, now);

    return cached;
  }

  Eigen::Vector3d desired = Eigen::Vector3d::Zero();
  double score = 0.0;

  switch (attack_method_)
  {
  case RANDOM_FAKE:
    /*
     * Random Fake：随机偏移，但必须通过攻击 UAV 自己的 local grid_map_
     * 障碍物检查；不使用全局地图，不优化 victim impact。
     */
    desired = chooseRandomFeasibleOffset(real_bspline, grid_map);
    score = scoreOffsetAgainstVictim(real_bspline, swarm_trajs, desired, now); // 只用于日志，不参与选择
    break;

  case NAIVE_INFEASIBLE:
    /*
     * Naive infeasible：故意不加物理一致性约束，生成明显异常偏移。
     * 该方法用于证明粗暴攻击虽然可能有效，但容易被物理一致性检测发现。
     */
    desired = chooseNaiveInfeasibleOffset();
    score = scoreOffsetAgainstVictim(real_bspline, swarm_trajs, desired, now);
    break;

  case NO_IMPACT_OBJECTIVE:
    /*
     * No impact objective：只保证轨迹尽量物理可行、局部地图可行，
     * 不针对 victim collision cost 优化。
     */
    desired = chooseNoImpactOffset(real_bspline, grid_map);
    score = scoreOffsetAgainstVictim(real_bspline, swarm_trajs, desired, now); // 只用于记录效果
    break;

  case FULL_ATTACK:
    /*
     * Full attack：
     * 普通场景使用候选偏移搜索；
     * oscillation 场景每个发布周期都根据最新 victim 轨迹，
     * 在当前飞行方向的局部左右方向上构造周期偏移。
     */
    if (attack_scenario_ == SCENARIO_OSCILLATION)
    {
      desired = computeOscillationOffset(
          real_bspline, swarm_trajs, odom_pos, odom_vel, now, grid_map, score);
    }
    else
    {
      desired = chooseFullAttackOffset(real_bspline, swarm_trajs, now, grid_map, score);
    }
    break;

  case NO_TEMPORAL_REFRESH:
    /*
     * No temporal refresh 第一次生成时使用完整方法，之后缓存不刷新。
     */
    desired = chooseFullAttackOffset(real_bspline, swarm_trajs, now, grid_map, score);
    break;

  default:
    return real_bspline;
  }

  target_offset_ = desired;

  /*
   * naive_infeasible 不做连续化；其它方法默认连续化。
   */
  const bool use_continuity = continuity_enable_ && attack_method_ != NAIVE_INFEASIBLE;
  const Eigen::Vector3d applied = use_continuity ? smoothOffset(desired, now) : desired;

  traj_utils::Bspline attacked = applyOffsetWithRamp(real_bspline, applied);

  if (attack_scenario_ == SCENARIO_OSCILLATION)
  {
    bool victim_centered_success = false;
    traj_utils::Bspline victim_centered = buildVictimCenteredOscillation(
        real_bspline,
        swarm_trajs,
        odom_pos,
        now,
        grid_map,
        victim_centered_success);
    if (victim_centered_success) attacked = victim_centered;
  }

  if (g_shape.enable &&
      attack_scenario_ != SCENARIO_HERDING &&
      attack_scenario_ != SCENARIO_OSCILLATION &&
      (attack_method_ == FULL_ATTACK || attack_method_ == NO_TEMPORAL_REFRESH))
  {
    Eigen::MatrixXd original = msgToMatrix(real_bspline);
    Eigen::MatrixXd target = msgToMatrix(attacked);
    std::vector<Eigen::Vector3d> victim_pts(static_cast<size_t>(original.cols()),
                                            Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(),
                                                            std::numeric_limits<double>::quiet_NaN(),
                                                            std::numeric_limits<double>::quiet_NaN()));

    for (int i = 0; i < original.cols(); ++i)
    {
      const double w = original.cols() > 1 ? static_cast<double>(i) / static_cast<double>(original.cols() - 1) : 1.0;
      Eigen::Vector3d p_victim;
      if (victim_drone_id_ >= 0 && victim_drone_id_ < static_cast<int>(swarm_trajs.size()) &&
          evalSwarmTraj(swarm_trajs[victim_drone_id_], now, w * horizon_, p_victim))
      {
        victim_pts[static_cast<size_t>(i)] = p_victim;
        Eigen::Vector3d pull = p_victim - target.col(i);
        const double pull_norm = pull.norm();
        if (pull_norm > 1e-3)
          target.col(i) += std::min(g_shape.victim_pull_gain, pull_norm) * pull.normalized();
      }

      Eigen::Vector3d delta = target.col(i) - original.col(i);
      delta *= g_shape.attack_strength_gain;
      if (delta.norm() > g_shape.max_ctrl_offset) delta = delta.normalized() * g_shape.max_ctrl_offset;
      target.col(i) = original.col(i) + delta;
    }

    ShapeOptContext ctx;
    ctx.original = original;
    ctx.target = target;
    ctx.victim_pts = victim_pts;
    ctx.dt = dt_;
    ctx.max_vel = max_vel_check_;
    ctx.max_acc = max_acc_check_;
    ctx.first_free = std::min(3, std::max(0, static_cast<int>(original.cols()) - 1));

    Eigen::MatrixXd optimized = target;
    const int variable_num = 3 * std::max(0, static_cast<int>(optimized.cols()) - ctx.first_free);
    if (variable_num > 0)
    {
      std::vector<double> q(static_cast<size_t>(variable_num));
      for (int i = ctx.first_free; i < optimized.cols(); ++i)
      {
        q[static_cast<size_t>(3 * (i - ctx.first_free))] = optimized(0, i);
        q[static_cast<size_t>(3 * (i - ctx.first_free) + 1)] = optimized(1, i);
        q[static_cast<size_t>(3 * (i - ctx.first_free) + 2)] = optimized(2, i);
      }

      double final_cost = 0.0;
      lbfgs::lbfgs_parameter_t lbfgs_params;
      lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
      lbfgs_params.mem_size = 12;
      lbfgs_params.max_iterations = std::max(5, g_shape.max_iterations);
      lbfgs_params.g_epsilon = 0.01;
      lbfgs::lbfgs_optimize(variable_num, q.data(), &final_cost, shapeCost, NULL, NULL, &ctx, &lbfgs_params);

      for (int i = ctx.first_free; i < optimized.cols(); ++i)
        optimized.col(i) = Eigen::Vector3d(q[static_cast<size_t>(3 * (i - ctx.first_free))],
                                           q[static_cast<size_t>(3 * (i - ctx.first_free) + 1)],
                                           q[static_cast<size_t>(3 * (i - ctx.first_free) + 2)]);
    }

    preserveFakeOffset(
        original,
        target,
        optimized,
        ctx.first_free,
        g_shape.preserve_fake_ratio);

    for (int i = 0; i < optimized.cols(); ++i)
    {
      Eigen::Vector3d delta = optimized.col(i) - original.col(i);
      if (delta.norm() > g_shape.max_ctrl_offset) optimized.col(i) = original.col(i) + delta.normalized() * g_shape.max_ctrl_offset;
      if (grid_map != nullptr && grid_map->getInflateOccupancy(optimized.col(i)))
      {
        for (int k = std::max(1, g_shape.obstacle_backoff_steps); k >= 0; --k)
        {
          const double a = static_cast<double>(k) / static_cast<double>(std::max(1, g_shape.obstacle_backoff_steps));
          Eigen::Vector3d repaired = original.col(i) + a * (optimized.col(i) - original.col(i));
          if (!grid_map->getInflateOccupancy(repaired))
          {
            optimized.col(i) = repaired;
            break;
          }
        }
      }
    }

    traj_utils::Bspline shaped = real_bspline;
    matrixToMsg(optimized, shaped);
    if (controlPointsFree(optimized, grid_map) && curveSamplesFree(optimized, real_bspline, grid_map))
      attacked = shaped;
  }

  /*
   * 关键注意事项：
   * 1. 只改控制点 pos_pts；
   * 2. 不改 drone_id；
   * 3. 不伪造 fake id；
   * 4. 这样不会破坏 EGO-Swarm 的 MultiBsplines traj[drone_id] 结构，
   *    也不会导致攻击机自己把假轨迹当成外部无人机。
   */
  attacked.start_time = now;
  attacked.drone_id = real_bspline.drone_id;
  attacked.traj_id = real_bspline.traj_id;

  applied_offset_ = estimateAppliedOffset(real_bspline, attacked, 3);

  if (attack_scenario_ == SCENARIO_HERDING)
  {
    Eigen::Vector3d desired_push = herding_direction_;
    if (desired_push.norm() > 1e-3)
    {
      desired_push.normalize();
      ROS_WARN_STREAM_THROTTLE(
          0.5,
          "[HerdingPublished] desired_victim_direction=" << desired_push.transpose()
          << " target_offset=" << target_offset_.transpose()
          << " applied_offset=" << applied_offset_.transpose()
          << " (fake trajectory must stay opposite the desired victim direction)");
    }
  }

  if (attack_scenario_ == SCENARIO_OSCILLATION)
  {
    ROS_WARN_STREAM_THROTTLE(
        0.5,
        "[OscillationPublished] target_offset=" << target_offset_.transpose()
        << " applied_offset=" << applied_offset_.transpose()
        << " center=victim_predicted_trajectory");
  }

  last_score_ = score;
  last_physical_check_pass_ = physicalCheck(attacked);

  if (attack_method_ == NO_TEMPORAL_REFRESH && !no_refresh_cached_)
  {
    no_refresh_cache_ = attacked;
    no_refresh_target_offset_ = target_offset_;
    no_refresh_applied_offset_ = applied_offset_;
    no_refresh_cached_ = true;
  }

  ROS_WARN_STREAM_THROTTLE(1.0,
      "[PhantomAttackGenerator] method=" << static_cast<int>(attack_method_)
      << " target_offset=" << target_offset_.transpose()
      << " applied_offset=" << applied_offset_.transpose()
      << " score=" << last_score_
      << " physical_pass=" << last_physical_check_pass_);

  return attacked;
}

double PhantomAttackGenerator::sampleRadiusForRandom()
{
  const double r_min = std::max(0.0, min_offset_radius_);
  const double r_max = std::max(r_min, max_offset_radius_);
  std::uniform_real_distribution<double> radius_dist(r_min, r_max);
  return radius_dist(rng_);
}

Eigen::Vector3d PhantomAttackGenerator::chooseRandomFeasibleOffset(
    const traj_utils::Bspline &real_bspline,
    const GridMap::Ptr &grid_map)
{
  std::uniform_real_distribution<double> angle_dist(0.0, 2.0 * M_PI);

  const int max_trials = std::max(1, random_max_trials_);
  for (int k = 0; k < max_trials; ++k)
  {
    const double theta = angle_dist(rng_);
    const double r = sampleRadiusForRandom();
    Eigen::Vector3d offset(r * std::cos(theta), r * std::sin(theta), fixed_offset_z_);

    if (obstacleFreeAfterOffset(real_bspline, offset, grid_map))
    {
      return offset;
    }
  }

  ROS_WARN("[RandomFake] No local obstacle-free random offset found. Use zero offset.");
  return Eigen::Vector3d::Zero();
}

Eigen::Vector3d PhantomAttackGenerator::chooseNoImpactOffset(
    const traj_utils::Bspline &real_bspline,
    const GridMap::Ptr &grid_map)
{
  Eigen::Vector3d offset(fixed_offset_x_, fixed_offset_y_, fixed_offset_z_);

  /*
   * No impact objective 不针对 victim 优化，但仍尽量保持局部障碍物可行。
   * 如果固定偏移不可行，则缩小半径重试。
   */
  if (obstacleFreeAfterOffset(real_bspline, offset, grid_map)) return offset;

  for (int i = 4; i >= 1; --i)
  {
    Eigen::Vector3d scaled = offset * (static_cast<double>(i) / 5.0);
    if (obstacleFreeAfterOffset(real_bspline, scaled, grid_map)) return scaled;
  }

  ROS_WARN("[NoImpactObjective] Fixed offset not obstacle-free in local map. Use zero offset.");
  return Eigen::Vector3d::Zero();
}

Eigen::Vector3d PhantomAttackGenerator::chooseNaiveInfeasibleOffset()
{
  return 3.0 * Eigen::Vector3d(fixed_offset_x_, fixed_offset_y_, fixed_offset_z_);
}

double PhantomAttackGenerator::scoreSemanticObjective(
    const traj_utils::Bspline &real_bspline,
    const SwarmTrajData &swarm_trajs,
    const Eigen::Vector3d &offset,
    const ros::Time &now) const
{
  if (attack_scenario_ == SCENARIO_RANDOM) return 0.0;

  if (victim_drone_id_ < 0 || victim_drone_id_ >= static_cast<int>(swarm_trajs.size()))
    return 0.0;

  const OneTrajDataOfSwarm &victim = swarm_trajs[victim_drone_id_];
  if (victim.drone_id < 0) return 0.0;

  double score = 0.0;
  int count = 0;

  for (double t = 0.0; t <= horizon_; t += std::max(0.02, dt_))
  {
    Eigen::Vector3d self_p, victim_p;
    if (!evalBsplinePoint(real_bspline, t, self_p)) continue;
    if (!evalSwarmTraj(victim, now, t, victim_p)) continue;

    const Eigen::Vector3d fake_p = self_p + offset;

    /*
     * fake_pressure approximates the direction in which the victim is pushed
     * away by the fake trajectory. The semantic score rewards offsets that
     * push the victim toward the scenario-specific objective.
     */
    Eigen::Vector3d fake_pressure = victim_p - fake_p;
    if (fake_pressure.norm() < 1e-3) continue;
    fake_pressure.normalize();

    if (attack_scenario_ == SCENARIO_HERDING)
    {
      Eigen::Vector3d dir = herding_direction_;
      if (dir.norm() > 1e-3)
      {
        dir.normalize();
        const double align = fake_pressure.dot(dir);
        score += herding_weight_ * std::max(0.0, align);
      }
    }
    else if (attack_scenario_ == SCENARIO_TRAP)
    {
      Eigen::Vector3d to_trap = trap_exit_ - victim_p;
      const double dist_to_trap = to_trap.norm();
      if (dist_to_trap > 1e-3)
      {
        to_trap.normalize();
        const double align = fake_pressure.dot(to_trap);

        score += trap_weight_ * std::max(0.0, align);
        score += 0.5 * trap_weight_ / (dist_to_trap + 1.0);
      }
    }
    else if (attack_scenario_ == SCENARIO_OSCILLATION)
    {
      const Eigen::Vector3d side_dir(0.0, static_cast<double>(oscillation_side_), 0.0);
      const double align = fake_pressure.dot(side_dir);
      score += oscillation_weight_ * std::max(0.0, align);
    }

    ++count;
  }

  if (count == 0) return 0.0;
  return scenario_weight_ * score / static_cast<double>(count);
}

Eigen::Vector3d PhantomAttackGenerator::chooseFullAttackOffset(
    const traj_utils::Bspline &real_bspline,
    const SwarmTrajData &swarm_trajs,
    const ros::Time &now,
    const GridMap::Ptr &grid_map,
    double &best_score)
{
  best_score = -std::numeric_limits<double>::infinity();
  Eigen::Vector3d best = chooseNoImpactOffset(real_bspline, grid_map);

  const bool enforce_herding_side =
      attack_scenario_ == SCENARIO_HERDING &&
      herding_direction_.norm() > 1e-3;
  Eigen::Vector3d herding_dir = Eigen::Vector3d::Zero();
  if (enforce_herding_side) herding_dir = herding_direction_.normalized();

  const int dirs = std::max(1, direction_samples_);
  const int radii = std::max(1, radius_samples_);

  for (int i = 0; i < dirs; ++i)
  {
    const double theta = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(dirs);
    const Eigen::Vector3d dir(std::cos(theta), std::sin(theta), 0.0);

    for (int j = 1; j <= radii; ++j)
    {
      const double r = max_offset_radius_ * static_cast<double>(j) / static_cast<double>(radii);
      Eigen::Vector3d offset = r * dir;
      offset.z() = fixed_offset_z_;

      if (enforce_herding_side)
      {
        double alignment_sum = 0.0;
        int alignment_count = 0;

        if (victim_drone_id_ >= 0 &&
            victim_drone_id_ < static_cast<int>(swarm_trajs.size()) &&
            swarm_trajs[victim_drone_id_].drone_id >= 0)
        {
          for (double t = 0.0; t <= horizon_; t += std::max(0.05, dt_))
          {
            Eigen::Vector3d self_p, victim_p;
            if (!evalBsplinePoint(real_bspline, t, self_p) ||
                !evalSwarmTraj(swarm_trajs[victim_drone_id_], now, t, victim_p))
              continue;

            Eigen::Vector3d pressure = victim_p - (self_p + offset);
            pressure.z() = 0.0;
            if (pressure.norm() < 1e-3) continue;

            alignment_sum += pressure.normalized().dot(herding_dir);
            ++alignment_count;
          }
        }

        if (alignment_count == 0 ||
            alignment_sum / static_cast<double>(alignment_count) <= 0.05)
          continue;
      }

      /* Full Attack 必须保证偏移后的轨迹在攻击 UAV 当前 local grid_map_ 中不撞障碍物。 */
      if (!obstacleFreeAfterOffset(real_bspline, offset, grid_map)) continue;

      const double victim_score = scoreOffsetAgainstVictim(real_bspline, swarm_trajs, offset, now);
      const double semantic_score = scoreSemanticObjective(real_bspline, swarm_trajs, offset, now);
      const double s = victim_score + semantic_score;
      if (s > best_score)
      {
        best_score = s;
        best = offset;
      }
    }
  }

  if (!std::isfinite(best_score))
  {
    if (enforce_herding_side)
    {
      ROS_WARN_THROTTLE(
          1.0,
          "[HerdingAttack] No directionally valid candidate; publish the real trajectory.");
      best_score = 0.0;
      return Eigen::Vector3d::Zero();
    }

    best_score = scoreOffsetAgainstVictim(real_bspline, swarm_trajs, best, now) +
                 scoreSemanticObjective(real_bspline, swarm_trajs, best, now);
  }
  return best;
}

bool PhantomAttackGenerator::obstacleFreeAfterOffset(
    const traj_utils::Bspline &real_bspline,
    const Eigen::Vector3d &offset,
    const GridMap::Ptr &grid_map) const
{
  /*
   * 如果 grid_map 为空，说明当前没有可用相机/深度局部地图。
   * 这里不使用全局地图，也不强行判断障碍物；返回 true 保持系统可运行。
   */
  if (grid_map == nullptr)
  {
    ROS_WARN_THROTTLE(1.0, "[PhantomAttack] local grid_map is null. Skip obstacle check.");
    return true;
  }

  /*
   * 只检查偏移后的控制点是否落入膨胀障碍物。
   * 这与正常 EGO-Swarm 使用本地 grid_map_ 做规划/碰撞判断保持一致。
   */
  for (size_t i = 0; i < real_bspline.pos_pts.size(); ++i)
  {
    Eigen::Vector3d p(
        real_bspline.pos_pts[i].x + offset.x(),
        real_bspline.pos_pts[i].y + offset.y(),
        real_bspline.pos_pts[i].z + offset.z());

    if (grid_map->getInflateOccupancy(p))
    {
      return false;
    }
  }

  return true;
}

bool PhantomAttackGenerator::evalBsplinePoint(
    const traj_utils::Bspline &msg,
    double future_t,
    Eigen::Vector3d &p) const
{
  if (msg.pos_pts.empty()) return false;
  if (msg.pos_pts.size() == 1)
  {
    p = Eigen::Vector3d(msg.pos_pts[0].x, msg.pos_pts[0].y, msg.pos_pts[0].z);
    return true;
  }

  /*
   * 用控制点线性插值近似真实 B-spline，用于攻击打分。
   * 这里不参与真实规划，只用于选择 offset，因此近似足够稳定。
   */
  const double normalized = std::max(0.0, std::min(1.0, future_t / std::max(1e-3, horizon_)));
  const double idx_f = normalized * static_cast<double>(msg.pos_pts.size() - 1);
  const int i0 = std::max(0, std::min(static_cast<int>(std::floor(idx_f)), static_cast<int>(msg.pos_pts.size() - 1)));
  const int i1 = std::max(0, std::min(i0 + 1, static_cast<int>(msg.pos_pts.size() - 1)));
  const double a = idx_f - static_cast<double>(i0);

  Eigen::Vector3d p0(msg.pos_pts[i0].x, msg.pos_pts[i0].y, msg.pos_pts[i0].z);
  Eigen::Vector3d p1(msg.pos_pts[i1].x, msg.pos_pts[i1].y, msg.pos_pts[i1].z);
  p = (1.0 - a) * p0 + a * p1;
  return true;
}

bool PhantomAttackGenerator::evalSwarmTraj(
    const OneTrajDataOfSwarm &traj,
    const ros::Time &now,
    double future_t,
    Eigen::Vector3d &p) const
{
  if (traj.drone_id < 0) return false;

  const double raw_t =
      (now - traj.start_time_).toSec() + future_t;

  if (raw_t < -0.05)
    return false;

  /*
   * A victim may replan less frequently than the attack broadcast.
   * Permit a short grace interval and clamp to the last valid point,
   * rather than declaring the victim unavailable immediately.
   */
  const double grace = 1.0;
  if (raw_t > traj.duration_ + grace)
    return false;

  const double t =
      std::max(0.0, std::min(raw_t, traj.duration_));
  p = traj.position_traj_.evaluateDeBoorT(t);
  return true;
}

double PhantomAttackGenerator::scoreOffsetAgainstVictim(
    const traj_utils::Bspline &real_bspline,
    const SwarmTrajData &swarm_trajs,
    const Eigen::Vector3d &offset,
    const ros::Time &now) const
{
  if (victim_drone_id_ < 0 || victim_drone_id_ >= static_cast<int>(swarm_trajs.size()))
  {
    return -offset_penalty_weight_ * offset.norm();
  }

  const OneTrajDataOfSwarm &victim = swarm_trajs[victim_drone_id_];
  if (victim.drone_id < 0) return -offset_penalty_weight_ * offset.norm();

  double score = 0.0;
  int count = 0;

  for (double t = 0.0; t <= horizon_; t += std::max(0.02, dt_))
  {
    Eigen::Vector3d self_p, victim_p;
    if (!evalBsplinePoint(real_bspline, t, self_p)) continue;
    if (!evalSwarmTraj(victim, now, t, victim_p)) continue;

    const double dist = (self_p + offset - victim_p).norm();
    const double penetration = std::max(0.0, clearance_ - dist);

    /*
     * 距离越近，victim 感知到的邻机冲突压力越大。
     * penetration 项模拟进入安全距离后的避让代价增大。
     */
    score += penetration * penetration + impact_distance_weight_ / (dist + 1e-3);
    ++count;
  }

  if (count == 0) return -offset_penalty_weight_ * offset.norm();
  return score / static_cast<double>(count) - offset_penalty_weight_ * offset.norm();
}

Eigen::Vector3d PhantomAttackGenerator::computeOscillationOffset(
    const traj_utils::Bspline &real_bspline,
    const SwarmTrajData &swarm_trajs,
    const Eigen::Vector3d &odom_pos,
    const Eigen::Vector3d &odom_vel,
    const ros::Time &now,
    const GridMap::Ptr &grid_map,
    double &score) const
{
  score = 0.0;
  (void)grid_map;

  if (victim_drone_id_ < 0 ||
      victim_drone_id_ >= static_cast<int>(swarm_trajs.size()))
  {
    ROS_WARN_THROTTLE(1.0, "[OscillationAttack] Invalid victim id.");
    return Eigen::Vector3d::Zero();
  }

  const OneTrajDataOfSwarm &victim = swarm_trajs[victim_drone_id_];
  Eigen::Vector3d victim_pos;
  if (victim.drone_id < 0 ||
      !evalSwarmTraj(victim, now, 0.0, victim_pos))
  {
    ROS_WARN_STREAM_THROTTLE(
        1.0,
        "[OscillationAttack] victim unavailable"
        << " victim_id=" << victim_drone_id_
        << " stored_id=" << victim.drone_id
        << " age=" << (now - victim.start_time_).toSec()
        << " duration=" << victim.duration_
        << " buffer_size=" << swarm_trajs.size());
    return Eigen::Vector3d::Zero();
  }

  ROS_INFO_STREAM_THROTTLE(
      0.5,
      "[OscillationAttack] victim available"
      << " victim_id=" << victim_drone_id_
      << " age=" << (now - victim.start_time_).toSec()
      << " duration=" << victim.duration_
      << " pos=" << victim_pos.transpose());

  Eigen::Vector3d victim_ahead;
  Eigen::Vector3d tangent = Eigen::Vector3d::Zero();
  if (evalSwarmTraj(victim, now, std::max(0.05, dt_), victim_ahead))
  {
    tangent = victim_ahead - victim_pos;
    tangent.z() = 0.0;
  }

  if (tangent.norm() < 1e-3)
  {
    tangent = odom_vel;
    tangent.z() = 0.0;
  }

  if (tangent.norm() < 1e-3)
    tangent = Eigen::Vector3d::UnitX();

  tangent.normalize();
  const Eigen::Vector3d lateral(-tangent.y(), tangent.x(), 0.0);

  Eigen::Vector3d to_victim = victim_pos - odom_pos;
  to_victim.z() = 0.0;

  Eigen::Vector3d approach = to_victim;
  const double victim_dist = to_victim.norm();

  const double period = std::max(0.2, oscillation_period_);
  const double elapsed = attack_window_enable_
                             ? std::max(0.0, now.toSec() - attack_start_time_)
                             : now.toSec();

  double wave = 0.0;
  if (oscillation_use_sine_)
  {
    wave = std::sin(2.0 * M_PI * elapsed / period);
  }
  else
  {
    const double phase = std::fmod(std::max(0.0, elapsed), period);
    wave = phase < 0.5 * period ? 1.0 : -1.0;
  }

  Eigen::Vector3d desired =
      approach + oscillation_amplitude_ * wave * lateral;
  desired.z() = fixed_offset_z_;

  score = scoreOffsetAgainstVictim(
      real_bspline, swarm_trajs, desired, now);

  ROS_WARN_STREAM_THROTTLE(
      0.5,
      "[OscillationAttack] victim_dist=" << victim_dist
      << " wave=" << wave
      << " lateral=" << lateral.transpose()
      << " desired=" << desired.transpose());

  return desired;
}

traj_utils::Bspline PhantomAttackGenerator::buildVictimCenteredOscillation(
    const traj_utils::Bspline &real_bspline,
    const SwarmTrajData &swarm_trajs,
    const Eigen::Vector3d &odom_pos,
    const ros::Time &now,
    const GridMap::Ptr &grid_map,
    bool &success) const
{
  success = false;
  traj_utils::Bspline out = real_bspline;

  if (victim_drone_id_ < 0 ||
      victim_drone_id_ >= static_cast<int>(swarm_trajs.size()) ||
      swarm_trajs[victim_drone_id_].drone_id < 0 ||
      out.pos_pts.empty())
    return out;

  const OneTrajDataOfSwarm &victim = swarm_trajs[victim_drone_id_];
  Eigen::Vector3d victim_now, victim_ahead;
  if (!evalSwarmTraj(victim, now, 0.0, victim_now) ||
      !evalSwarmTraj(victim, now, std::max(0.05, dt_), victim_ahead))
    return out;

  Eigen::Vector3d tangent_now = victim_ahead - victim_now;
  tangent_now.z() = 0.0;
  if (tangent_now.norm() < 1e-3) tangent_now = Eigen::Vector3d::UnitX();
  tangent_now.normalize();
  const Eigen::Vector3d lateral_now(-tangent_now.y(), tangent_now.x(), 0.0);

  const Eigen::Vector3d current_fake_center = odom_pos + target_offset_;
  double lateral_distance = (current_fake_center - victim_now).dot(lateral_now);
  lateral_distance = std::max(
      -std::abs(oscillation_amplitude_),
      std::min(std::abs(oscillation_amplitude_), lateral_distance));

  const int n = static_cast<int>(out.pos_pts.size());
  const int anchor_count = std::min(3, n);
  const int transition_count = std::max(3, ramp_ctrl_pts_);
  const double victim_age = std::max(0.0, (now - victim.start_time_).toSec());
  const double prediction_horizon = std::max(
      std::max(0.05, dt_),
      std::min(horizon_, std::max(std::max(0.05, dt_), victim.duration_ - victim_age)));

  for (int i = anchor_count; i < n; ++i)
  {
    const double route_ratio = n > 1
                                   ? static_cast<double>(i) / static_cast<double>(n - 1)
                                   : 1.0;
    const double future_t = route_ratio * prediction_horizon;

    Eigen::Vector3d center, center_ahead;
    if (!evalSwarmTraj(victim, now, future_t, center)) continue;
    if (!evalSwarmTraj(victim, now, future_t + std::max(0.05, dt_), center_ahead))
      center_ahead = center;

    Eigen::Vector3d tangent = center_ahead - center;
    tangent.z() = 0.0;
    if (tangent.norm() < 1e-3) tangent = tangent_now;
    tangent.normalize();
    const Eigen::Vector3d lateral(-tangent.y(), tangent.x(), 0.0);

    const Eigen::Vector3d original(
        real_bspline.pos_pts[static_cast<size_t>(i)].x,
        real_bspline.pos_pts[static_cast<size_t>(i)].y,
        real_bspline.pos_pts[static_cast<size_t>(i)].z);

    Eigen::Vector3d side_target = center + lateral_distance * lateral;
    side_target.z() = center.z() + fixed_offset_z_;

    // Keep the published trajectory as the attacker's trajectory plus a
    // per-control-point offset. The offset is chosen so the resulting fake
    // point lies on the selected side of the victim's predicted trajectory.
    const Eigen::Vector3d target_offset = side_target - original;
    const double blend = std::min(
        1.0,
        static_cast<double>(i - anchor_count + 1) /
            static_cast<double>(transition_count));
    Eigen::Vector3d candidate = original + blend * target_offset;

    if (grid_map != nullptr && grid_map->getInflateOccupancy(candidate))
    {
      bool repaired = false;
      for (int k = 9; k >= 0; --k)
      {
        const double a = blend * static_cast<double>(k) / 10.0;
        const Eigen::Vector3d fallback = original + a * target_offset;
        if (!grid_map->getInflateOccupancy(fallback))
        {
          candidate = fallback;
          repaired = true;
          break;
        }
      }
      if (!repaired) candidate = original;
    }

    out.pos_pts[static_cast<size_t>(i)].x = candidate.x();
    out.pos_pts[static_cast<size_t>(i)].y = candidate.y();
    out.pos_pts[static_cast<size_t>(i)].z = candidate.z();
  }

  success = true;
  ROS_WARN_STREAM_THROTTLE(
      0.5,
      "[OscillationVictimCentered] victim_center=" << victim_now.transpose()
      << " lateral_distance=" << lateral_distance
      << " anchored_ctrl_pts=" << anchor_count
      << " construction=attacker_trajectory_plus_offset");
  return out;
}

Eigen::Vector3d PhantomAttackGenerator::smoothOffset(
    const Eigen::Vector3d &desired,
    const ros::Time &now)
{
  if (!has_last_update_time_)
  {
    last_update_time_ = now;
    has_last_update_time_ = true;
    smoothed_offset_.setZero();
  }

  double dt = (now - last_update_time_).toSec();
  if (dt <= 0.0 || dt > 1.0) dt = 0.05;
  last_update_time_ = now;

  const double alpha = std::max(0.0, std::min(1.0, dt / std::max(1e-3, smoothing_time_)));
  Eigen::Vector3d step = alpha * (desired - smoothed_offset_);

  const double max_step = std::max(1e-6, max_offset_rate_ * dt);
  if (step.norm() > max_step) step = step.normalized() * max_step;

  smoothed_offset_ += step;
  return smoothed_offset_;
}

traj_utils::Bspline PhantomAttackGenerator::applyOffsetWithRamp(
    const traj_utils::Bspline &real_bspline,
    const Eigen::Vector3d &offset) const
{
  traj_utils::Bspline out = real_bspline;

  /*
   * Naive infeasible 不做 ramp，直接整体大幅偏移。
   */
  if (attack_method_ == NAIVE_INFEASIBLE)
  {
    for (size_t i = 0; i < out.pos_pts.size(); ++i)
    {
      out.pos_pts[i].x += offset.x();
      out.pos_pts[i].y += offset.y();
      out.pos_pts[i].z += offset.z();
    }
    return out;
  }

  /*
   * 其它方法做控制点渐变：
   * 起点附近偏移小，后段逐渐达到完整偏移，避免发布轨迹出现跳变。
   */
  const int ramp_n = std::max(1, ramp_ctrl_pts_);
  for (size_t i = 0; i < out.pos_pts.size(); ++i)
  {
    double w = 1.0;
    if (static_cast<int>(i) < ramp_n)
    {
      w = static_cast<double>(i) / static_cast<double>(ramp_n);
      w = std::pow(std::max(0.0, std::min(1.0, w)), std::max(0.1, ramp_power_));
    }

    out.pos_pts[i].x += w * offset.x();
    out.pos_pts[i].y += w * offset.y();
    out.pos_pts[i].z += w * offset.z();
  }

  return out;
}

double PhantomAttackGenerator::pointDistance(
    const geometry_msgs::Point &a,
    const geometry_msgs::Point &b) const
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool PhantomAttackGenerator::physicalCheck(const traj_utils::Bspline &msg) const
{
  if (msg.pos_pts.size() < 4) return true;

  const double h = std::max(0.02, dt_);

  for (size_t i = 1; i < msg.pos_pts.size(); ++i)
  {
    const double v = pointDistance(msg.pos_pts[i], msg.pos_pts[i - 1]) / h;
    if (v > max_vel_check_) return false;
  }

  for (size_t i = 2; i < msg.pos_pts.size(); ++i)
  {
    Eigen::Vector3d p0(msg.pos_pts[i - 2].x, msg.pos_pts[i - 2].y, msg.pos_pts[i - 2].z);
    Eigen::Vector3d p1(msg.pos_pts[i - 1].x, msg.pos_pts[i - 1].y, msg.pos_pts[i - 1].z);
    Eigen::Vector3d p2(msg.pos_pts[i].x, msg.pos_pts[i].y, msg.pos_pts[i].z);

    const double a = ((p2 - p1) - (p1 - p0)).norm() / (h * h);
    if (a > max_acc_check_) return false;
  }

  for (size_t i = 3; i < msg.pos_pts.size(); ++i)
  {
    Eigen::Vector3d p0(msg.pos_pts[i - 3].x, msg.pos_pts[i - 3].y, msg.pos_pts[i - 3].z);
    Eigen::Vector3d p1(msg.pos_pts[i - 2].x, msg.pos_pts[i - 2].y, msg.pos_pts[i - 2].z);
    Eigen::Vector3d p2(msg.pos_pts[i - 1].x, msg.pos_pts[i - 1].y, msg.pos_pts[i - 1].z);
    Eigen::Vector3d p3(msg.pos_pts[i].x, msg.pos_pts[i].y, msg.pos_pts[i].z);

    const double j = (p3 - 3.0 * p2 + 3.0 * p1 - p0).norm() / (h * h * h);
    if (j > max_jerk_check_) return false;
  }

  return true;
}

} // namespace ego_planner
