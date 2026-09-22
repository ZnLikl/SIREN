#include <plan_manage/experiment_trial_logger.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace ego_planner
{

namespace
{
std::string derivePerUavPath(const std::string &trial_path)
{
  if (trial_path.empty()) return std::string("/tmp/ego_swarm_per_uav.csv");
  const std::string needle = "trial_summary.csv";
  const std::size_t pos = trial_path.rfind(needle);
  if (pos != std::string::npos)
  {
    return trial_path.substr(0, pos) + "per_uav_summary.csv";
  }
  return trial_path + ".per_uav.csv";
}

std::string deriveTimeseriesPath(const std::string &trial_path)
{
  if (trial_path.empty()) return std::string("/tmp/ego_swarm_timeseries.csv");
  const std::string needle = "trial_summary.csv";
  const std::size_t pos = trial_path.rfind(needle);
  if (pos != std::string::npos)
  {
    return trial_path.substr(0, pos) + "timeseries.csv";
  }
  return trial_path + ".timeseries.csv";
}
}

void ExperimentTrialLogger::init(ros::NodeHandle &nh, int drone_id)
{
  drone_id_ = drone_id;

  nh.param("experiment/log_enable", enabled_, true);
  nh.param<std::string>("experiment/log_file", path_, std::string("/tmp/ego_swarm_trial_summary.csv"));
  nh.param<std::string>("experiment/per_uav_log_file", per_uav_path_, derivePerUavPath(path_));
  nh.param("experiment/per_uav_log_enable", per_uav_enabled_, true);
  nh.param<std::string>("experiment/timeseries_log_file", timeseries_path_, deriveTimeseriesPath(path_));
  nh.param("experiment/timeseries_log_enable", timeseries_enabled_, true);
  nh.param("experiment/timeseries_interval", timeseries_interval_, 0.2);

  // Normalize a wrongly wired legacy launch parameter.  Some older launch
  // files passed the per-UAV filename through experiment/log_file; recover the
  // trial-level filename in the same directory before deriving fragments.
  const std::string per_marker = "per_uav_summary.csv";
  const std::size_t per_pos = path_.find(per_marker);
  if (per_pos != std::string::npos)
  {
    path_.replace(per_pos, per_marker.size(), "trial_summary.csv");
  }

  // The batch runner supplies trial_summary.csv inside the current seed
  // directory. Derive all fragment paths from it so nested launch defaults
  // cannot redirect per-UAV logs to $HOME.
  if (path_.find("trial_summary.csv") != std::string::npos)
  {
    per_uav_path_ = derivePerUavPath(path_);
    timeseries_path_ = deriveTimeseriesPath(path_);
  }

  nh.param<std::string>("experiment/trial_id", trial_id_, std::string("trial_0"));
  nh.param<std::string>("experiment/map", map_name_, std::string("unknown"));
  nh.param("experiment/comm_radius", comm_radius_, -1.0);
  nh.param("experiment/sharing_freq", sharing_freq_, -1.0);
  nh.param("fsm/planning_horizon", planning_horizon_, -1.0);
  nh.param("experiment/success_radius", success_radius_, 0.5);
  nh.param("attack/goal_region_radius", goal_region_radius_, 2.0);
  nh.param("experiment/deadlock_speed_thresh", deadlock_speed_thresh_, 0.12);
  nh.param("experiment/deadlock_goal_dist_thresh", deadlock_goal_dist_thresh_, 1.0);
  nh.param("experiment/velocity_limit", velocity_limit_, -1.0);
  nh.param("experiment/acceleration_limit", acceleration_limit_, -1.0);
  nh.param("experiment/jerk_limit", jerk_limit_, -1.0);
  nh.param("attack/malicious_drone_id", attacker_id_, -1);

  nh.param("experiment/log_owner_drone_id", log_owner_drone_id_, -9999);

}

void ExperimentTrialLogger::setScenarioInfo(const std::string &scenario, const std::string &method, int seed,
                                            int swarm_size, int attacker_count, int victim_id)
{
  scenario_ = scenario;
  method_ = method;
  seed_ = seed;
  swarm_size_ = swarm_size;
  attacker_count_ = attacker_count;
  victim_id_ = victim_id;

  if (log_owner_drone_id_ == -9999 ||
      (log_owner_drone_id_ != attacker_id_ && log_owner_drone_id_ != victim_id_))
  {
    log_owner_drone_id_ = victim_id_ >= 0 ? victim_id_ : attacker_id_;
  }

  const bool attack_participant =
      (attacker_id_ >= 0 && drone_id_ == attacker_id_) ||
      (victim_id_ >= 0 && drone_id_ == victim_id_);
  if (attack_participant)
  {
    start_time_ = ros::Time::now();
    has_start_ = true;

    ROS_INFO_STREAM("[ExperimentTrialLogger] recording drone=" << drone_id_
                    << " role=" << role());

    // Role IDs are unavailable during init(), so create fragments only after
    // setScenarioInfo() confirms this node is the attacker or victim.
    if (enabled_ && per_uav_enabled_)
    {
      const std::string file = perUavFragmentPath();
      const bool empty = !std::ifstream(file.c_str()).good();
      std::ofstream fragment(file.c_str(), std::ios::app);
      if (!fragment.good())
      {
        ROS_ERROR_STREAM("[ExperimentTrialLogger] drone=" << drone_id_
                         << " cannot create per-UAV log file: " << file);
      }
      else if (empty)
      {
        writePerUavHeader(fragment);
      }
    }
  }

  if (trial_id_ == "trial_0")
  {
    std::ostringstream ss;
    ss << scenario_ << "_" << method_ << "_seed" << seed_;
    trial_id_ = ss.str();
  }
}

bool ExperimentTrialLogger::shouldWriteThisDrone() const
{
  const bool attack_participant =
      (attacker_id_ >= 0 && drone_id_ == attacker_id_) ||
      (victim_id_ >= 0 && drone_id_ == victim_id_);
  return enabled_ && attack_participant && drone_id_ == log_owner_drone_id_;
}

std::string ExperimentTrialLogger::role() const
{
  if (drone_id_ == attacker_id_) return "attacker";
  if (drone_id_ == victim_id_) return "victim";
  return "non_victim";
}

std::string ExperimentTrialLogger::perUavFragmentPath() const
{
  std::ostringstream ss;
  ss << per_uav_path_ << ".drone_" << drone_id_ << ".csv";
  return ss.str();
}

std::string ExperimentTrialLogger::timeseriesFragmentPath() const
{
  std::ostringstream ss;
  ss << timeseries_path_ << ".drone_" << drone_id_ << ".csv";
  return ss.str();
}

void ExperimentTrialLogger::updateOdom(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel, const ros::Time &stamp)
{
  if (!enabled_ || !((attacker_id_ >= 0 && drone_id_ == attacker_id_) ||
                     (victim_id_ >= 0 && drone_id_ == victim_id_))) return;

  if (!has_start_)
  {
    start_time_ = stamp;
    has_start_ = true;
    first_pos_ = pos;
  }

  if (!has_last_)
  {
    first_pos_ = pos;
    last_pos_ = pos;
    last_vel_ = vel;
    last_acc_.setZero();
    last_stamp_ = stamp;
    has_last_ = true;
    return;
  }

  const double dt = std::max(1e-3, (stamp - last_stamp_).toSec());
  const Eigen::Vector3d acc = (vel - last_vel_) / dt;

  trajectory_length_ += (pos - last_pos_).norm();
  max_velocity_ = std::max(max_velocity_, vel.norm());
  max_acceleration_ = std::max(max_acceleration_, acc.norm());

  double jerk_norm = 0.0;
  if (has_last_acc_)
  {
    jerk_norm = (acc - last_acc_).norm() / dt;
    max_jerk_ = std::max(max_jerk_, jerk_norm);
  }
  has_last_acc_ = true;

  const bool dyn_bad =
      (velocity_limit_ > 0.0 && vel.norm() > velocity_limit_) ||
      (acceleration_limit_ > 0.0 && acc.norm() > acceleration_limit_) ||
      (jerk_limit_ > 0.0 && jerk_norm > jerk_limit_);
  if (dyn_bad) dynamic_violation_count_++;

  const double deviation = computePathDeviationToGoalLine(pos);
  victim_deviation_max_ = std::max(victim_deviation_max_, deviation);
  victim_deviation_sum_ += deviation;
  victim_deviation_count_++;

  if (has_goal_ && (pos - goal_).norm() < goal_region_radius_)
  {
    const Eigen::Vector3d goal_dir = (goal_ - first_pos_).norm() > 1e-6 ? (goal_ - first_pos_).normalized() : Eigen::Vector3d::UnitX();
    Eigen::Vector3d lateral(-goal_dir.y(), goal_dir.x(), 0.0);
    if (lateral.norm() < 1e-6) lateral = Eigen::Vector3d::UnitY();
    lateral.normalize();
    const double side_value = (pos - goal_).dot(lateral);
    int side_sign = 0;
    if (side_value > 0.15) side_sign = 1;
    if (side_value < -0.15) side_sign = -1;
    if (side_sign != 0 && last_goal_side_sign_ != 0 && side_sign != last_goal_side_sign_)
    {
      goal_oscillation_count_++;
    }
    if (side_sign != 0) last_goal_side_sign_ = side_sign;
  }

  last_pos_ = pos;
  last_vel_ = vel;
  last_acc_ = acc;
  last_stamp_ = stamp;
}

void ExperimentTrialLogger::onPlanningTime(double planning_time_sec)
{
  if (!enabled_ || !((attacker_id_ >= 0 && drone_id_ == attacker_id_) ||
                     (victim_id_ >= 0 && drone_id_ == victim_id_))) return;
  if (planning_time_sec < 0.0) return;
  planning_time_sum_ += planning_time_sec;
  planning_time_max_ = std::max(planning_time_max_, planning_time_sec);
  planning_time_count_++;
}

void ExperimentTrialLogger::onReplan(double j_swarm, bool physical_pass, double attack_score)
{
  if (!enabled_ || !((attacker_id_ >= 0 && drone_id_ == attacker_id_) ||
                     (victim_id_ >= 0 && drone_id_ == victim_id_))) return;

  replan_count_internal_++;
  current_j_swarm_ = j_swarm;

  j_swarm_sum_ += j_swarm;
  j_swarm_max_ = std::max(j_swarm_max_, j_swarm);
  j_swarm_count_++;

  (void)physical_pass;
  (void)attack_score;
}

void ExperimentTrialLogger::updateGoalStatus(const Eigen::Vector3d &goal, bool have_target, const ros::Time &stamp)
{
  if (!enabled_ || !((attacker_id_ >= 0 && drone_id_ == attacker_id_) ||
                     (victim_id_ >= 0 && drone_id_ == victim_id_))) return;

  goal_ = goal;
  has_goal_ = true;

  if (have_target) ever_had_target_ = true;
  if (has_last_)
  {
    last_goal_distance_ = (last_pos_ - goal_).norm();
    if (last_goal_distance_ < success_radius_ || (ever_had_target_ && !have_target))
    {
      mission_success_ = true;
    }
  }

  if (has_last_status_ && has_last_)
  {
    const double dt = std::max(0.0, (stamp - last_status_stamp_).toSec());
    const double speed = last_vel_.norm();
    const bool low_speed = speed < deadlock_speed_thresh_;
    const bool far_from_goal = last_goal_distance_ > deadlock_goal_dist_thresh_;

    if (low_speed && have_target)
    {
      if (scenario_.find("trap") != std::string::npos || scenario_.find("Trap") != std::string::npos)
      {
        trap_dwell_time_ += dt;
      }
      if (far_from_goal)
      {
        deadlock_duration_ += dt;
      }
    }
  }

  last_status_stamp_ = stamp;
  has_last_status_ = true;
}

void ExperimentTrialLogger::updateReplanCounters(int total_replan_count, int collision_replan_count)
{
  if (!enabled_ || !((attacker_id_ >= 0 && drone_id_ == attacker_id_) ||
                     (victim_id_ >= 0 && drone_id_ == victim_id_))) return;
  replan_count_external_ = total_replan_count;
  collision_replan_count_ = collision_replan_count;
}

void ExperimentTrialLogger::updateMinInterUavDistance(double min_distance)
{
  if (!enabled_ || !((attacker_id_ >= 0 && drone_id_ == attacker_id_) ||
                     (victim_id_ >= 0 && drone_id_ == victim_id_))) return;
  if (min_distance < 0.0 || !std::isfinite(min_distance)) return;
  if (min_inter_uav_distance_ < 0.0)
  {
    min_inter_uav_distance_ = min_distance;
  }
  else
  {
    min_inter_uav_distance_ = std::min(min_inter_uav_distance_, min_distance);
  }
}

void ExperimentTrialLogger::updateAttackStatus(bool attack_active,
                                               int attack_method,
                                               const Eigen::Vector3d &target_offset,
                                               const Eigen::Vector3d &applied_offset,
                                               double attack_score,
                                               bool physical_pass)
{
  if (!enabled_ || !((attacker_id_ >= 0 && drone_id_ == attacker_id_) ||
                     (victim_id_ >= 0 && drone_id_ == victim_id_))) return;

  attack_active_ = attack_active;
  attack_method_id_ = attack_method;
  last_target_offset_ = target_offset;
  last_applied_offset_ = applied_offset;
  last_attack_score_ = attack_score;
  last_attack_physical_pass_ = physical_pass;

  if (attack_active_)
  {
    attack_score_sum_ += attack_score;
    attack_score_max_ = std::max(attack_score_max_, attack_score);
    attack_score_count_++;

    physical_total_count_++;
    if (physical_pass)
      physical_pass_count_++;
    else
      detection_flag_ = 1;
  }
}

double ExperimentTrialLogger::computePathDeviationToGoalLine(const Eigen::Vector3d &pos) const
{
  if (!has_goal_) return 0.0;

  const Eigen::Vector3d a = first_pos_;
  const Eigen::Vector3d b = goal_;
  const Eigen::Vector3d ab = b - a;
  const double ab2 = ab.squaredNorm();
  if (ab2 < 1e-6) return 0.0;

  const double u = std::max(0.0, std::min(1.0, (pos - a).dot(ab) / ab2));
  const Eigen::Vector3d projection = a + u * ab;
  return (pos - projection).norm();
}

void ExperimentTrialLogger::writeHeader(std::ofstream &out) const
{
  out << "trial_id,seed,map,scenario,method,swarm_size,attacker_count,attacker_id,victim_id,"
      << "comm_radius,planning_horizon,sharing_freq,max_velocity_limit,max_acceleration_limit,max_jerk_limit,mission_success,completion_time,"
      << "trajectory_length,replanning_count,collision_replanning_count,avg_planning_time,max_planning_time,energy_overhead_proxy,"
      << "J_swarm_avg,J_swarm_max,attack_score_avg,attack_score_max,min_inter_uav_distance,victim_deviation_max,"
      << "victim_deviation_avg,trap_dwell_time,deadlock_duration,goal_oscillation_count,"
      << "max_velocity,max_acceleration,max_jerk,dynamic_violation_count,"
      << "timestamp_deviation,current_position_residual,"
      << "current_velocity_residual,uwb_residual,physical_check_pass,detection_flag,"
      << "last_attack_active,last_attack_method,target_offset_norm,applied_offset_norm\n";
}

void ExperimentTrialLogger::writePerUavHeader(std::ofstream &out) const
{
  out << "trial_id,seed,map,scenario,method,swarm_size,attacker_count,attacker_id,victim_id,"
      << "uav_id,role,mission_success,completion_time,trajectory_length,energy_overhead_proxy,replanning_count,"
      << "collision_replanning_count,avg_planning_time,max_planning_time,J_swarm_avg,J_swarm_max,"
      << "attack_score_avg,attack_score_max,min_inter_uav_distance,victim_deviation_max,victim_deviation_avg,"
      << "trap_dwell_time,deadlock_duration,goal_oscillation_count,max_velocity,max_acceleration,max_jerk,"
      << "dynamic_violation_count,timestamp_deviation,current_position_residual,current_velocity_residual,uwb_residual,"
      << "physical_check_pass,detection_flag,last_attack_active,last_attack_method,target_offset_norm,applied_offset_norm\n";
}

void ExperimentTrialLogger::writeTimeseriesHeader(std::ofstream &out) const
{
  out << "trial_id,seed,map,scenario,method,swarm_size,attacker_count,attacker_id,victim_id,"
      << "uav_id,role,time,mission_success,"
      << "pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,acc_x,acc_y,acc_z,"
      << "speed,acc_norm,goal_x,goal_y,goal_z,goal_distance,path_deviation_to_goal_line,"
      << "trajectory_length,replanning_count,collision_replanning_count,J_swarm_current,J_swarm_avg,J_swarm_max,"
      << "min_inter_uav_distance,max_velocity_so_far,max_acceleration_so_far,max_jerk_so_far,dynamic_violation_count,"
      << "attack_active,attack_method,target_offset_x,target_offset_y,target_offset_z,"
      << "applied_offset_x,applied_offset_y,applied_offset_z,applied_offset_norm,"
      << "attack_score,physical_check_pass,detection_flag,"
      << "trap_dwell_time,deadlock_duration,goal_oscillation_count\n";
}

void ExperimentTrialLogger::writePerUavSnapshot(bool mission_success, double completion_time, int replan_count,
                                                double avg_planning_time, double j_avg, double dev_avg,
                                                double physical_pass) const
{
  if (!enabled_ || !((attacker_id_ >= 0 && drone_id_ == attacker_id_) ||
                     (victim_id_ >= 0 && drone_id_ == victim_id_)) || !per_uav_enabled_) return;

  std::ofstream out(perUavFragmentPath().c_str(), std::ios::out | std::ios::trunc);
  if (!out.good())
  {
    ROS_WARN_STREAM_THROTTLE(2.0, "[ExperimentTrialLogger] Cannot open per-UAV log file: " << perUavFragmentPath());
    return;
  }

  const double attack_score_avg = attack_score_count_ > 0 ? attack_score_sum_ / attack_score_count_ : 0.0;
  writePerUavHeader(out);
  out << std::fixed << std::setprecision(6)
      << trial_id_ << "," << seed_ << "," << map_name_ << "," << scenario_ << "," << method_ << ","
      << swarm_size_ << "," << attacker_count_ << "," << attacker_id_ << "," << victim_id_ << ","
      << drone_id_ << "," << role() << "," << (mission_success ? 1 : 0) << "," << completion_time << ","
      << trajectory_length_ << "," << trajectory_length_ << "," << replan_count << "," << collision_replan_count_ << ","
      << avg_planning_time << "," << planning_time_max_ << "," << j_avg << "," << j_swarm_max_ << ","
      << attack_score_avg << "," << attack_score_max_ << "," << min_inter_uav_distance_ << ","
      << victim_deviation_max_ << "," << dev_avg << "," << trap_dwell_time_ << ","
      << deadlock_duration_ << "," << goal_oscillation_count_ << "," << max_velocity_ << ","
      << max_acceleration_ << "," << max_jerk_ << "," << dynamic_violation_count_ << ","
      << timestamp_deviation_ << "," << current_position_residual_ << ","
      << current_velocity_residual_ << "," << uwb_residual_ << ","
      << physical_pass << "," << detection_flag_ << ","
      << (attack_active_ ? 1 : 0) << "," << attack_method_id_ << ","
      << last_target_offset_.norm() << "," << last_applied_offset_.norm() << "\n";
}

void ExperimentTrialLogger::writeTimeseriesSample(const ros::Time &stamp)
{
  if (!enabled_ || !((attacker_id_ >= 0 && drone_id_ == attacker_id_) ||
                     (victim_id_ >= 0 && drone_id_ == victim_id_)) || !timeseries_enabled_) return;
  if (!has_last_) return;

  if (has_last_timeseries_stamp_)
  {
    const double dt = (stamp - last_timeseries_stamp_).toSec();
    if (dt < timeseries_interval_) return;
  }
  last_timeseries_stamp_ = stamp;
  has_last_timeseries_stamp_ = true;

  const std::string file = timeseriesFragmentPath();
  const bool need_header = !std::ifstream(file.c_str()).good();

  std::ofstream out(file.c_str(), std::ios::out | std::ios::app);
  if (!out.good())
  {
    ROS_WARN_STREAM_THROTTLE(2.0, "[ExperimentTrialLogger] Cannot open timeseries log file: " << file);
    return;
  }
  if (need_header) writeTimeseriesHeader(out);

  const double t = has_start_ ? (stamp - start_time_).toSec() : 0.0;
  const int replan_count = replan_count_external_ >= 0 ? replan_count_external_ : replan_count_internal_;
  const double j_avg = j_swarm_count_ > 0 ? j_swarm_sum_ / j_swarm_count_ : 0.0;
  const double goal_dist = has_goal_ ? (last_pos_ - goal_).norm() : -1.0;
  const double path_dev = computePathDeviationToGoalLine(last_pos_);

  out << std::fixed << std::setprecision(6)
      << trial_id_ << "," << seed_ << "," << map_name_ << "," << scenario_ << "," << method_ << ","
      << swarm_size_ << "," << attacker_count_ << "," << attacker_id_ << "," << victim_id_ << ","
      << drone_id_ << "," << role() << "," << t << "," << (mission_success_ ? 1 : 0) << ","
      << last_pos_.x() << "," << last_pos_.y() << "," << last_pos_.z() << ","
      << last_vel_.x() << "," << last_vel_.y() << "," << last_vel_.z() << ","
      << last_acc_.x() << "," << last_acc_.y() << "," << last_acc_.z() << ","
      << last_vel_.norm() << "," << last_acc_.norm() << ","
      << goal_.x() << "," << goal_.y() << "," << goal_.z() << "," << goal_dist << "," << path_dev << ","
      << trajectory_length_ << "," << replan_count << "," << collision_replan_count_ << ","
      << current_j_swarm_ << "," << j_avg << "," << j_swarm_max_ << ","
      << min_inter_uav_distance_ << "," << max_velocity_ << "," << max_acceleration_ << ","
      << max_jerk_ << "," << dynamic_violation_count_ << ","
      << (attack_active_ ? 1 : 0) << "," << attack_method_id_ << ","
      << last_target_offset_.x() << "," << last_target_offset_.y() << "," << last_target_offset_.z() << ","
      << last_applied_offset_.x() << "," << last_applied_offset_.y() << "," << last_applied_offset_.z() << ","
      << last_applied_offset_.norm() << "," << last_attack_score_ << ","
      << (last_attack_physical_pass_ ? 1 : 0) << "," << detection_flag_ << ","
      << trap_dwell_time_ << "," << deadlock_duration_ << "," << goal_oscillation_count_ << "\n";
  out.flush();
  if (!out.good())
  {
    ROS_ERROR_STREAM_THROTTLE(2.0,
        "[ExperimentTrialLogger] failed while writing timeseries file: " << file);
    return;
  }
  if (!timeseries_first_row_written_)
  {
    timeseries_first_row_written_ = true;
    ROS_INFO_STREAM("[ExperimentTrialLogger] first timeseries row written: " << file);
  }
}

void ExperimentTrialLogger::writeSummary(bool mission_success_hint, const ros::Time &stamp)
{
  if (!enabled_ || !((attacker_id_ >= 0 && drone_id_ == attacker_id_) ||
                     (victim_id_ >= 0 && drone_id_ == victim_id_))) return;

  const double completion_time = has_start_ ? (stamp - start_time_).toSec() : 0.0;
  const int replan_count = replan_count_external_ >= 0 ? replan_count_external_ : replan_count_internal_;
  const double avg_planning_time = planning_time_count_ > 0 ? planning_time_sum_ / planning_time_count_ : 0.0;
  const double j_avg = j_swarm_count_ > 0 ? j_swarm_sum_ / j_swarm_count_ : 0.0;
  const double dev_avg = victim_deviation_count_ > 0 ? victim_deviation_sum_ / victim_deviation_count_ : 0.0;
  const double physical_pass = physical_total_count_ > 0 ? static_cast<double>(physical_pass_count_) / physical_total_count_ : 1.0;
  const double attack_score_avg = attack_score_count_ > 0 ? attack_score_sum_ / attack_score_count_ : 0.0;

  const bool mission_success = mission_success_ || mission_success_hint;

  // Only the attacker and victim write role-level snapshots.
  writePerUavSnapshot(mission_success, completion_time, replan_count, avg_planning_time, j_avg, dev_avg, physical_pass);

  if (!shouldWriteThisDrone()) return;

  std::ofstream out(path_.c_str(), std::ios::out | std::ios::trunc);
  if (!out.good())
  {
    ROS_WARN_STREAM_THROTTLE(2.0, "[ExperimentTrialLogger] Cannot open log file: " << path_);
    return;
  }

  writeHeader(out);
  out << std::fixed << std::setprecision(6)
      << trial_id_ << "," << seed_ << "," << map_name_ << "," << scenario_ << "," << method_ << ","
      << swarm_size_ << "," << attacker_count_ << "," << attacker_id_ << "," << victim_id_ << ","
      << comm_radius_ << "," << planning_horizon_ << "," << sharing_freq_ << ","
      << velocity_limit_ << "," << acceleration_limit_ << "," << jerk_limit_ << ","
      << (mission_success ? 1 : 0) << "," << completion_time << ","
      << trajectory_length_ << "," << replan_count << "," << collision_replan_count_ << ","
      << avg_planning_time << "," << planning_time_max_ << "," << trajectory_length_ << ","
      << j_avg << "," << j_swarm_max_ << ","
      << attack_score_avg << "," << attack_score_max_ << ","
      << min_inter_uav_distance_ << "," << victim_deviation_max_ << "," << dev_avg << ","
      << trap_dwell_time_ << "," << deadlock_duration_ << "," << goal_oscillation_count_ << ","
      << max_velocity_ << "," << max_acceleration_ << "," << max_jerk_ << "," << dynamic_violation_count_ << ","
      << timestamp_deviation_ << "," << current_position_residual_ << ","
      << current_velocity_residual_ << "," << uwb_residual_ << ","
      << physical_pass << "," << detection_flag_ << ","
      << (attack_active_ ? 1 : 0) << "," << attack_method_id_ << ","
      << last_target_offset_.norm() << "," << last_applied_offset_.norm() << "\n";
}

} // namespace ego_planner
