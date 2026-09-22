#ifndef _PHANTOM_ATTACK_GENERATOR_H_
#define _PHANTOM_ATTACK_GENERATOR_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <random>
#include <string>

#include <traj_utils/Bspline.h>
#include <plan_manage/planner_manager.h>
#include <plan_env/grid_map.h>

namespace ego_planner
{

class PhantomAttackGenerator
{
public:
  enum AttackMethod
  {
    BENIGN = 0,
    RANDOM_FAKE = 1,
    NO_TEMPORAL_REFRESH = 2,
    NO_IMPACT_OBJECTIVE = 3,
    FULL_ATTACK = 4,
    NAIVE_INFEASIBLE = 5
  };

  enum AttackScenario
  {
    SCENARIO_RANDOM = 0,
    SCENARIO_HERDING = 1,
    SCENARIO_TRAP = 2,
    SCENARIO_OSCILLATION = 3
  };

  PhantomAttackGenerator();

  void init(ros::NodeHandle &nh);

  bool shouldAttack(int drone_id) const;

  traj_utils::Bspline generate(
      const traj_utils::Bspline &real_bspline,
      const SwarmTrajData &swarm_trajs,
      const Eigen::Vector3d &odom_pos,
      const Eigen::Vector3d &odom_vel,
      int self_drone_id,
      const ros::Time &now,
      const GridMap::Ptr &grid_map);

  Eigen::Vector3d lastTargetOffset() const { return target_offset_; }
  Eigen::Vector3d lastAppliedOffset() const { return applied_offset_; }
  double lastScore() const { return last_score_; }
  bool lastPhysicalCheckPass() const { return last_physical_check_pass_; }
  AttackMethod method() const { return attack_method_; }

private:
  AttackMethod parseMethod(const std::string &name, int id) const;
  bool isWithinAttackWindow(const ros::Time &now) const;

  Eigen::Vector3d chooseRandomFeasibleOffset(
      const traj_utils::Bspline &real_bspline,
      const GridMap::Ptr &grid_map);

  Eigen::Vector3d chooseNoImpactOffset(
      const traj_utils::Bspline &real_bspline,
      const GridMap::Ptr &grid_map);

  Eigen::Vector3d chooseNaiveInfeasibleOffset();

  Eigen::Vector3d chooseFullAttackOffset(
      const traj_utils::Bspline &real_bspline,
      const SwarmTrajData &swarm_trajs,
      const ros::Time &now,
      const GridMap::Ptr &grid_map,
      double &best_score);

  bool obstacleFreeAfterOffset(
      const traj_utils::Bspline &real_bspline,
      const Eigen::Vector3d &offset,
      const GridMap::Ptr &grid_map) const;

  bool evalBsplinePoint(
      const traj_utils::Bspline &msg,
      double future_t,
      Eigen::Vector3d &p) const;

  bool evalSwarmTraj(
      const OneTrajDataOfSwarm &traj,
      const ros::Time &now,
      double future_t,
      Eigen::Vector3d &p) const;

  double scoreOffsetAgainstVictim(
      const traj_utils::Bspline &real_bspline,
      const SwarmTrajData &swarm_trajs,
      const Eigen::Vector3d &offset,
      const ros::Time &now) const;

  double scoreSemanticObjective(
      const traj_utils::Bspline &real_bspline,
      const SwarmTrajData &swarm_trajs,
      const Eigen::Vector3d &offset,
      const ros::Time &now) const;

  Eigen::Vector3d computeOscillationOffset(
      const traj_utils::Bspline &real_bspline,
      const SwarmTrajData &swarm_trajs,
      const Eigen::Vector3d &odom_pos,
      const Eigen::Vector3d &odom_vel,
      const ros::Time &now,
      const GridMap::Ptr &grid_map,
      double &score) const;

  traj_utils::Bspline buildVictimCenteredOscillation(
      const traj_utils::Bspline &real_bspline,
      const SwarmTrajData &swarm_trajs,
      const Eigen::Vector3d &odom_pos,
      const ros::Time &now,
      const GridMap::Ptr &grid_map,
      bool &success) const;

  Eigen::Vector3d smoothOffset(
      const Eigen::Vector3d &desired,
      const ros::Time &now);

  traj_utils::Bspline applyOffsetWithRamp(
      const traj_utils::Bspline &real_bspline,
      const Eigen::Vector3d &offset) const;

  double pointDistance(
      const geometry_msgs::Point &a,
      const geometry_msgs::Point &b) const;

  bool physicalCheck(const traj_utils::Bspline &msg) const;

  double sampleRadiusForRandom();

private:
  bool attack_enable_{false};
  AttackMethod attack_method_{BENIGN};
  int malicious_drone_id_{-1};
  int victim_drone_id_{-1};

  AttackScenario attack_scenario_{SCENARIO_RANDOM};
  double scenario_weight_{1.0};

  double herding_weight_{3.0};
  Eigen::Vector3d herding_direction_{0.0, -1.0, 0.0};

  double trap_weight_{4.0};
  Eigen::Vector3d trap_exit_{5.0, -7.0, 1.0};

  double oscillation_weight_{3.0};
  double oscillation_period_{4.0};
  double oscillation_amplitude_{0.8};
  double oscillation_approach_offset_{0.5};
  bool oscillation_use_sine_{true};
  int oscillation_side_{1};
  ros::Time last_oscillation_switch_time_;

  bool attack_window_enable_{false};
  double attack_start_time_{0.0};
  double attack_duration_{1e9};

  double fixed_offset_x_{1.5};
  double fixed_offset_y_{2.0};
  double fixed_offset_z_{0.0};
  double max_offset_radius_{2.5};
  double min_offset_radius_{0.2};
  int direction_samples_{16};
  int radius_samples_{5};
  int random_max_trials_{50};

  double horizon_{5.0};
  double dt_{0.1};
  double clearance_{0.5};
  double impact_distance_weight_{0.05};
  double offset_penalty_weight_{0.02};

  bool continuity_enable_{true};
  double smoothing_time_{1.0};
  double max_offset_rate_{0.8};
  int ramp_ctrl_pts_{5};
  double ramp_power_{1.0};

  double max_vel_check_{3.0};
  double max_acc_check_{4.0};
  double max_jerk_check_{8.0};
  double min_applied_offset_ratio_{0.7};

  Eigen::Vector3d target_offset_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d applied_offset_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d smoothed_offset_{Eigen::Vector3d::Zero()};
  ros::Time last_update_time_;
  bool has_last_update_time_{false};

  bool no_refresh_cached_{false};
  traj_utils::Bspline no_refresh_cache_;
  Eigen::Vector3d no_refresh_target_offset_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d no_refresh_applied_offset_{Eigen::Vector3d::Zero()};

  double last_score_{0.0};
  bool last_physical_check_pass_{true};

  std::mt19937 rng_;
};

} // namespace ego_planner

#endif
