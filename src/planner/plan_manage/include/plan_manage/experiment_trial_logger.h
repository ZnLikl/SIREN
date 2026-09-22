#ifndef EXPERIMENT_TRIAL_LOGGER_H_
#define EXPERIMENT_TRIAL_LOGGER_H_

#include <Eigen/Eigen>
#include <fstream>
#include <ros/ros.h>
#include <string>

namespace ego_planner
{

/**
 * ExperimentTrialLogger
 *
 * Trial-level logger + attacker/victim logger for MIRAGE experiments.
 *
 * Outputs:
 * 1. trial_summary.csv: one row per trial, written only by log_owner_drone_id
 *    (default: victim UAV). This is used for Table II/III/IV style metrics.
 * 2. per_uav_summary.csv.drone_<id>.csv: one row for the attacker and one row
 *    for the victim. Other UAVs do not collect or write experiment data.
 */
class ExperimentTrialLogger
{
public:
  void init(ros::NodeHandle &nh, int drone_id);

  void setScenarioInfo(const std::string &scenario, const std::string &method, int seed,
                       int swarm_size, int attacker_count, int victim_id);

  void updateOdom(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel, const ros::Time &stamp);

  void onPlanningTime(double planning_time_sec);

  void onReplan(double j_swarm, bool physical_pass, double attack_score);

  void updateGoalStatus(const Eigen::Vector3d &goal, bool have_target, const ros::Time &stamp);

  void updateReplanCounters(int total_replan_count, int collision_replan_count);

  void updateMinInterUavDistance(double min_distance);

  // Attack-state snapshot from PhantomAttackGenerator.
  // This is written to timeseries and summary so paper plots do not need ROS logs.
  void updateAttackStatus(bool attack_active,
                          int attack_method,
                          const Eigen::Vector3d &target_offset,
                          const Eigen::Vector3d &applied_offset,
                          double attack_score,
                          bool physical_pass);

  // Periodic process-level logger for plotting curves.
  void writeTimeseriesSample(const ros::Time &stamp);

  void writeSummary(bool mission_success_hint, const ros::Time &stamp);

private:
  std::string path_;
  std::string per_uav_path_;
  std::string timeseries_path_;
  bool per_uav_enabled_ = true;
  bool timeseries_enabled_ = true;
  double timeseries_interval_ = 0.2;
  ros::Time last_timeseries_stamp_;
  bool has_last_timeseries_stamp_ = false;
  bool timeseries_first_row_written_ = false;
  bool enabled_ = false;
  int drone_id_ = -1;
  int log_owner_drone_id_ = -9999;

  std::string trial_id_ = "trial_0";
  std::string map_name_ = "unknown";
  std::string scenario_ = "unknown";
  std::string method_ = "unknown";
  int seed_ = 0;
  int swarm_size_ = 0;
  int attacker_count_ = 0;
  int victim_id_ = -1;
  int attacker_id_ = -1;

  double comm_radius_ = -1.0;
  double planning_horizon_ = -1.0;
  double sharing_freq_ = -1.0;

  ros::Time start_time_;
  bool has_start_ = false;

  Eigen::Vector3d first_pos_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_pos_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_vel_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_acc_ = Eigen::Vector3d::Zero();
  ros::Time last_stamp_;
  bool has_last_ = false;
  bool has_last_acc_ = false;

  Eigen::Vector3d goal_ = Eigen::Vector3d::Zero();
  bool has_goal_ = false;
  bool ever_had_target_ = false;
  bool mission_success_ = false;
  double last_goal_distance_ = 1e9;
  double success_radius_ = 0.5;
  double goal_region_radius_ = 2.0;
  double deadlock_speed_thresh_ = 0.12;
  double deadlock_goal_dist_thresh_ = 1.0;

  double trajectory_length_ = 0.0;
  int replan_count_internal_ = 0;
  int replan_count_external_ = -1;
  int collision_replan_count_ = 0;

  double planning_time_sum_ = 0.0;
  double planning_time_max_ = 0.0;
  int planning_time_count_ = 0;

  double j_swarm_sum_ = 0.0;
  double j_swarm_max_ = 0.0;
  int j_swarm_count_ = 0;

  double victim_deviation_max_ = 0.0;
  double victim_deviation_sum_ = 0.0;
  int victim_deviation_count_ = 0;

  double min_inter_uav_distance_ = -1.0;

  double trap_dwell_time_ = 0.0;
  double deadlock_duration_ = 0.0;
  int goal_oscillation_count_ = 0;
  int last_goal_side_sign_ = 0;

  double max_velocity_ = 0.0;
  double max_acceleration_ = 0.0;
  double max_jerk_ = 0.0;
  double velocity_limit_ = -1.0;
  double acceleration_limit_ = -1.0;
  double jerk_limit_ = -1.0;
  int dynamic_violation_count_ = 0;

  bool attack_active_ = false;
  int attack_method_id_ = 0;
  Eigen::Vector3d last_target_offset_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_applied_offset_ = Eigen::Vector3d::Zero();
  double last_attack_score_ = 0.0;
  bool last_attack_physical_pass_ = true;
  double current_j_swarm_ = 0.0;

  double attack_score_sum_ = 0.0;
  double attack_score_max_ = 0.0;
  int attack_score_count_ = 0;
  int physical_pass_count_ = 0;
  int physical_total_count_ = 0;

  double timestamp_deviation_ = 0.0;
  double current_position_residual_ = 0.0;
  double current_velocity_residual_ = 0.0;
  double uwb_residual_ = 0.0;
  int detection_flag_ = 0;

  ros::Time last_status_stamp_;
  bool has_last_status_ = false;

private:
  bool shouldWriteThisDrone() const;
  std::string role() const;
  std::string perUavFragmentPath() const;
  std::string timeseriesFragmentPath() const;
  double computePathDeviationToGoalLine(const Eigen::Vector3d &pos) const;
  void writeHeader(std::ofstream &out) const;
  void writePerUavHeader(std::ofstream &out) const;
  void writeTimeseriesHeader(std::ofstream &out) const;
  void writePerUavSnapshot(bool mission_success, double completion_time, int replan_count,
                           double avg_planning_time, double j_avg, double dev_avg,
                           double physical_pass) const;
};

} // namespace ego_planner

#endif
