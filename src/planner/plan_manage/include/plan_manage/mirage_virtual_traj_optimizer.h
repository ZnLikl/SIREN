#ifndef _MIRAGE_VIRTUAL_TRAJ_OPTIMIZER_H_
#define _MIRAGE_VIRTUAL_TRAJ_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <vector>

#include <traj_utils/Bspline.h>
#include <plan_manage/planner_manager.h>
#include <plan_env/grid_map.h>

namespace ego_planner
{

/**
 * MirageVirtualTrajOptimizer
 *
 * This module implements the paper-style MIRAGE virtual trajectory generator:
 *
 *   victim-centered reference sampling
 *        -> regularized B-spline least-squares fitting
 *        -> L-BFGS refinement over B-spline control points
 *        -> physically plausible poisoned Bspline message
 *
 * It is intentionally independent from the normal EGO planner optimizer.  The
 * attacker does not change its executed trajectory; it only optimizes the
 * trajectory message that will be broadcast to the swarm.
 */
class MirageVirtualTrajOptimizer
{
public:
  struct Config
  {
    int victim_drone_id = -1;
    int scenario = 0;

    int order = 3;
    double horizon = 5.0;
    double dt = 0.1;
    double clearance = 0.5;
    double max_offset_radius = 5.0;

    double max_vel = 3.0;
    double max_acc = 4.0;
    double max_jerk = 8.0;

    double lambda_smooth = 1.0;
    double lambda_fit = 2.0;
    double lambda_dynamic = 5.0;
    double lambda_phys = 30.0;
    double lambda_attack = 8.0;

    double fitting_rho = 0.05;
    int max_iterations = 80;
    int sample_num = 24;

    Eigen::Vector3d herding_direction = Eigen::Vector3d(0.0, 1.0, 0.0);
    Eigen::Vector3d goal_point = Eigen::Vector3d(0.0, 0.0, 1.0);
    bool use_manual_goal = false;
    double goal_region_radius = 2.0;
    double oscillation_period = 2.0;
  };

  MirageVirtualTrajOptimizer() = default;
  explicit MirageVirtualTrajOptimizer(const Config &cfg) : cfg_(cfg) {}

  void setConfig(const Config &cfg) { cfg_ = cfg; }

  bool generate(
      const traj_utils::Bspline &real_bspline,
      const SwarmTrajData &swarm_trajs,
      const Eigen::Vector3d &attacker_pos,
      const Eigen::Vector3d &attacker_vel,
      const ros::Time &now,
      const GridMap::Ptr &grid_map,
      traj_utils::Bspline &poisoned_bspline,
      double &final_cost);

private:
  struct OptData
  {
    MirageVirtualTrajOptimizer *self = nullptr;
    Eigen::MatrixXd A;
    Eigen::MatrixXd ref;     // L x 3, fitting reference points
    Eigen::MatrixXd target;  // L x 3, attack target points
    Eigen::Vector3d p0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d v0 = Eigen::Vector3d::Zero();
    double dt_ctrl = 0.1;
    int n_ctrl = 0;
  };

  bool buildReference(
      const traj_utils::Bspline &real_bspline,
      const SwarmTrajData &swarm_trajs,
      const Eigen::Vector3d &attacker_pos,
      const ros::Time &now,
      Eigen::MatrixXd &ref,
      Eigen::MatrixXd &target) const;

  bool fitInitialControlPoints(
      const Eigen::MatrixXd &ref,
      int n_ctrl,
      int degree,
      Eigen::MatrixXd &ctrl_pts,
      Eigen::MatrixXd &basis_A) const;

  bool optimizeControlPoints(
      Eigen::MatrixXd &ctrl_pts,
      const Eigen::MatrixXd &basis_A,
      const Eigen::MatrixXd &ref,
      const Eigen::MatrixXd &target,
      const Eigen::Vector3d &p0,
      const Eigen::Vector3d &v0,
      double dt_ctrl,
      double &final_cost);

  traj_utils::Bspline buildMsgFromControlPoints(
      const traj_utils::Bspline &real_bspline,
      const Eigen::MatrixXd &ctrl_pts,
      const ros::Time &now) const;

  bool obstacleFree(const Eigen::MatrixXd &ctrl_pts, const GridMap::Ptr &grid_map) const;

  static double lbfgsEvaluate(void *instance, const double *x, double *g, const int n);

  double evaluateCost(const Eigen::VectorXd &x, Eigen::VectorXd &grad, const OptData &data) const;

  bool evalSwarmTraj(
      const OneTrajDataOfSwarm &traj,
      const ros::Time &now,
      double future_t,
      Eigen::Vector3d &p) const;

  bool evalBsplineMsgPoint(
      const traj_utils::Bspline &msg,
      double future_t,
      Eigen::Vector3d &p) const;

  Eigen::MatrixXd buildOpenUniformBasis(int sample_num, int n_ctrl, int degree) const;
  double basisValue(int i, int p, double u, const std::vector<double> &knots) const;
  Eigen::MatrixXd secondDifferenceMatrix(int n_ctrl) const;

private:
  Config cfg_;
};

} // namespace ego_planner

#endif
