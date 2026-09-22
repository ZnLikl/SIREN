// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>
#include "visualization_msgs/Marker.h" // zx-todo

namespace ego_planner
{

  // SECTION interfaces for setup and query

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() {}

  void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    /* read algorithm parameters */

    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);
    nh.param("manager/use_distinctive_trajs", pp_.use_distinctive_trajs, false);
    nh.param("manager/drone_id", pp_.drone_id, -1);

    local_data_.traj_id_ = 0;
    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    // obj_predictor_.reset(new fast_planner::ObjPredictor(nh));
    // obj_predictor_->init();
    // obj_pub_ = nh.advertise<visualization_msgs::Marker>("/dynamic/obj_prdi", 10); // zx-todo

    bspline_optimizer_.reset(new BsplineOptimizer);
    bspline_optimizer_->setParam(nh);
    bspline_optimizer_->setEnvironment(grid_map_, obj_predictor_);
    bspline_optimizer_->a_star_.reset(new AStar);
    bspline_optimizer_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

    visualization_ = vis;

    std::string log_path;
    std::string home = std::getenv("HOME") ? std::getenv("HOME") : "/tmp";
    std::string default_path = home + "/ego_log_uav" + std::to_string(pp_.drone_id) + ".csv";
    nh.param("exp_log_file", log_path, default_path);
    exp_logger_.init(log_path);
    ROS_INFO_STREAM("[ExpLogger] logging to: " << log_path);
  }

  // !SECTION

  // SECTION rebond replanning

  bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {
    static int count = 0;
    printf("\033[47;30m\n[drone %d replan %d]==============================================\033[0m\n", pp_.drone_id, count++);
    // cout.precision(3);
    // cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
    //      << endl;

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
      continous_failures_count_++;
      return false;
    }

    bspline_optimizer_->setLocalTargetPt(local_target_pt);

    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt, t_refine;

    /*** STEP 1: INIT ***/
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.5 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
    do
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
      {
        flag_first_call = false;
        flag_force_polynomial = false;

        PolynomialTraj gl_traj;

        double dist = (start_pt - local_target_pt).norm();
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

        if (!flag_randomPolyTraj)
        {
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        else
        {
          Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        double t;
        bool flag_too_far;
        ts *= 1.5; // ts will be divided by 1.5 in the next
        do
        {
          ts /= 1.5;
          point_set.clear();
          flag_too_far = false;
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        t -= ts;
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
      }
      else // Initial path generated from previous trajectory.
      {

        double t;
        double t_cur = (ros::Time::now() - local_data_.start_time_).toSec();

        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
          if (t > t_cur)
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;

        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (poly_time > ts)
        {
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              ROS_ERROR("pseudo_arc_length is empty, return!");
              continous_failures_count_++;
              return false;
            }
          }
        }

        double sample_length = 0;
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        size_t id = 0;
        do
        {
          cps_dist /= 1.5;
          point_set.clear();
          sample_length = 0;
          id = 0;
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          point_set.push_back(local_target_pt);
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help

        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3) // The initial path is unnormally too long!
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    Eigen::MatrixXd ctrl_pts, ctrl_pts_temp;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    vector<std::pair<int, int>> segments;
    segments = bspline_optimizer_->initControlPoints(ctrl_pts, true);

    t_init = ros::Time::now() - t_start;
    t_start = ros::Time::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_step_1_success = false;
    vector<vector<Eigen::Vector3d>> vis_trajs;

    if (pp_.use_distinctive_trajs)
    {
      // cout << "enter" << endl;
      std::vector<ControlPoints> trajs = bspline_optimizer_->distinctiveTrajs(segments);
      cout << "\033[1;33m"
           << "multi-trajs=" << trajs.size() << "\033[1;0m" << endl;

      double final_cost, min_cost = 999999.0;
      for (int i = trajs.size() - 1; i >= 0; i--)
      {
        if (bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts_temp, final_cost, trajs[i], ts))
        {

          cout << "traj " << trajs.size() - i << " success." << endl;

          flag_step_1_success = true;
          if (final_cost < min_cost)
          {
            min_cost = final_cost;
            ctrl_pts = ctrl_pts_temp;
          }

          // visualization
          point_set.clear();
          for (int j = 0; j < ctrl_pts_temp.cols(); j++)
          {
            point_set.push_back(ctrl_pts_temp.col(j));
          }
          vis_trajs.push_back(point_set);
        }
        else
        {
          cout << "traj " << trajs.size() - i << " failed." << endl;
        }
      }

      t_opt = ros::Time::now() - t_start;

      visualization_->displayMultiInitPathList(vis_trajs, 0.2); // This visuallization will take up several milliseconds.
    }
    else
    {
      flag_step_1_success = bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
      t_opt = ros::Time::now() - t_start;
      //static int vis_id = 0;
      visualization_->displayInitPathList(point_set, 0.2, 0);
    }

    cout << "plan_success=" << flag_step_1_success << endl;
    if (!flag_step_1_success)
    {
      visualization_->displayOptimalList(ctrl_pts, 0);
      continous_failures_count_++;
      
      ExpStats s_fail;
      s_fail.drone_id = pp_.drone_id;
      s_fail.plan_seq = plan_seq_++;
      s_fail.stamp = ros::Time::now().toSec();

      s_fail.t_init = t_init.toSec();
      s_fail.t_opt = t_opt.toSec();
      s_fail.t_refine = 0.0;
      s_fail.t_total = (t_init + t_opt).toSec();

      s_fail.success = false;

      s_fail.J_total = bspline_optimizer_->getLastTotalCost();
      s_fail.J_smooth = bspline_optimizer_->getLastSmoothCost();
      s_fail.J_feas = bspline_optimizer_->getLastFeasCost();
      s_fail.J_swarm = bspline_optimizer_->getLastSwarmCost();
      s_fail.J_terminal = bspline_optimizer_->getLastTerminalCost();
      s_fail.min_ellip_dist = bspline_optimizer_->getLastMinEllipDist();

      exp_logger_.log(s_fail);    
      return false;
    }

    t_start = ros::Time::now();

    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    // Note: Only adjust time in single drone mode. But we still allow drone_0 to adjust its time profile.
    if (pp_.drone_id <= 0)
    {

      double ratio;
      bool flag_step_2_success = true;
      if (!pos.checkFeasibility(ratio, false))
      {
        cout << "Need to reallocate time." << endl;

        Eigen::MatrixXd optimal_control_points;
        flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
        if (flag_step_2_success)
          pos = UniformBspline(optimal_control_points, 3, ts);
      }

      if (!flag_step_2_success)
      {
        printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appeares occasionally. But if continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
        continous_failures_count_++;
        
        // ====== 记录轨迹精化失败日志 ======
        ExpStats s_refine_fail;
        s_refine_fail.drone_id = pp_.drone_id;
        s_refine_fail.plan_seq = plan_seq_++;
        s_refine_fail.stamp = ros::Time::now().toSec();

        s_refine_fail.t_init = t_init.toSec();
        s_refine_fail.t_opt = t_opt.toSec();
        s_refine_fail.t_refine = t_refine.toSec();
        s_refine_fail.t_total = (t_init + t_opt + t_refine).toSec();

        s_refine_fail.success = false;

        s_refine_fail.J_total = bspline_optimizer_->getLastTotalCost();
        s_refine_fail.J_smooth = bspline_optimizer_->getLastSmoothCost();
        s_refine_fail.J_feas = bspline_optimizer_->getLastFeasCost();
        s_refine_fail.J_swarm = bspline_optimizer_->getLastSwarmCost();
        s_refine_fail.J_terminal = bspline_optimizer_->getLastTerminalCost();
        s_refine_fail.min_ellip_dist = bspline_optimizer_->getLastMinEllipDist();

        exp_logger_.log(s_refine_fail);
        
        return false;
      }
    }
    else
    {
      static bool print_once = true;
      if (print_once)
      {
        print_once = false;
        ROS_ERROR("IN SWARM MODE, REFINE DISABLED!");
      }
    }

    t_refine = ros::Time::now() - t_start;

    // save planned results
    updateTrajInfo(pos, ros::Time::now());

    static double sum_time = 0;
    static int count_success = 0;
    sum_time += (t_init + t_opt + t_refine).toSec();
    count_success++;
    cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << ",avg_time=" << sum_time / count_success << endl;

    // 假设你用 bspline_optimizer_ 存了各 cost（后面第 3 部分会讲怎么改）
    double J_total      = bspline_optimizer_->getLastTotalCost();
    double J_smooth     = bspline_optimizer_->getLastSmoothCost();
    double J_feas       = bspline_optimizer_->getLastFeasCost();
    double J_swarm      = bspline_optimizer_->getLastSwarmCost();
    double J_terminal   = bspline_optimizer_->getLastTerminalCost();
    double min_ellip_dist = bspline_optimizer_->getLastMinEllipDist();

    // ====== 2. 写一条日志 ======
    ExpStats s;
    ros::NodeHandle pnh("~");
    int mal_type = pnh.param("mal_type", 0);

    s.drone_id = pp_.drone_id;
    s.plan_seq = plan_seq_++;
    s.stamp    = ros::Time::now().toSec();

    s.t_init   = t_init.toSec();
    s.t_opt    = t_opt.toSec();
    s.t_refine = t_refine.toSec();
    s.t_total  = (t_init + t_opt + t_refine).toSec();

    s.success = true;

    s.J_total        = J_total;
    s.J_smooth       = J_smooth;
    s.J_feas         = J_feas;
    s.J_swarm        = J_swarm;
    s.J_terminal     = J_terminal;
    s.min_ellip_dist = min_ellip_dist;
    s.attack_type = mal_type;

    exp_logger_.log(s);
    // success. YoY
    continous_failures_count_ = 0;
    return true;
  }

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), ros::Time::now());

    return true;
  }

  bool EGOPlannerManager::checkCollision(int drone_id)
  {
    if (local_data_.start_time_.toSec() < 1e9) // It means my first planning has not started
      return false;

    double my_traj_start_time = local_data_.start_time_.toSec();
    double other_traj_start_time = swarm_trajs_buf_[drone_id].start_time_.toSec();

    double t_start = max(my_traj_start_time, other_traj_start_time);
    double t_end = min(my_traj_start_time + local_data_.duration_ * 2 / 3, other_traj_start_time + swarm_trajs_buf_[drone_id].duration_);

    for (double t = t_start; t < t_end; t += 0.03)
    {
      if ((local_data_.position_traj_.evaluateDeBoorT(t - my_traj_start_time) - swarm_trajs_buf_[drone_id].position_traj_.evaluateDeBoorT(t - other_traj_start_time)).norm() < bspline_optimizer_->getSwarmClearance())
      {
        return true;
      }
    }

    return false;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    total_len += (start_pos - waypoints[0]).norm();
    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {
      total_len += (waypoints[i + 1] - waypoints[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // for ( int i=0; i<inter_points.size(); i++ )
    // {
    //   cout << inter_points[i].transpose() << endl;
    // }

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  void EGOPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

  void EGOPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;
    // double length = bspline.getLength(0.1);
    // int seg_num = ceil(length / pp_.ctrl_pt_dist);

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

  // bool EGOPlannerManager::planTargetAdversarialPhantomTraj(traj_utils::Bspline &phantom, double horizon, double dt, int phantom_target_drone_id, Eigen::Vector3d start_pos)
  // {
  //   if (swarm_trajs_buf_.empty() || phantom_target_drone_id < 0) return false;

  //   const int order = 3;
  //   if (dt <= 1e-4) dt = 0.1;
  //   int N_sample = static_cast<int>(std::round(horizon / dt));
  //   N_sample = std::max(N_sample, 10);

  //   Eigen::MatrixXd init_points(3, N_sample);
  //   ros::Time now = ros::Time::now();

  //   if (phantom_target_drone_id >= (int)swarm_trajs_buf_.size() || swarm_trajs_buf_[phantom_target_drone_id].drone_id < 0) return false;
  //   const auto& target_traj_data = swarm_trajs_buf_[phantom_target_drone_id];

  //   for (int k = 0; k < N_sample; ++k)
  //   {
  //     double t_global = now.toSec() + k * dt;
  //     double t_local_target = std::max(0.0, std::min(t_global - target_traj_data.start_time_.toSec(), target_traj_data.duration_));
      
  //     Eigen::Vector3d p_target;
  //     try {
  //         p_target = target_traj_data.position_traj_.evaluateDeBoorT(t_local_target);
  //     } catch (...) {
  //         p_target = target_traj_data.start_pos_;
  //     }
      
  //     // 核心修正：从幽灵当前的投影起点 (start_pos) 向 目标位置 (p_target) 进行 8 点平滑插值
  //     if (k < 8) {
  //         double alpha = static_cast<double>(k) / 8.0;
  //         init_points.col(k) = (1.0 - alpha) * start_pos + alpha * p_target;
  //     } else {
  //         init_points.col(k) = p_target;
  //     }
  //   }
    
  //   Eigen::MatrixXd opt_points;
  //   if (bspline_optimizer_->BsplineOptimizeTrajAdversarial(init_points, dt, opt_points))
  //   {
  //     UniformBspline phantom_bspline(opt_points, order, dt);
  //     phantom.order = order;
  //     phantom.start_time = now;
  //     phantom.drone_id   = bspline_optimizer_->getPhantomFakeDroneId(); 
  //     phantom.traj_id    = phantom.drone_id; 

  //     phantom.pos_pts.clear();
  //     for (int i = 0; i < opt_points.cols(); ++i) {
  //       geometry_msgs::Point pt;
  //       pt.x = opt_points(0, i); pt.y = opt_points(1, i); pt.z = opt_points(2, i);
  //       phantom.pos_pts.push_back(pt);
  //     }

  //     phantom.knots.clear();
  //     Eigen::VectorXd knots = phantom_bspline.getKnot();
  //     for (int k = 0; k < knots.size(); ++k) phantom.knots.push_back(knots(k));
  //     return true;
  //   }
  //   return false;
  // }


  bool EGOPlannerManager::planAdversarialPhantomTraj(traj_utils::Bspline &phantom, double horizon, double dt, Eigen::Vector3d start_pos)
  {
    if (!bspline_optimizer_ || swarm_trajs_buf_.empty()) return false;

    const int order = 3;
    if (dt <= 1e-4) dt = 0.1;
    int N_sample = static_cast<int>(std::round(horizon / dt));
    N_sample = std::max(N_sample, 10);

    Eigen::MatrixXd init_points(3, N_sample);
    ros::Time now = ros::Time::now();
    double swarm_clearance = bspline_optimizer_->getSwarmClearance();
    const double CLEARANCE = swarm_clearance * 2.0;

    for (int k = 0; k < N_sample; ++k)
    {
      double t_global = now.toSec() + k * dt;
      bool found_any = false;
      double best_score = -1.0;
      Eigen::Vector3d p_best = start_pos; 

      for (size_t i = 0; i < swarm_trajs_buf_.size(); ++i) {
        auto &traj_i = swarm_trajs_buf_[i];
        if (traj_i.drone_id < 0 || traj_i.drone_id == pp_.drone_id) continue;
        double t_local = std::max(0.0, std::min(t_global - traj_i.start_time_.toSec(), traj_i.duration_));
        Eigen::Vector3d p_cand = traj_i.position_traj_.evaluateDeBoorT(t_local);

        double score = 0.0;
        for (size_t j = 0; j < swarm_trajs_buf_.size(); ++j) {
           if (swarm_trajs_buf_[j].drone_id < 0) continue;
           double t_j = std::max(0.0, std::min(t_global - swarm_trajs_buf_[j].start_time_.toSec(), swarm_trajs_buf_[j].duration_));
           double d = (p_cand - swarm_trajs_buf_[j].position_traj_.evaluateDeBoorT(t_j)).norm();
           if (d < CLEARANCE) score += pow(CLEARANCE - d, 2);
        }
        if (score > best_score) { best_score = score; p_best = p_cand; found_any = true; }
      }

      // 核心修正：从幽灵当前的投影起点 (start_pos) 向 探测到的最拥挤点 (p_best) 进行 8 点插值
      if (k < 8) {
        double alpha = static_cast<double>(k) / 8.0;
        init_points.col(k) = (1.0 - alpha) * start_pos + alpha * p_best;
      } else {
        init_points.col(k) = found_any ? p_best : init_points.col(k-1);
      }
    }

    Eigen::MatrixXd opt_points;
    if (bspline_optimizer_->BsplineOptimizeTrajAdversarial(init_points, dt, opt_points)) {
      UniformBspline b(opt_points, order, dt);
      phantom.order = order;
      phantom.start_time = now;
      phantom.drone_id = bspline_optimizer_->getPhantomFakeDroneId();
      phantom.traj_id = phantom.drone_id;
      phantom.pos_pts.clear();
      for (int i = 0; i < opt_points.cols(); ++i) {
        geometry_msgs::Point pt; pt.x = opt_points(0, i); pt.y = opt_points(1, i); pt.z = opt_points(2, i);
        phantom.pos_pts.push_back(pt);
      }
      phantom.knots.clear();
      Eigen::VectorXd knots = b.getKnot();
      for (int k = 0; k < knots.size(); ++k) phantom.knots.push_back(knots(k));
      return true;
    }
    return false;
  }
  bool EGOPlannerManager::planTargetAdversarialPhantomTraj(traj_utils::Bspline &phantom, double horizon, double dt, int phantom_target_drone_id, Eigen::Vector3d start_pos)  // mal_type = 4；针对单个无人机target_drone_id
  {  //x效果好
    if (swarm_trajs_buf_.empty() || phantom_target_drone_id < 0)
    {
      ROS_WARN("[TargetAdv] Swarm buffer empty or target ID invalid (%d).", phantom_target_drone_id); // 针对单个无人机target_drone_id
      return false;
    }

    const int order = 3;
    if (dt <= 1e-4) { dt = 0.1; }

    int N_sample = static_cast<int>(std::round(horizon / dt));
    N_sample = std::max(N_sample, 10);

    Eigen::MatrixXd init_points(3, N_sample);
    ros::Time now = ros::Time::now();

    double swarm_clearance = bspline_optimizer_->getSwarmClearance();
    const double CLEARANCE = swarm_clearance;

    // --- Check if target trajectory is available ---
    if (phantom_target_drone_id >= (int)swarm_trajs_buf_.size() || swarm_trajs_buf_[phantom_target_drone_id].drone_id < 0)
    {
        ROS_ERROR("[TargetAdv] Target drone %d trajectory not in buffer.", phantom_target_drone_id); // 针对单个无人机target_drone_id
        return false;
    }
    const auto& target_traj_data = swarm_trajs_buf_[phantom_target_drone_id];
    // ----------------------------------------------

    Eigen::Vector3d local_target_pt = target_traj_data.start_pos_; // Use the target's start position as the local target point
    local_target_pt = local_data_.position_traj_.evaluateDeBoorT(0.0);
    for (int k = 0; k < N_sample; ++k)
    {
      if (k == 0)
      {
        init_points.col(k) = start_pos;
        continue;
      }
      double tau = k * dt;
      double t_global = now.toSec() + tau;

      bool found_any = false;
      double best_score = -1.0;
      Eigen::Vector3d best_p(0.0, 0.0, 0.0);
      
      // We only consider the target drone's current position as the initial candidate for maximum crowding
      // Get target position (p_target)
      double t_local_target = t_global - target_traj_data.start_time_.toSec();
      if (t_local_target < 0.0) t_local_target = 0.0;
      if (t_local_target > target_traj_data.duration_) t_local_target = target_traj_data.duration_;
      
      Eigen::Vector3d p_target;
      try {
          p_target = target_traj_data.position_traj_.evaluateDeBoorT(t_local_target);
      } catch (...) {
          // If evaluation fails, fall back to start pos
          p_target = target_traj_data.start_pos_;
      }
      if (k < 5) {                          //确保从local_target_pt开始，避免初始点过近
        double alpha = static_cast<double>(k) / 5.0;
        init_points.col(k) = alpha * p_target + (1.0 - alpha) * local_target_pt;
      }
      else {
        init_points.col(k) = p_target;
      }
      // Candidate point p_i is set to the target's position for maximum immediate collision risk
      Eigen::Vector3d p_i = p_target;

      // Score calculation: Score = sum_j (max(0, CLEARANCE - ||p_target - p_j||))^2
      double score = 0.0;
      for (size_t j = 0; j < swarm_trajs_buf_.size(); ++j)
      {
          const auto &traj_j = swarm_trajs_buf_[j];
          if (traj_j.drone_id < 0) continue;
          
          double t_local_j = t_global - traj_j.start_time_.toSec();
          if (t_local_j < 0.0) t_local_j = 0.0;
          if (t_local_j > traj_j.duration_) t_local_j = traj_j.duration_;

          Eigen::Vector3d p_j;
          try {
            p_j = traj_j.position_traj_.evaluateDeBoorT(t_local_j);
          } catch (...) { continue; }

          // The score is calculated based on how crowded p_target is
          double d = (p_target - p_j).norm(); 
          double diff = CLEARANCE - d;
          if (diff > 0.0)
          {
            score += diff * diff;
          }
      }
      
      // For target-specific adversarial, the best score is the score at the target's position
      best_score = score;
      best_p = p_target;
      found_any = true; // Always true since p_target is always selected

      if (!found_any) // Fallback logic (should not be reached)
      {
        best_p = local_data_.start_pos_;
      }

      init_points.col(k) = best_p;
    }
    
    // --- STEP 2: Adversarial Optimization ---
    Eigen::MatrixXd opt_points;

    // Optimization uses the normal adversarial cost (mal_type 3 logic is now used)
    if (init_points.cols() < 10)
    {
      opt_points = init_points;
    }
    else
    {
      bool ok = bspline_optimizer_->BsplineOptimizeTrajAdversarial(init_points, dt, opt_points);
      if (!ok)
      {
        opt_points = init_points;
      }
    }

    // --- STEP 3: Fill ROS Message ---
    UniformBspline phantom_bspline(opt_points, order, dt);
    
    phantom.order = order;
    phantom.start_time = ros::Time::now();
    phantom.drone_id   = bspline_optimizer_->getPhantomFakeDroneId(); // Use configured fake ID
    phantom.traj_id    = phantom.drone_id; 

    // Fill pos_pts
    phantom.pos_pts.clear();
    for (int i = 0; i < opt_points.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = opt_points(0, i);
      pt.y = opt_points(1, i);
      pt.z = opt_points(2, i);
      phantom.pos_pts.push_back(pt);
    }

    // Fill knots
    phantom.knots.clear();
    Eigen::VectorXd knots = phantom_bspline.getKnot();
    for (int k = 0; k < knots.size(); ++k)
    {
      phantom.knots.push_back(knots(k));
    }

    return true;
  }
  // bool EGOPlannerManager::planTargetAdversarialPhantomTraj(
  //     traj_utils::Bspline &phantom,
  //     double horizon,
  //     double dt,
  //     int phantom_target_drone_id,
  //     Eigen::Vector3d start_pos)
  // {需要修改的版本
  //   if (!bspline_optimizer_ || swarm_trajs_buf_.empty() || phantom_target_drone_id < 0)
  //     return false;

  //   if (phantom_target_drone_id >= (int)swarm_trajs_buf_.size() ||
  //       swarm_trajs_buf_[phantom_target_drone_id].drone_id < 0)
  //   {
  //     ROS_ERROR("[TargetAdv] Target drone invalid.");
  //     return false;
  //   }

  //   const int order = 3;
  //   if (dt <= 1e-4) dt = 0.1;

  //   int N_sample = std::max(10, (int)std::round(horizon / dt));

  //   Eigen::MatrixXd init_points(3, N_sample);
  //   ros::Time now = ros::Time::now();

  //   double CLEARANCE = bspline_optimizer_->getSwarmClearance();

  //   const auto &target_traj = swarm_trajs_buf_[phantom_target_drone_id];

  //   // ======== 核心1：连续起点 ========
  //   Eigen::Vector3d seg_start = start_pos;
  //   if (has_last_target_phantom_end_)
  //     seg_start = last_target_phantom_end_;

  //   for (int k = 0; k < N_sample; ++k)
  //   {
  //     double t_global = now.toSec() + k * dt;

  //     // ======== 只取 target 的位置 ========
  //     double t_local = std::max(
  //         0.0,
  //         std::min(t_global - target_traj.start_time_.toSec(),
  //                 target_traj.duration_));

  //     Eigen::Vector3d p_target;
  //     try {
  //       p_target = target_traj.position_traj_.evaluateDeBoorT(t_local);
  //     } catch (...) {
  //       p_target = target_traj.start_pos_;
  //     }

  //     // ======== 拥挤度（和群体版完全一样） ========
  //     double score = 0.0;
  //     for (size_t j = 0; j < swarm_trajs_buf_.size(); ++j)
  //     {
  //       const auto &traj_j = swarm_trajs_buf_[j];
  //       if (traj_j.drone_id < 0) continue;

  //       double t_j = std::max(
  //           0.0,
  //           std::min(t_global - traj_j.start_time_.toSec(),
  //                   traj_j.duration_));

  //       try {
  //         Eigen::Vector3d p_j = traj_j.position_traj_.evaluateDeBoorT(t_j);
  //         double d = (p_target - p_j).norm();
  //         if (d < CLEARANCE)
  //           score += pow(CLEARANCE - d, 2);
  //       } catch (...) {}
  //     }

  //     Eigen::Vector3d best_p = p_target; // 单目标版本

  //     // ======== 核心2：完全复用群体版控制方式 ========
  //     if (k == 0)
  //     {
  //       init_points.col(k) = seg_start;
  //     }
  //     else if (k < 8)
  //     {
  //       double alpha = (double)k / 8.0;
  //       init_points.col(k) = (1.0 - alpha) * seg_start + alpha * best_p;
  //     }
  //     else
  //     {
  //       init_points.col(k) = best_p;
  //     }
  //   }

  //   // ======== 对抗优化（保持一致） ========
  //   Eigen::MatrixXd opt_points;
  //   bool ok = bspline_optimizer_->BsplineOptimizeTrajAdversarial(init_points, dt, opt_points);
  //   if (!ok) opt_points = init_points;

  //   // ======== 强制首点一致（关键！！） ========
  //   opt_points.col(0) = seg_start;

  //   // ======== 输出 ========
  //   UniformBspline b(opt_points, order, dt);

  //   phantom.order = order;
  //   phantom.start_time = now;
  //   phantom.drone_id = bspline_optimizer_->getPhantomFakeDroneId();
  //   phantom.traj_id = phantom.drone_id;

  //   phantom.pos_pts.clear();
  //   for (int i = 0; i < opt_points.cols(); ++i)
  //   {
  //     geometry_msgs::Point pt;
  //     pt.x = opt_points(0, i);
  //     pt.y = opt_points(1, i);
  //     pt.z = opt_points(2, i);
  //     phantom.pos_pts.push_back(pt);
  //   }

  //   phantom.knots.clear();
  //   Eigen::VectorXd knots = b.getKnot();
  //   for (int i = 0; i < knots.size(); ++i)
  //     phantom.knots.push_back(knots(i));

  //   // ======== 核心3：记录终点 ========
  //   last_target_phantom_end_ = opt_points.col(opt_points.cols() - 1);
  //   has_last_target_phantom_end_ = true;

  //   return true;
  // }
  // bool EGOPlannerManager::planAdversarialPhantomTraj(traj_utils::Bspline &phantom,
  //                                                  double horizon,
  //                                                  double dt,
  //                                                  Eigen::Vector3d start_pos)
  // {
  //   if (!bspline_optimizer_)
  //   {
  //     ROS_ERROR("[Adversarial] bspline_optimizer_ is null.");
  //     return false;
  //   }
  //   if (swarm_trajs_buf_.empty())
  //   {
  //     ROS_WARN("[Adversarial] swarm_trajs_buf_ is empty, cannot plan phantom.");
  //     return false;
  //   }

  //   const int order = 3;
  //   //防御性处理，dt需为正数
  //   if (dt <= 1e-4)
  //   {
  //     ROS_WARN("[Adversarial] dt is too small, using default value.");
  //     dt = 0.1; // 默认时间间隔
  //   }
  //   if (horizon <= 1e-4)
  //   {
  //     ROS_WARN("[Adversarial] horizon is too small, using default value.");
  //     horizon = 5.0; // 默认时间间隔
  //   }
  //   //计算控制点数量，防止优化器出错
  //   int N_sample = static_cast<int>(std::round(horizon / dt)); // 控制点数量
  //   N_sample = std::max(N_sample, 10); // 最少10个点

  //   Eigen::MatrixXd init_points(3, N_sample);
  //   ros::Time now = ros::Time::now();

  //   // 从参数中读 swarm_clearance，用来定义“什么叫拥挤”
  //   double swarm_clearance = bspline_optimizer_->getSwarmClearance();
  //   const double CLEARANCE = swarm_clearance * 2.0; // 设定“拥挤”阈值为 swarm_clearance 的两倍

  //   const double ATTACK_RATID = 0.6;
  //   int attack_sample_end = static_cast<int>(std::floor(ATTACK_RATID * N_sample));
  //   attack_sample_end = std::max(attack_sample_end, 1);

  //   //椭球参数
  //   const double a = 2.0;
  //   const double b = 1.0;
  //   const double inv_a2 = 1.0 / (a * a);
  //   const double inv_b2 = 1.0 / (b * b);

  //   //时间连续性约束
  //   const double TIME_SMOOTH_WEIGHT = 0.1;

  //   // 记录上一时间片的best_points，用于时间连续性约束
  //   Eigen::Vector3d last_best_p = local_data_.start_pos_;
  //   bool has_last_best = false;

  //   for (int k = 0; k < N_sample; ++k)
  //   {
  //     if (k == 0)
  //     {
  //       init_points.col(k) = start_pos;
  //       continue;
  //     }
  //     double tau = k * dt;
  //     double t_global = now.toSec() + tau;

  //     // ------------------- 前半段：强攻击（密集搜索） -------------------
  //     if (k < attack_sample_end)
  //     {
  //       bool   found_any  = false;
  //       double best_score = -1.0;
  //       Eigen::Vector3d best_p(0.0, 0.0, 0.0);

  //       // 预先估计当前时刻的 swarm center，用于构造偏移候选
  //       Eigen::Vector3d swarm_center(0.0, 0.0, 0.0);
  //       int valid_center_cnt = 0;
  //       for (size_t j = 0; j < swarm_trajs_buf_.size(); ++j)
  //       {
  //         const auto &traj_j = swarm_trajs_buf_[j];
  //         if (traj_j.drone_id < 0)
  //           continue;

  //         double t_local_j = t_global - traj_j.start_time_.toSec();
  //         if (t_local_j < 0.0)
  //           t_local_j = 0.0;
  //         if (t_local_j > traj_j.duration_)
  //           t_local_j = traj_j.duration_;

  //         Eigen::Vector3d p_j;
  //         try
  //         {
  //           p_j = traj_j.position_traj_.evaluateDeBoorT(t_local_j);
  //         }
  //         catch (...)
  //         {
  //           continue;
  //         }

  //         swarm_center += p_j;
  //         valid_center_cnt++;
  //       }
  //       if (valid_center_cnt > 0)
  //         swarm_center /= static_cast<double>(valid_center_cnt);

  //       // 外层：枚举每个无人机 i，把它在该时刻的位置当作候选中心
  //       for (size_t i = 0; i < swarm_trajs_buf_.size(); ++i)
  //       {
  //         const auto &traj_i = swarm_trajs_buf_[i];
  //         // 跳过自己和无效 id
  //         if (traj_i.drone_id < 0 || traj_i.drone_id == pp_.drone_id)
  //           continue;

  //         double t_local_i = t_global - traj_i.start_time_.toSec();
  //         if (t_local_i < 0.0)
  //           t_local_i = 0.0;
  //         if (t_local_i > traj_i.duration_)
  //           t_local_i = traj_i.duration_;

  //         Eigen::Vector3d p_i;
  //         try
  //         {
  //           p_i = traj_i.position_traj_.evaluateDeBoorT(t_local_i);
  //         }
  //         catch (...)
  //         {
  //           continue;
  //         }

  //         // 构造候选点集合：原始点 p_i + 一个“朝 swarm center 偏移”的点
  //         std::vector<Eigen::Vector3d> candidate_points;
  //         candidate_points.reserve(2);
  //         candidate_points.push_back(p_i);

  //         if (valid_center_cnt > 0)
  //         {
  //           Eigen::Vector3d dir_to_center = swarm_center - p_i;
  //           if (dir_to_center.norm() > 1e-3)
  //           {
  //             dir_to_center.normalize();
  //             double offset_len = 0.5 * CLEARANCE; // 偏移长度，可调
  //             Eigen::Vector3d p_offset = p_i + offset_len * dir_to_center;
  //             candidate_points.push_back(p_offset);
  //           }
  //         }

  //         // 对每个候选点计算“拥挤度 + 时间平滑”的综合得分
  //         for (const auto &cand_p : candidate_points)
  //         {
  //           double score = 0.0;

  //           // 1）密集度评分（与 calcSwarmCost 一致的椭球距离）
  //           for (size_t j = 0; j < swarm_trajs_buf_.size(); ++j)
  //           {
  //             const auto &traj_j = swarm_trajs_buf_[j];
  //             if (traj_j.drone_id < 0)
  //               continue;

  //             double t_local_j = t_global - traj_j.start_time_.toSec();
  //             if (t_local_j < 0.0)
  //               t_local_j = 0.0;
  //             if (t_local_j > traj_j.duration_)
  //               t_local_j = traj_j.duration_;

  //             Eigen::Vector3d p_j;
  //             try
  //             {
  //               p_j = traj_j.position_traj_.evaluateDeBoorT(t_local_j);
  //             }
  //             catch (...)
  //             {
  //               continue;
  //             }

  //             Eigen::Vector3d dist_vec = cand_p - p_j;
  //             double ellip_dist = std::sqrt(
  //                 dist_vec(2) * dist_vec(2) * inv_a2 +
  //                 (dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) * inv_b2);

  //             double dist_err = CLEARANCE - ellip_dist;
  //             if (dist_err > 0.0)
  //             {
  //               score += dist_err * dist_err;
  //             }
  //           }

  //           // 2）时间连续性约束：鼓励 cand_p 靠近上一帧的 best_p
  //           if (has_last_best)
  //           {
  //             double smooth_penalty = (cand_p - last_best_p).squaredNorm();
  //             score -= TIME_SMOOTH_WEIGHT * smooth_penalty;
  //           }

  //           if (score > best_score)
  //           {
  //             best_score = score;
  //             best_p     = cand_p;
  //             found_any  = true;
  //           }
  //         } // end for candidate_points
  //       }   // end for i (swarm_trajs_buf_)

  //       if (!found_any)
  //       {
  //         best_p = local_data_.start_pos_;
  //       }

  //       if (k == 0) // For the first point, ensure it starts near the local target point
  //       {
  //         best_p = local_data_.start_pos_;
  //       }
  //       init_points.col(k) = best_p;
  //       last_best_p        = best_p;
  //       has_last_best      = true;
  //     }
  //     // ------------------- 后半段：弱攻击 / 收尾 -------------------
  //     else
  //     {
  //       // 简单策略：保持上一帧位置，或缓慢抬高 z 轴，让 phantom 逐渐远离主要碰撞区域
  //       Eigen::Vector3d p = last_best_p;

  //       // 例如可以轻微抬高高度（可选）
  //       // p.z() += 0.02;

  //       init_points.col(k) = p;
  //       last_best_p        = p;
  //       has_last_best      = true;
  //     }
  //   } // end for k

  //   // 用对抗 cost 做 B-spline 优化
  //   // 用对抗 cost 做 B-spline 优化
  //   Eigen::MatrixXd opt_points;

  //   if (init_points.cols() < 10)
  //   {
  //     ROS_WARN("[Adversarial] init_points is too small (%d), using init_points as phantom.",
  //             (int)init_points.cols());
  //     opt_points = init_points;
  //   }
  //   else
  //   {
  //     bool ok = bspline_optimizer_->BsplineOptimizeTrajAdversarial(init_points, dt, opt_points);
  //     if (!ok)
  //     {
  //       ROS_WARN("[Adversarial] BsplineOptimizeTrajAdversarial failed, use init_points as phantom.");
  //       opt_points = init_points;
  //     }
  //   }

  //   // 填充 phantom Bspline 消息
  //   phantom.order = order;
  //   phantom.pos_pts.clear();
  //   phantom.knots.clear();

  //   for (int i = 0; i < opt_points.cols(); ++i)
  //   {
  //     geometry_msgs::Point pt;
  //     pt.x = opt_points(0, i);
  //     pt.y = opt_points(1, i);
  //     pt.z = opt_points(2, i);
  //     phantom.pos_pts.push_back(pt);
  //   }

  //   int knot_num = opt_points.cols() + order + 1;
  //   phantom.knots.reserve(knot_num);
  //   for (int k = 0; k < knot_num; ++k)
  //   {
  //     phantom.knots.push_back(static_cast<double>(k) * dt);
  //   }

  //   phantom.start_time = ros::Time::now();
  //   phantom.drone_id   = 11; // 虚拟 id，可以改成参数
  //   phantom.traj_id    = 11;

  //   // 记录虚影轨迹规划
  //   ExpStats s_phantom;
  //   s_phantom.drone_id = pp_.drone_id;
  //   s_phantom.plan_seq = plan_seq_++;
  //   s_phantom.stamp = ros::Time::now().toSec();
  //   s_phantom.success = true;
  //   s_phantom.phantom_id = phantom.drone_id;
  //   // 虚影轨迹也使用对抗性优化，记录成本
  //   s_phantom.J_total = bspline_optimizer_->getLastTotalCost();
  //   s_phantom.J_smooth = bspline_optimizer_->getLastSmoothCost();
  //   s_phantom.J_feas = bspline_optimizer_->getLastFeasCost();
  //   s_phantom.J_swarm = bspline_optimizer_->getLastSwarmCost();
  //   s_phantom.J_terminal = bspline_optimizer_->getLastTerminalCost();
  //   s_phantom.min_ellip_dist = bspline_optimizer_->getLastMinEllipDist();
  //   exp_logger_.log(s_phantom);

  //   return true;
  // }


} // namespace ego_planner
