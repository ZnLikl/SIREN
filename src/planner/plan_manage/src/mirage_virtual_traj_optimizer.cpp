#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <bspline_opt/lbfgs.hpp>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <traj_utils/Bspline.h>

namespace ego_planner
{

namespace
{
struct MirageOptContext
{
  Eigen::MatrixXd original;
  Eigen::MatrixXd target;
  double dt = 0.1;
  double max_vel = 3.0;
  double max_acc = 4.0;
  double target_w = 1.0;
  double smooth_w = 12.0;
  double vel_w = 1.5;
  double acc_w = 1.0;
  int first_free = 3;
};

static double mirageCost(void *data, const double *x, double *grad, const int n)
{
  MirageOptContext *ctx = reinterpret_cast<MirageOptContext *>(data);
  const int cols = ctx->original.cols();
  Eigen::MatrixXd q = ctx->original;
  Eigen::MatrixXd g = Eigen::MatrixXd::Zero(3, cols);

  for (int i = ctx->first_free; i < cols && 3 * (i - ctx->first_free) + 2 < n; ++i)
    q.col(i) = Eigen::Vector3d(x[3 * (i - ctx->first_free)],
                               x[3 * (i - ctx->first_free) + 1],
                               x[3 * (i - ctx->first_free) + 2]);

  double cost = 0.0;
  for (int i = ctx->first_free; i < cols; ++i)
  {
    const Eigen::Vector3d d = q.col(i) - ctx->target.col(i);
    cost += ctx->target_w * d.squaredNorm();
    g.col(i) += 2.0 * ctx->target_w * d;
  }

  for (int i = 1; i + 1 < cols; ++i)
  {
    const Eigen::Vector3d a = q.col(i - 1) - 2.0 * q.col(i) + q.col(i + 1);
    cost += ctx->smooth_w * a.squaredNorm();
    g.col(i - 1) += 2.0 * ctx->smooth_w * a;
    g.col(i) -= 4.0 * ctx->smooth_w * a;
    g.col(i + 1) += 2.0 * ctx->smooth_w * a;
  }

  const double dt = std::max(0.02, ctx->dt);
  for (int i = 1; i < cols; ++i)
  {
    const Eigen::Vector3d v = (q.col(i) - q.col(i - 1)) / dt;
    const double excess = v.norm() - ctx->max_vel;
    if (excess > 0.0)
    {
      const Eigen::Vector3d dir = v.normalized();
      cost += ctx->vel_w * excess * excess;
      g.col(i) += 2.0 * ctx->vel_w * excess * dir / dt;
      g.col(i - 1) -= 2.0 * ctx->vel_w * excess * dir / dt;
    }
  }

  for (int i = 2; i < cols; ++i)
  {
    const Eigen::Vector3d a = (q.col(i) - 2.0 * q.col(i - 1) + q.col(i - 2)) / (dt * dt);
    const double excess = a.norm() - ctx->max_acc;
    if (excess > 0.0)
    {
      const Eigen::Vector3d step = 2.0 * ctx->acc_w * excess * a.normalized() / (dt * dt);
      cost += ctx->acc_w * excess * excess;
      g.col(i) += step;
      g.col(i - 1) -= 2.0 * step;
      g.col(i - 2) += step;
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

static Eigen::MatrixXd toMatrix(const traj_utils::Bspline &msg)
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

static void toMsg(const Eigen::MatrixXd &pts, traj_utils::Bspline &msg)
{
  msg.pos_pts.resize(static_cast<size_t>(pts.cols()));
  for (int i = 0; i < pts.cols(); ++i)
  {
    msg.pos_pts[static_cast<size_t>(i)].x = pts(0, i);
    msg.pos_pts[static_cast<size_t>(i)].y = pts(1, i);
    msg.pos_pts[static_cast<size_t>(i)].z = pts(2, i);
  }
}


static void preserveFakeOffset(
    const Eigen::MatrixXd &original,
    const Eigen::MatrixXd &target,
    Eigen::MatrixXd &optimized,
    int first_free,
    double preserve_ratio)
{
  const int cols = std::min(original.cols(), std::min(target.cols(), optimized.cols()));
  const double ratio = std::max(0.0, std::min(1.0, preserve_ratio));

  for (int i = std::max(0, first_free); i < cols; ++i)
  {
    const Eigen::Vector3d ref_delta = target.col(i) - original.col(i);
    const double ref_norm = ref_delta.norm();
    if (ref_norm < 1e-6) continue;

    const Eigen::Vector3d dir = ref_delta / ref_norm;
    const Eigen::Vector3d opt_delta = optimized.col(i) - original.col(i);
    const double projected = opt_delta.dot(dir);
    const double required = ratio * ref_norm;

    if (projected < required)
      optimized.col(i) += (required - projected) * dir;
  }
}
} // namespace

class MirageVirtualTrajOptimizer
{
public:
  traj_utils::Bspline optimize(const traj_utils::Bspline &real_bspline,
                               const std::vector<Eigen::Vector3d> &policy_offsets,
                               const GridMap::Ptr &grid_map,
                               double dt,
                               double max_vel,
                               double max_acc) const
  {
    traj_utils::Bspline out = real_bspline;
    if (real_bspline.pos_pts.empty()) return out;

    Eigen::MatrixXd original = toMatrix(real_bspline);
    Eigen::MatrixXd target = original;
    for (int i = 0; i < target.cols(); ++i)
    {
      const Eigen::Vector3d off =
          policy_offsets.empty() ? Eigen::Vector3d::Zero()
                                 : policy_offsets[std::min(static_cast<size_t>(i), policy_offsets.size() - 1)];
      target.col(i) += off;
    }

    MirageOptContext ctx;
    ctx.original = original;
    ctx.target = target;
    ctx.dt = dt;
    ctx.max_vel = max_vel;
    ctx.max_acc = max_acc;
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
      lbfgs::lbfgs_parameter_t params;
      lbfgs::lbfgs_load_default_parameters(&params);
      params.mem_size = 12;
      params.max_iterations = 80;
      params.g_epsilon = 0.01;
      lbfgs::lbfgs_optimize(variable_num, q.data(), &final_cost, mirageCost, NULL, NULL, &ctx, &params);

      for (int i = ctx.first_free; i < optimized.cols(); ++i)
        optimized.col(i) = Eigen::Vector3d(q[static_cast<size_t>(3 * (i - ctx.first_free))],
                                           q[static_cast<size_t>(3 * (i - ctx.first_free) + 1)],
                                           q[static_cast<size_t>(3 * (i - ctx.first_free) + 2)]);
    }

    preserveFakeOffset(original, target, optimized, ctx.first_free, 0.7);

    if (grid_map != nullptr)
    {
      for (int i = 0; i < optimized.cols(); ++i)
      {
        if (!grid_map->getInflateOccupancy(optimized.col(i))) continue;
        for (int k = 12; k >= 0; --k)
        {
          const double a = static_cast<double>(k) / 12.0;
          const Eigen::Vector3d repaired = original.col(i) + a * (optimized.col(i) - original.col(i));
          if (!grid_map->getInflateOccupancy(repaired))
          {
            optimized.col(i) = repaired;
            break;
          }
        }
      }
    }

    toMsg(optimized, out);
    return out;
  }
};

} // namespace ego_planner
