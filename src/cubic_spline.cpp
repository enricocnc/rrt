#include <rrt/cubic_spline.h>

#include <rrt/utils.h>

#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>

using namespace types;

namespace spline {

// class for handling cubic spline interpolation
class CubicSpline
{
public:
  CubicSpline(const double &xi, const double &xf, const double &xi_dot, const double &xf_dot, const double &tf);

  double computeValue(const double &t) const;
  double computeDerivative(const double &t) const;
private:
  double a_, b_, c_, d_;
  double tf_;
};

CubicSpline::CubicSpline(const double &xi, const double &xf, const double &xi_dot, const double &xf_dot, const double &tf) : tf_(tf) {
  if (tf <= 0.0)
    throw std::runtime_error("Final time must be positive!");
  if (xi == xf && xi_dot == xf_dot)
    throw std::runtime_error("Can't interpolate between two equal values!");

  /*
  Spline coefficients are computed solving the following linear system:
  p(0) = xi
  p(tf) = xf
  p_dot(0) = xi_dot
  p_dot(tf) = xf_dot

  where p(t) = a + b * t + c * t^2 + d * t^3
  and p_dot(t) = b + 2 * c * t + 3 * d * t^2
  
  here, the solution of this system (computed throuhg symbolic computation) is directly used
  */
  a_ = xi;
  b_ = xi_dot;
  c_ = (3.0 * (xf - xi) - tf * (xf_dot + 2.0 * xi_dot)) / (tf * tf);
  d_ = (tf * (xf_dot + xi_dot) + 2.0 * (xi - xf)) / (tf * tf * tf);
}

double CubicSpline::computeValue(const double &t) const {
  if (t < 0.0 || t > tf_)
    throw std::runtime_error("Spline is not valid in the given point!");

  return a_ + t * (b_ + t * (c_ + t * d_));
}

double CubicSpline::computeDerivative(const double &t) const {
  if (t < 0.0 || t > tf_)
    throw std::runtime_error("Spline is not valid in the given point!");

  return b_ + t * (2.0 * c_ + t * 3.0 * d_);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

// this class uses cubic splines in order to interpolate the path between the given configurations
class Spline2D {
public:
  Spline2D(const Pose2D &conf1, const Pose2D &conf2, const double &tf, const double &v);

  Pose2D computePose(const double &t) const;
private:
  std::unique_ptr<CubicSpline> x_spline_, y_spline_;

};

Spline2D::Spline2D(const Pose2D &conf1, const Pose2D &conf2, const double &tf, const double &v) {
  // constraints on derivatives terms are computed using the unicycle model
  double xi_dot = v * cos(conf1.yaw);
  double xf_dot = v * cos(conf2.yaw);
  x_spline_ = std::make_unique<CubicSpline>(conf1.position.x, conf2.position.x, xi_dot, xf_dot, tf);
  double yi_dot = v * sin(conf1.yaw);
  double yf_dot = v * sin(conf2.yaw);
  y_spline_ = std::make_unique<CubicSpline>(conf1.position.y, conf2.position.y, yi_dot, yf_dot, tf);
}

Pose2D Spline2D::computePose(const double &t) const {
  return Pose2D{.position = WorldPosition{.x = x_spline_->computeValue(t),
                                          .y = y_spline_->computeValue(t)},
                .yaw = atan2(y_spline_->computeDerivative(t), x_spline_->computeDerivative(t))};
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

std::pair<std::vector<Pose2D>, double> computeSplinePath(const Pose2D &conf1, const Pose2D &conf2,
                                                         const SplineParams &params) {
  if (conf1 == conf2)
    return std::make_pair(Path{conf1, conf2}, 0.0);

  double t = 0.0;
  static constexpr double dt = 0.15;

  Spline2D spline(conf1, conf2, params.tf, params.v);

  std::vector<Pose2D> path;
  path.reserve(static_cast<size_t>(std::floor(params.tf / dt + 2)));
  for (; t < params.tf; t += dt)
    path.push_back(spline.computePose(t));
  path.push_back(spline.computePose(params.tf));

  double path_length = 0.0;
  for (size_t i = 1; i < path.size(); ++i)
    path_length += computeDistance(path[i - 1].position, path[i].position);

  return std::pair<std::vector<Pose2D>, double>(path, path_length);
}

} // namespace spline