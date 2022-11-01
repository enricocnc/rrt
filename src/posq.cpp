#include <rrt/posq.h>
#include <rrt/utils.h>

#include <cstddef>
#include <memory>
#include <cmath>
#include <string>

using namespace types;

namespace posq {

struct RelativeConf {
  double rho;
  double phi;
  double alpha;
  std::shared_ptr<Pose2D> center;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

RelativeConf computeRelativeConfiguration(const Pose2D &conf, const Pose2D &center) {
  double los_angle = atan2(center.position.y - conf.position.y, center.position.x - conf.position.x);
  return {.rho = computeDistance(conf.position, center.position),
          .phi = utility::shortestAngularDistance(conf.yaw, center.yaw),
          .alpha = utility::shortestAngularDistance(conf.yaw, los_angle),
          .center = std::make_shared<Pose2D>(center)};
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

Pose2D fromRelativeToAbsoluteConf(const RelativeConf &conf) {
  double tmp_angle = conf.center->yaw - conf.phi + conf.alpha;
  return {.position = WorldPosition{.x = conf.center->position.x - conf.rho * cos(tmp_angle),
                                    .y = conf.center->position.y - conf.rho * sin(tmp_angle)},
          .yaw = tmp_angle - conf.alpha};
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

std::pair<std::vector<Pose2D>, double> computePOSQPath(const Pose2D &conf1, const Pose2D &conf2,
                                                       const double &K_rho, const double &K_phi,
																											 const double &K_alpha, const double &K_v) {
  std::vector<RelativeConf> relative_path;
  relative_path.push_back(computeRelativeConfiguration(conf1, conf2));
  
  static constexpr double gamma = 0.10; // tolerance
  static constexpr double dt = 0.15;

  while (relative_path.back().rho > gamma) {
    double tanh_v = tanh(K_v * relative_path.back().rho);
    double rho_dot = -K_rho * cos(relative_path.back().alpha) * tanh_v;
    double tmp = fabs(relative_path.back().rho) > 1e-4 ? tanh_v / relative_path.back().rho : K_v; // substitute standard limit value if needed
    double alpha_dot = K_rho * sin(relative_path.back().alpha) * tmp - K_alpha * relative_path.back().alpha - K_phi * relative_path.back().phi;
    double phi_dot = -K_alpha * relative_path.back().alpha - K_phi * relative_path.back().phi;
    RelativeConf new_point = relative_path.back();
    new_point.rho += rho_dot * dt;
    new_point.alpha += alpha_dot * dt;
    new_point.phi += phi_dot * dt;
    if (new_point.rho > 2.0 * relative_path.front().rho && new_point.rho > 5.0)
      // control might have diverged
      return std::pair<std::vector<Pose2D>, double>({}, std::numeric_limits<double>::infinity());

    relative_path.push_back(new_point);
  }

  if (utility::rad2deg(fabs(relative_path.back().phi)) > 8.0)
    // discard solutions with a final in-place rotation
    return std::pair<std::vector<Pose2D>, double>({}, std::numeric_limits<double>::infinity());

  std::vector<Pose2D> absolute_path(relative_path.size() + 1);
  absolute_path[0] = conf1;
  for (size_t i = 1; i < relative_path.size(); ++i)
    absolute_path[i] = fromRelativeToAbsoluteConf(relative_path[i]);
  absolute_path.back() = conf2;

  double path_length = 0.0;
  for (size_t i = 1; i < absolute_path.size(); ++i)
    path_length += computeDistance(absolute_path[i - 1].position, absolute_path[i].position);
  
  return std::pair<std::vector<Pose2D>, double>(absolute_path, path_length);
}

} // namespace posq