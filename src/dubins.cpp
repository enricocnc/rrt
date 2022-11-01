#include <rrt/dubins.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>

using namespace types;

namespace dubins {

enum class TrajectoryType{
  S,
  R,
  L
};
typedef std::array<TrajectoryType, 3> PathType;

struct DubinsPathDescriptor {
  std::array<double, 3> params;
  double cost;
  PathType type;
  Pose2D start;
  Pose2D goal;
  double radius;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

WorldPosition computeRightCircleCenter(const Pose2D &conf, const double &rho) {
  double th = conf.yaw - M_PI_2;
  return WorldPosition{.x = conf.position.x + rho * cos(th),
                       .y = conf.position.y + rho * sin(th)};
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

WorldPosition computeLeftCircleCenter(const Pose2D &conf, const double &rho) {
  double th = conf.yaw + M_PI_2;
  return WorldPosition{.x = conf.position.x + rho * cos(th),
                       .y = conf.position.y + rho * sin(th)};
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

WorldPosition computeIntermediateCircleCenter(const WorldPosition &C1, const WorldPosition &C2,
                                              const double radius, const PathType &type) {
  double l = computeDistance(C1, C2);
  if (l > 4 * radius)
    return WorldPosition{.x = std::numeric_limits<double>::quiet_NaN(),
                         .y = std::numeric_limits<double>::quiet_NaN()};
  
  double th;
  if (type.front() == TrajectoryType::R && type.at(1) == TrajectoryType::L &&
      type.back() == TrajectoryType::R)
    th = atan2(C2.y - C1.y, C2.x - C1.x) - acos(l / (4.0 * radius));
  else if (type.front() == TrajectoryType::L && type.at(1) == TrajectoryType::R &&
           type.back() == TrajectoryType::L)
    th = atan2(C2.y - C1.y, C2.x - C1.x) + acos(l / (4.0 * radius));
  else
    throw std::invalid_argument("Invalid CCC path!");
  
  return WorldPosition{.x = C1.x + 2 * radius * cos(th), .y = C1.y + 2 * radius * sin(th)};
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

bool isValidDubinsPathType(const PathType type) {
  if (type.front() == TrajectoryType::S || type.back() == TrajectoryType::S)
    return false;
  if (type.front() == type.at(1) || type.at(1) == type.back())
    return false;
  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

DubinsPathDescriptor computeDubinsPathDescriptor(const Pose2D &conf1, const Pose2D &conf2,
                                                 const double &rho, const PathType &type) {
  auto C1 = type.front() == TrajectoryType::R ? computeRightCircleCenter(conf1, rho) : computeLeftCircleCenter(conf1, rho);
  auto C2 = type.back() == TrajectoryType::R ? computeRightCircleCenter(conf2, rho) : computeLeftCircleCenter(conf2, rho);

  bool path_exists = true;

  std::array<double, 3> params;
  double l = computeDistance(C1, C2);
  double final_yaw1; // final yaw on starting circle
  double start_yaw2; // starting yaw on final circle
  if (type.at(1) == TrajectoryType::S && type.front() == type.back()) { // LSL, RSR
    final_yaw1 = atan2(C2.y - C1.y, C2.x - C1.x);
    start_yaw2 = final_yaw1;

    params.at(1) = l;
  } else if (type.at(1) == TrajectoryType::S && type.front() != type.back()) { // LSR, RSL
    if (l > 2 * rho) { // CSC path can be computed
      double d = sqrt(l * l - 4 * rho * rho); // distance between last point on starting circle and first point on final circle
      int sign = type.front() == TrajectoryType::R ? 1 : -1; // RSL -> +1, LSR -> -1

      final_yaw1 = (atan2(0.5 * d, rho) - M_PI_2) * sign + atan2(C2.y - C1.y, C2.x - C1.x); 
      start_yaw2 = final_yaw1;

      params.at(1) = d;
    } else {
      path_exists = false;
    }
  } else { // RLR, LRL
    auto C_tmp = computeIntermediateCircleCenter(C1, C2, rho, type);
    if (!std::isnan(C_tmp.x) && !std::isnan(C_tmp.y)) { // CCC path can be computed
      int sign = type.front() == TrajectoryType::L ? 1 : -1; // LRL -> +1, RLR -> -1

      final_yaw1 = atan2(C_tmp.y - C1.y, C_tmp.x - C1.x) + M_PI_2 * sign;
      start_yaw2 = atan2(C_tmp.y - C2.y, C_tmp.x - C2.x) + M_PI_2 * sign;

      double tmp_final_yaw1 = final_yaw1;
      while (tmp_final_yaw1 * sign < start_yaw2 * sign)
        tmp_final_yaw1 += sign * 2 * M_PI;
    
      params.at(1) = fabs(fmod(start_yaw2 - tmp_final_yaw1, 2.0 * M_PI));
    } else {
      path_exists = false;
    }
  }
  if (!path_exists)
    return DubinsPathDescriptor{.params = {std::numeric_limits<double>::quiet_NaN(),
                                           std::numeric_limits<double>::quiet_NaN(),
                                           std::numeric_limits<double>::quiet_NaN()},
                                .cost = std::numeric_limits<double>::infinity(),
                                .type = type,
                                .start = conf1,
                                .goal = conf2,
                                .radius = rho};
  int sign = type.front() == TrajectoryType::R ? 1 : -1;
  while (sign * final_yaw1 > sign * conf1.yaw)
    final_yaw1 -= sign * 2 * M_PI;
  sign = type.back() == TrajectoryType::R ? 1 : -1;
  while (sign * start_yaw2 < sign * conf2.yaw)
    start_yaw2 += sign * 2 * M_PI;
  params.front() = fabs(fmod(conf1.yaw - final_yaw1, 2.0 * M_PI));
  params.back() = fabs(fmod(conf2.yaw - start_yaw2, 2.0 * M_PI));

  double cost = type.at(1) == TrajectoryType::S ? (params.front() + params.back()) * rho + params.at(1) : std::accumulate(params.cbegin(), params.cend(), 0.0) * rho;

  return DubinsPathDescriptor{.params = params, .cost = cost, .type = type, .start = conf1, .goal = conf2, .radius = rho};
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<Pose2D> computeDubinsPathFromDescriptor(const DubinsPathDescriptor &descriptor) {
  static constexpr double dphi = 3.0 * M_PI / 180.0;
  if (descriptor.cost == std::numeric_limits<double>::infinity())
    return std::vector<Pose2D>{};

  auto C1 = descriptor.type.front() == TrajectoryType::R ? computeRightCircleCenter(descriptor.start, descriptor.radius)
                                                         : computeLeftCircleCenter(descriptor.start, descriptor.radius);
  auto C2 = descriptor.type.back() == TrajectoryType::R ? computeRightCircleCenter(descriptor.goal, descriptor.radius)
                                                         : computeLeftCircleCenter(descriptor.goal, descriptor.radius);
  auto point_on_circle = [] (const double yaw0, const double dyaw, const WorldPosition &C, const double radius, const int sign) {
    double theta = yaw0 - sign * dyaw;
    return Pose2D{.position = WorldPosition{.x = C.x - sign * radius * sin(theta),
                                            .y = C.y + sign * radius * cos(theta)},
                  .yaw = theta};
  };
  std::vector<Pose2D> path;
  if (descriptor.type.at(1) == TrajectoryType::S)
    path.reserve(std::floor(descriptor.params.front() / dphi) + std::floor(descriptor.params.back() / dphi) + 1);
  else
    path.reserve(std::floor(descriptor.params.front() / dphi) + std::floor(descriptor.params.back() / dphi) + std::floor(descriptor.params.at(1) / dphi));

  // add points on first arc
  double dyaw = 0.0;
  int sign = descriptor.type.front() == TrajectoryType::R ? 1 : -1;
  while (dyaw < descriptor.params.front()) {
    path.push_back(point_on_circle(descriptor.start.yaw, dyaw, C1, descriptor.radius, sign));
    dyaw += dphi;
  }
  // manually add last point on first circle
  dyaw = descriptor.params.front();
  path.push_back(point_on_circle(descriptor.start.yaw, dyaw, C1, descriptor.radius, sign));
  
  double start_yaw2;
  if (descriptor.type.at(1) == TrajectoryType::S) {
    // add first point on final circle (for a CSC path) 
    auto conf = Pose2D{.position = WorldPosition{.x = path.back().position.x + descriptor.params.at(1) * cos(path.back().yaw),
                                                 .y = path.back().position.y + descriptor.params.at(1) * sin(path.back().yaw)},
                       .yaw = path.back().yaw};
    path.push_back(conf);
    start_yaw2 = path.back().yaw;
  } else {
    // add points on second arc (for a CCC path)
    auto C_tmp = computeIntermediateCircleCenter(C1, C2, descriptor.radius, descriptor.type);
    sign = descriptor.type.at(1) == TrajectoryType::R ? 1 : -1;
    double final_yaw1 = path.back().yaw;
    dyaw = dphi;
    while (dyaw < descriptor.params.at(1)) {
      path.push_back(point_on_circle(final_yaw1, dyaw, C_tmp, descriptor.radius, sign));
      dyaw += dphi;
    }
    start_yaw2 = final_yaw1 - sign * descriptor.params.at(1);
  }
  // add points on last arc
  dyaw = 0.0;
  sign = descriptor.type.back() == TrajectoryType::R ? 1 : -1;
  while (dyaw < descriptor.params.back()) {
    path.push_back(point_on_circle(start_yaw2, dyaw, C2, descriptor.radius, sign));
    dyaw += dphi;
  }
  // manually add last point
  path.push_back(descriptor.goal);
  return path;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<Pose2D> computeDubinsPath(const Pose2D &conf1, const Pose2D &conf2,
                                      const double &rho, const PathType &type) {
  if (!isValidDubinsPathType(type))
    throw std::invalid_argument("Invalid Dubins path type!");
  return computeDubinsPathFromDescriptor(computeDubinsPathDescriptor(conf1, conf2, rho, type));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

std::pair<std::vector<Pose2D>, double> computeShortestDubinsPath(const Pose2D &conf1,
                                                                 const Pose2D &conf2,
                                                                 const double &rho) {
  static constexpr std::array<PathType, 6> allowed_types= {{{TrajectoryType::R, TrajectoryType::S, TrajectoryType::R},
                                                            {TrajectoryType::L, TrajectoryType::S, TrajectoryType::L},
                                                            {TrajectoryType::R, TrajectoryType::S, TrajectoryType::L},
                                                            {TrajectoryType::L, TrajectoryType::S, TrajectoryType::R},
                                                            {TrajectoryType::R, TrajectoryType::L, TrajectoryType::R},
                                                            {TrajectoryType::L, TrajectoryType::R, TrajectoryType::L}}};
  std::vector<DubinsPathDescriptor> descriptors;
  descriptors.reserve(allowed_types.size());
  for (const auto &type : allowed_types)
    descriptors.push_back(computeDubinsPathDescriptor(conf1, conf2, rho, type));

  auto optimal = std::min_element(descriptors.cbegin(), descriptors.cend(), 
                                  [](const DubinsPathDescriptor &d1, const DubinsPathDescriptor &d2) { return d1.cost < d2.cost;});
  return std::make_pair(computeDubinsPathFromDescriptor(*optimal), optimal->cost);
}

} // namespace dubins