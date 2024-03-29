#pragma once

#include <rrt/types.h>

#include <vector>

namespace dubins {

struct DubinsParams{
  double turning_radius;
};

// return a pair composed by the shortest dubins path from conf1 to conf2 (with turning radius rho) and its length
std::pair<std::vector<types::Pose2D>, double> computeShortestDubinsPath(const types::Pose2D &conf1,
                                                                        const types::Pose2D &conf2,
                                                                        const DubinsParams &params);

} // namespace dubins