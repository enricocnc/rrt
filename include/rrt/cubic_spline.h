#pragma once

#include <rrt/types.h>

#include <vector>

namespace spline {

// return a pair composed by the path from conf1 to conf2 (computed with cubic splines interpolation) and its length
std::pair<std::vector<types::Pose2D>, double> computeSplinePath(const types::Pose2D &conf1,
                                                                const types::Pose2D &conf2,
                                                                const double &tf,
                                                                const double &v);

} // namespace spline