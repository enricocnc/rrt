#pragma once

#include <rrt/types.h>

#include <vector>

namespace posq {

// return a pair composed by the path from conf1 to conf2 (computed with the posq extend function, considering the given control gains) and its length
std::pair<std::vector<types::Pose2D>, double> computePOSQPath(const types::Pose2D &conf1, const types::Pose2D &conf2,
                                                              const double &K_rho, const double &K_phi,
                                                              const double &K_alpha, const double &K_v);

} // namespace posq