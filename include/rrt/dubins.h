#pragma once

#include <rrt/types.h>

#include <vector>

// compute the shortest dubins path from conf1 to conf2, with turning radius rho
std::pair<std::vector<ConfigurationDiffDrive>, double> computeShortestDubinsPath(const ConfigurationDiffDrive &conf1,
                                                                                 const ConfigurationDiffDrive &conf2,
                                                                                 const double &rho);
