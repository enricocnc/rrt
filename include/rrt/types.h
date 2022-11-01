#pragma once

#include <cstddef>

namespace types {

// define a point in map coordinates
struct GridPosition {
  size_t x, y;
};

// define a point in world coordinates
struct WorldPosition {
  double x, y;
};

// configuration for a differential drive robot
struct Pose2D {
  WorldPosition position;
  double yaw;
  bool operator==(const Pose2D &conf) const;
};

double computeDistance(const WorldPosition &p1, const WorldPosition &p2);

double computeSquaredDistance(const WorldPosition &p1, const WorldPosition &p2);

} // namespace types