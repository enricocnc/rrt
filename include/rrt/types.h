#pragma once

#include <cstddef>
#include <vector>

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

using Path = std::vector<Pose2D>;

double computeDistance(const WorldPosition &p1, const WorldPosition &p2);

double computeSquaredDistance(const WorldPosition &p1, const WorldPosition &p2);

// steer q_rand towards q_nearest
Pose2D steeringFunction(const Pose2D &q_nearest, const Pose2D &q_rand,
                        double steering_dist, double steering_angular_dist);

} // namespace types