#pragma once

#include <cmath>
#include <cstddef>

// define a point in map coordinates
struct GridPosition {
  size_t x, y;
};

// define a point in world coordinates
struct WorldPosition {
  double x, y;
};

// robot configuration which does not take into account orientation
struct Configuration2D {
  WorldPosition position;
  bool operator==(const Configuration2D &conf) const {
	auto approx_equal = [] (double a, double b) { return std::abs(a - b) <= 1e-6;};
	return approx_equal(position.x, conf.position.x) && approx_equal(position.y, conf.position.y);
  }
};

// configuration for a differential drive robot
struct ConfigurationDiffDrive {
  WorldPosition position;
  double yaw;
  bool operator==(const ConfigurationDiffDrive &conf) const {
	auto approx_equal = [] (double a, double b) { return std::abs(a - b) <= 1e-6;};
	return approx_equal(position.x, conf.position.x) && approx_equal(position.y, conf.position.y) && approx_equal(yaw, conf.yaw);
  }
};