#pragma once

#include <rrt/types.h>

#include <random>

class RandomPoseGenerator {
public:
	RandomPoseGenerator(const double &min_x, const double &min_y, const double &max_x, const double &max_y);

	// generate random pose
	types::Pose2D sample();

	// set boundaries for the random poses
	void setBoundaries(const double &min_x, const double &min_y, const double &max_x, const double &max_y);

private:
  std::mt19937 mt_;

	std::uniform_real_distribution<double> x_dist_;
  std::uniform_real_distribution<double> y_dist_;
  std::uniform_real_distribution<double> yaw_dist_;
};