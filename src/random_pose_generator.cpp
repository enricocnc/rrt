#include <rrt/random_pose_generator.h>

#include <rrt/types.h>

#include <random>
#include <stdexcept>
#include <string>

RandomPoseGenerator::RandomPoseGenerator(const double &min_x, const double &min_y, const double &max_x, const double &max_y) {
  setBoundaries(min_x, min_y, max_x, max_y);
  
  std::random_device rd;
  mt_.seed(rd());
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

types::Pose2D RandomPoseGenerator::sample() {
  return types::Pose2D{.position = types::WorldPosition{.x = x_dist_(mt_),
                                                        .y = y_dist_(mt_)},
                       .yaw = yaw_dist_(mt_)};
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

void RandomPoseGenerator::setBoundaries(const double &min_x, const double &min_y, const double &max_x, const double &max_y) {
  if (min_x >= max_x)
    throw std::invalid_argument("Invalid x interval! Given min: " + std::to_string(min_x) + ", given max: " + std::to_string(max_x));
  if (min_y >= max_y)
    throw std::invalid_argument("Invalid y interval! Given min: " + std::to_string(min_y) + ", given max: " + std::to_string(max_y));

  x_dist_.param(decltype(x_dist_)::param_type(min_x, max_x));
  y_dist_.param(decltype(y_dist_)::param_type(min_y, max_y));
  yaw_dist_.param(decltype(yaw_dist_)::param_type(-M_PI, M_PI));
}