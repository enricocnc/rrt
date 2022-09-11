#include <rrt/rrt_diff_drive.h>
#include <rrt/dubins.h>

#include <geometry_msgs/Pose.h>
#include <tf2/utils.h>

#include <cmath>
#include <random>
#include <stdexcept>

RRTDiffDrive::RRTDiffDrive(ros::NodeHandle* nodehandle) : RRT<ConfigurationDiffDrive>(nodehandle, 3) {
  double dubins_radius, dubins_radius_default = 0.5;
  nodehandle->param("dubins_radius", dubins_radius, dubins_radius_default);
  if (dubins_radius <= 0.0) {
    ROS_WARN("Dubins radius must be positive (received value: %f)! Setting it to %f...", dubins_radius, dubins_radius_default);
    dubins_radius = dubins_radius_default;
  }
  
  std::string extend_function_type;
  nodehandle->param("extend_function_type", extend_function_type, std::string("dubins"));

  if (extend_function_type == "dubins")
    extend_function_ = std::bind(computeShortestDubinsPath, std::placeholders::_1, std::placeholders::_2, dubins_radius);
  else
    throw std::invalid_argument("Unsupported extend function type!");
  // TODO: implement POSQ extend function from Palmieri: http://www.spencer.eu/papers/palmieriICAPS14.pdf
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

ConfigurationDiffDrive RRTDiffDrive::fromPoseToRobotConfiguration_(const geometry_msgs::Pose &pose) {
  return ConfigurationDiffDrive{.position = WorldPosition{.x = pose.position.x, .y = pose.position.y},
                                .yaw = tf2::getYaw(pose.orientation)};
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

ConfigurationDiffDrive RRTDiffDrive::randomRobotConfiguration_() {
  static std::random_device rd;
  static std::mt19937 mt(rd());
  static std::uniform_real_distribution<double> x_coordinate(map_.info.origin.position.x, map_.info.origin.position.x + map_.info.resolution * map_.info.width);
  static std::uniform_real_distribution<double> y_coordinate(map_.info.origin.position.y, map_.info.origin.position.y + map_.info.resolution * map_.info.height);
  static std::uniform_real_distribution<double> yaw_coordinate(-M_PI, M_PI);
  return ConfigurationDiffDrive{.position = WorldPosition{.x = x_coordinate(mt), .y = y_coordinate(mt)}, .yaw = yaw_coordinate(mt)};
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

std::pair<std::vector<ConfigurationDiffDrive>, double> RRTDiffDrive::computeMinCostPath_(const ConfigurationDiffDrive &parent,
                                                                                         const ConfigurationDiffDrive &child) {
  return extend_function_(parent, child);
}
