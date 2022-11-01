#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include <rrt/types.h>

#include <functional>
#include <optional>

namespace rrt {

// TODO: rrt search and ros interface should be decoupled
class RRT {
public:
  RRT(ros::NodeHandle* nodehandle);

private:
  nav_msgs::OccupancyGrid map_;

  //ROS stuff
  ros::NodeHandle nh_;
  ros::Publisher	pub_marker_;
  ros::Subscriber	start_sub_, goal_sub_;

  // params
  double radius_constant_;
  unsigned int dimensionality_;
  size_t min_iterations_, max_iterations_;
  unsigned int ancestors_depth_;
  double steering_distance_, steering_angular_distance_;

  struct Node {
    types::Pose2D configuration;
    size_t id;
    size_t parent;
    double cost;
    std::unique_ptr<std::vector<types::Pose2D>> path_from_parent;
  };
  std::vector<Node> nodes_;

  std::optional<types::Pose2D> start_node_;
  std::optional<types::Pose2D> goal_node_;

  // return a random robot pose
  types::Pose2D randomRobotConfiguration_();
  
  size_t findNearestNode_(const types::Pose2D &pos);
  std::vector<size_t> findNodesWithinRadius_(const types::Pose2D &pos, const double &radius);

  // add the ancestors to the given vector of nodes 
  void includeAncestors_(std::vector<size_t> &indexes);

  // steer q_rand towards q_nearest
  types::Pose2D steeringFunction_(const types::Pose2D &q_nearest, const types::Pose2D &q_rand);

  bool isGoal_(const types::Pose2D &pos);
  
  // function used for computing paths (it returns a pair composed by the path from parent to child, and the path's cost)
  std::function<std::pair<std::vector<types::Pose2D>, double>(const types::Pose2D &parent, const types::Pose2D &child)> compute_path_;

  bool isValidConfiguration_(const types::Pose2D &configuration);
  bool collisionFree_(const std::vector<types::Pose2D> &path);

  // return true if the straight line connecting pos1 and pos2 does not cross occupied map cells
  bool rayTrace_(const types::GridPosition &pos1, const types::GridPosition &pos2);

  // convert the given position from world coordinates to grid coordinates
  types::GridPosition fromWorldToGrid_(const types::WorldPosition &world_position);

  bool isFreeMapCell_(const types::GridPosition &pos);
  bool isFreeMapCell_(const size_t &x, const size_t &y);

  void initialPoseCallback_(const geometry_msgs::PoseWithCovarianceStamped& msg);
  void goalCallback_(const geometry_msgs::PoseStamped& msg);

  // read and process the given ROS-params
  void initParams_();

  // utility functions for type conversions
  types::Pose2D fromPoseToRobotConfiguration_(const geometry_msgs::Pose &pose);
  geometry_msgs::Point fromConfigurationToPoint3D_(const types::Pose2D &configuration);

  // search a path from start_node_ to goal_node_
  void searchPath_();
  
  void visualizePath_();
  void visualizeTree_();
};

} // namespace rrt
