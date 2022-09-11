#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include <rrt/types.h>

#include <optional>

// Needed for polymorphism. Not the most beautiful code structure ever... 
class DummyClass {
public:
  DummyClass() {};
  virtual ~DummyClass() {};
};

// RRT base class
template<typename RobotConfiguration>
class RRT : public DummyClass {
public:
  RRT(ros::NodeHandle* nodehandle, const unsigned int &dim);
  virtual ~RRT() {};

protected:
  nav_msgs::OccupancyGrid map_;

  double computeSquaredDistance_(const RobotConfiguration &pos1, const RobotConfiguration &pos2);
private:
  //ROS stuff
  ros::NodeHandle nh_;
  ros::Publisher	pub_marker_;
  ros::Subscriber	start_sub_, goal_sub_;

  double radius_constant_;
  unsigned int dimensionality_;
  size_t min_iterations_;
  unsigned int ancestors_depth_;
  double steering_distance_;

  struct Node;
  std::vector<Node> nodes_;

  std::optional<RobotConfiguration> start_node_;
  std::optional<RobotConfiguration> goal_node_;

  virtual RobotConfiguration randomRobotConfiguration_() = 0;
  
  size_t findNearestNode_(const RobotConfiguration &pos);
  std::vector<size_t> findNodesWithinRadius_(const RobotConfiguration &pos, const double &radius);

  // add the ancestors to the given vector of nodes 
  void includeAncestors_(std::vector<size_t> &indexes);

  RobotConfiguration steeringFunction_(const RobotConfiguration &q_nearest, const RobotConfiguration &q_rand);

  bool isGoal_(const RobotConfiguration &pos);
  
  virtual std::pair<std::vector<RobotConfiguration>, double> computeMinCostPath_(const RobotConfiguration &parent, const RobotConfiguration &child) = 0;
  
  bool isValidConfiguration_(const RobotConfiguration &configuration);
  bool collisionFree_(const std::vector<RobotConfiguration> &path);

  // return true if the straight line connecting pos1 and pos2 does not cross occupied map cells
  bool rayTrace_(const GridPosition &pos1, const GridPosition &pos2);

  GridPosition fromWorldToGrid_(const WorldPosition &world_position);

  bool isFreeMapCell_(const GridPosition &pos);
  bool isFreeMapCell_(const size_t &x, const size_t &y);

  void initialPoseCallback_(const geometry_msgs::PoseWithCovarianceStamped& msg);
  void goalCallback_(const geometry_msgs::PoseStamped& msg);

  // utility functions for type conversions
  virtual RobotConfiguration fromPoseToRobotConfiguration_(const geometry_msgs::Pose &pose) = 0;
  geometry_msgs::Point fromConfigurationToPoint3D_(const RobotConfiguration &configuration);

  void searchPath_();
  
  void visualizePath_();
  void visualizeTree_();
};

template<typename RobotConfiguration>
struct RRT<RobotConfiguration>::Node {
  RobotConfiguration configuration;
  size_t id;
  size_t parent;
  double cost;
  std::unique_ptr<std::vector<RobotConfiguration>> path_from_parent;
};
