#pragma once

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <rrt/rrt_core.h>
#include <rrt/types.h>

namespace ros_interface {

class RRTRosInterface {
public:
  RRTRosInterface(ros::NodeHandle* nodehandle);

private:
  //ROS stuff
  ros::NodeHandle nh_;
  ros::Publisher	pub_path_, pub_tree_;
  ros::Subscriber	start_sub_, goal_sub_;

  // subscribers callbacks
  void initialPoseCallback_(const geometry_msgs::PoseWithCovarianceStamped& msg);
  void goalCallback_(const geometry_msgs::PoseStamped& msg);

  void parseAndSetParams_();

  void plan_();

  core::RRTCore rrt_core_;

  std::optional<types::Pose2D> start_pose_, goal_pose_;
};

} // namespace ros_interface