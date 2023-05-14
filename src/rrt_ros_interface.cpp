#include <rrt/rrt_ros_interface.h>

#include <rrt/dubins.h>
#include <rrt/posq.h>
#include <rrt/types.h>
#include <rrt/utils.h>

#include <cstddef>
#include <stdexcept>
#include <tf2/utils.h>

#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace ros_interface {

types::Pose2D fromPoseMsgToPose(const geometry_msgs::Pose &pose) {
  return types::Pose2D{.position = types::WorldPosition{.x = pose.position.x,
                                                        .y = pose.position.y},
                       .yaw = tf2::getYaw(pose.orientation)};
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

geometry_msgs::Point fromPoseToPointMsg(const types::Pose2D &pose) {
  geometry_msgs::Point p;
  p.x = pose.position.x;
  p.y = pose.position.y;
  return p;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

RRTRosInterface::RRTRosInterface(ros::NodeHandle* nodehandle) : nh_(*nodehandle) {
  std::string map_topic, start_topic, goal_topic;
  nh_.param("map_topic", map_topic, std::string("/map"));
  nh_.param("start_topic", start_topic, std::string("/initialpose"));
  nh_.param("goal_topic", goal_topic, std::string("/goal"));

  start_sub_ = nh_.subscribe(start_topic, 1, &RRTRosInterface::initialPoseCallback_, this); 
  goal_sub_ = nh_.subscribe(goal_topic, 1, &RRTRosInterface::goalCallback_, this); 
  
  pub_path_ = nh_.advertise<visualization_msgs::Marker>("/path_marker", 1, true);
  pub_tree_ = nh_.advertise<visualization_msgs::MarkerArray>("/tree_marker", 1, true);

  parseAndSetParams_();

  // Wait for map
  while(ros::ok()){
    boost::shared_ptr<nav_msgs::OccupancyGrid const> ros_map = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(map_topic, ros::Duration(4.0));
    if(ros_map){
      ROS_INFO("Got map!");
      std::vector<grid::Map::Cell> occ_data(ros_map->data.size(), grid::Map::Cell::FREE);
      for (size_t i = 0; i < occ_data.size(); ++i)
        if (ros_map->data[i] > 50 || ros_map->data[i] < 0)
          occ_data[i] = grid::Map::Cell::OCCUPIED;
      rrt_core_.setMap(grid::Map{.occupancy = occ_data,
                           .resolution = ros_map->info.resolution,
                           .width = ros_map->info.width,
                           .height = ros_map->info.height,
                           .origin = types::WorldPosition{.x = ros_map->info.origin.position.x,
                                                          .y = ros_map->info.origin.position.y}});
      break;
    } else {
      ROS_INFO("Waiting for map...");
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

void RRTRosInterface::parseAndSetParams_() {
  bool setting_ok = true;
  core::SearchParams search_params;
  // ROS can't take unsigned int params (nor long int)
  int int_param;
  if (nh_.getParam("min_iterations", int_param)) {
    if (int_param < 0)
      ROS_WARN("Min number of iterations can't be negative (received value: %d)! Ignoring given value...", int_param);
    else
      search_params.min_iterations = static_cast<decltype(search_params.min_iterations)>(int_param);
  }
  
  if (nh_.getParam("max_iterations", int_param)) {
    if (int_param < 0)
      ROS_WARN("Max number of iterations can't be negative (received value: %d)! Ignoring given value...", int_param);
    else
      search_params.max_iterations = static_cast<decltype(search_params.max_iterations)>(int_param);
  }

  std::string extend_function_type;
  nh_.param("extend_function_type", extend_function_type, std::string("dubins"));
  double double_param;
  if (extend_function_type == "dubins" && rrt_core_.setExtendFunction(core::RRTCore::EXTEND_FUNCTION::DUBINS)) {
    dubins::DubinsParams dubins_param;
    if (nh_.getParam(extend_function_type + "/turning_radius", double_param)) {
      if (double_param <= 0.0)
        ROS_WARN("Dubins turning radius must be positive (received value: %f)! Ignoring given value...", double_param);
      else
        dubins_param.turning_radius = double_param;
    }
    setting_ok = rrt_core_.setDubinsParams(dubins_param) && setting_ok;
  } else if (extend_function_type == "posq" && rrt_core_.setExtendFunction(core::RRTCore::EXTEND_FUNCTION::POSQ)) {
    posq::POSQParams posq_params;
    if (nh_.getParam(extend_function_type + "/K_rho", double_param)) {
      if (double_param <= 0.0)
        ROS_WARN("POSQ K_rho radius must be positive (received value: %f)! Ignoring given value...", double_param);
      else
        posq_params.K_rho = double_param;
    }
    if (nh_.getParam(extend_function_type + "/K_phi", double_param)) {
      if (double_param >= 0.0)
        ROS_WARN("POSQ K_phi radius must be positive (received value: %f)! Ignoring given value...", double_param);
      else
        posq_params.K_phi = double_param;
    }
    if (nh_.getParam(extend_function_type + "/K_alpha", double_param)) {
      if (double_param <= 0.0)
        ROS_WARN("POSQ K_alpha radius must be positive (received value: %f)! Ignoring given value...", double_param);
      else
        posq_params.K_alpha = double_param;
    }
    if (nh_.getParam(extend_function_type + "/K_v", double_param)) {
      if (double_param <= 0.0)
        ROS_WARN("POSQ K_v radius must be positive (received value: %f)! Ignoring given value...", double_param);
      else
        posq_params.K_v = double_param;
    }
    setting_ok = rrt_core_.setPOSQParams(posq_params) && setting_ok;
  } else if (extend_function_type == "spline" && rrt_core_.setExtendFunction(core::RRTCore::EXTEND_FUNCTION::SPLINE)) {
    spline::SplineParams spline_params;
    if (nh_.getParam(extend_function_type + "/tf", double_param)) {
      if (double_param <= 0.0)
        ROS_WARN("Spline tf radius must be positive (received value: %f)! Ignoring given value...", double_param);
      else
        spline_params.tf = double_param;
    }
    if (nh_.getParam(extend_function_type + "/v", double_param)) {
      if (double_param <= 0.0)
        ROS_WARN("Spline v radius must be positive (received value: %f)! Ignoring given value...", double_param);
      else
        spline_params.v = double_param;
    }
    setting_ok = rrt_core_.setSplineParams(spline_params) && setting_ok;
  } else {
    throw std::runtime_error("Something went wrong while parsing the extend function type!");
  }

  if (nh_.getParam(extend_function_type + "/steering_distance", double_param)) {
    if (double_param <= 0.0)
      ROS_WARN("Steering distance must be positive (received value: %f)! Ignoring given value...", double_param);
    else
      search_params.steering_distance = double_param;
  }

  if (nh_.getParam(extend_function_type + "/steering_angular_distance", double_param)) {
    if (double_param <= 0.0)
      ROS_WARN("Steering angular distance must be positive (received value: %f)! Ignoring given value...", double_param);
    else
      search_params.steering_angular_distance = utility::deg2rad(double_param);
  }

  if (nh_.getParam(extend_function_type + "/radius_constant", double_param)) {
    if (double_param <= 0.0)
      ROS_WARN("Radius constant must be positive (received value: %f)! Ignoring given value...", double_param);
    else
      search_params.radius_constant = double_param;
  }

  if (nh_.getParam(extend_function_type + "/ancestors_depth", int_param)) {
    if (int_param < 0)
      ROS_WARN("Ancestors depth can't be negative (received value: %d)! Ignoring given value...", int_param);
    else
      search_params.ancestors_depth = static_cast<decltype(search_params.ancestors_depth)>(int_param);
  }

  setting_ok = rrt_core_.setSearchParams(search_params) && setting_ok;
  if (!setting_ok)
    throw std::runtime_error("Something went wrong while setting params!");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

void RRTRosInterface::initialPoseCallback_(const geometry_msgs::PoseWithCovarianceStamped& msg) {
  start_pose_ = std::make_optional(fromPoseMsgToPose(msg.pose.pose));
  if (start_pose_.has_value() && goal_pose_.has_value())
    plan_();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

void RRTRosInterface::goalCallback_(const geometry_msgs::PoseStamped& msg) {
  goal_pose_ = std::make_optional(fromPoseMsgToPose(msg.pose));
  if (start_pose_.has_value() && goal_pose_.has_value())
    plan_();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

void RRTRosInterface::plan_() {
  auto search_output = rrt_core_.searchPath(core::RRTInput{.start = start_pose_.value(),
                                                           .goal = goal_pose_.value()});
  ROS_INFO("#################################################\nSearch took: %f s\nTree size is: %lu\nTotal iterations: %lu",
            search_output.search_time, search_output.nodes.size(), search_output.iterations);
  
  visualization_msgs::Marker path_marker;
  path_marker.header.frame_id = "map";
  path_marker.ns = "path";
  path_marker.id = 0;
  if (search_output.path.empty()) {
    ROS_INFO("PATH NOT FOUND!");
    // delete path from previous search path
    path_marker.action = visualization_msgs::Marker::DELETE;
  } else {
    ROS_INFO("Total path cost is: %f", search_output.path_cost);
    // visualize path
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.pose.orientation.w = 1;
    path_marker.scale.x = 0.1;
    path_marker.color.r = 1.0f;
    path_marker.color.g = 0.0f;
    path_marker.color.b = 0.0f;
    path_marker.color.a = 1.0f;
    path_marker.points.resize(search_output.path.size());
    for (size_t i = 0; i < search_output.path.size(); ++i)
      path_marker.points[i] = fromPoseToPointMsg(search_output.path[i]);
  }
  path_marker.header.stamp = ros::Time::now();
  pub_path_.publish(path_marker);

  // visualize tree
  visualization_msgs::MarkerArray tree_markers;
  visualization_msgs::Marker tmp_marker;
  tmp_marker.header.frame_id = "map";
  tmp_marker.header.stamp = ros::Time::now();
	tmp_marker.ns = "tree";
	tmp_marker.action = visualization_msgs::Marker::ADD;
	tmp_marker.id = 0;
	tmp_marker.type = visualization_msgs::Marker::LINE_STRIP;
	tmp_marker.pose.orientation.w = 1;
	tmp_marker.scale.x = 0.03;
	tmp_marker.color.r = 0.0f;
	tmp_marker.color.g = 0.8f;
	tmp_marker.color.b = 1.0f;
	tmp_marker.color.a = 1.0f;
  tree_markers.markers.resize(search_output.nodes.size(), tmp_marker);
  for (size_t i = 0; i < search_output.nodes.size(); ++i) {
    tree_markers.markers[i].id = static_cast<int>(i);
    tree_markers.markers[i].points.resize(search_output.nodes[i].path_from_parent.size());
    for (size_t j = 0; j < search_output.nodes[i].path_from_parent.size(); ++j)
      tree_markers.markers[i].points[j] = fromPoseToPointMsg(search_output.nodes[i].path_from_parent[j]);
  }

  pub_tree_.publish(tree_markers);
}

} // namespace ros_interface