#include "rrt/rrt.h"

#include <visualization_msgs/Marker.h>

#include <limits>
#include <memory>
#include <vector>

template<typename RobotConfiguration>
RRT<RobotConfiguration>::RRT(ros::NodeHandle* nodehandle, const unsigned int &dim) : nh_(*nodehandle), dimensionality_(dim) {
  std::string map_topic, start_topic, goal_topic;
  nh_.param("map_topic", map_topic, std::string("/map"));
  nh_.param("start_topic", start_topic, std::string("/initialpose"));
  nh_.param("goal_topic", goal_topic, std::string("/goal"));

  start_sub_ = nh_.subscribe(start_topic, 1, &RRT::initialPoseCallback_, this); 
  goal_sub_ = nh_.subscribe(goal_topic, 1, &RRT::goalCallback_, this); 

  double steering_distance_default = 3.0;
  nh_.param("steering_distance", steering_distance_, steering_distance_default);
  if (steering_distance_ <= 0.0) {
    ROS_WARN("Steering distance must be positive (received value: %f)! Setting it to %f...", steering_distance_, steering_distance_default);
    steering_distance_ = steering_distance_default;
  }
  nh_.param("radius_constant", radius_constant_, 50.0);
  
  // ROS can't take unsigned int params
  int tmp;
  nh_.param("min_iterations", tmp, 0);
  if (tmp < 0) {
    ROS_WARN("Min number of iterations can't be negative (received value: %d)! Setting it to 0...", tmp);
    tmp = 0;
  }
  min_iterations_ = static_cast<size_t>(tmp);
  nh_.param("ancestors_depth", tmp, 0);
  if (tmp < 0) {
    ROS_WARN("Ancestors depth can't be negative (received value: %d)! Setting it to 0...", tmp);
    tmp = 0;
  }
  ancestors_depth_ = static_cast<unsigned int>(tmp);

  pub_marker_ = nh_.advertise<visualization_msgs::Marker>("/path_marker", 1, true);
  
  // Wait for map
	while(ros::ok()){
		boost::shared_ptr<nav_msgs::OccupancyGrid const> map = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(map_topic, ros::Duration(4.0));
		if(map){
			ROS_INFO("Got map!");
      map_ = *map;
			break;
		} else {
			ROS_INFO("Waiting for map...");
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename RobotConfiguration>
bool RRT<RobotConfiguration>::isFreeMapCell_(const size_t &x, const size_t &y) {
  if (map_.data[x + y * map_.info.width] == 0)
    return true;
  else
    return false;
}

template <typename RobotConfiguration>
bool RRT<RobotConfiguration>::isFreeMapCell_(const GridPosition &pos) {
  if (map_.data[pos.x + pos.y * map_.info.width] == 0)
    return true;
  else
    return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename RobotConfiguration>
bool RRT<RobotConfiguration>::isValidConfiguration_(const RobotConfiguration &configuration) {
  return isFreeMapCell_(fromWorldToGrid_(configuration.position));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename RobotConfiguration>
GridPosition RRT<RobotConfiguration>::fromWorldToGrid_(const WorldPosition &world_position) {
  return GridPosition({.x = static_cast<size_t>(std::floor((world_position.x - map_.info.origin.position.x) / map_.info.resolution)),
                       .y = static_cast<size_t>(std::floor((world_position.y - map_.info.origin.position.y) / map_.info.resolution))});
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename RobotConfiguration>
bool RRT<RobotConfiguration>::collisionFree_(const std::vector<RobotConfiguration> &path) {
  if (path.size() <= 1)
    throw std::invalid_argument("Invalid path size!");
  
  for (size_t i = 0; i < path.size() - 1; ++i)
    if (!rayTrace_(fromWorldToGrid_(path[i].position), fromWorldToGrid_(path[i + 1].position)))
      return false;

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename RobotConfiguration>
bool RRT<RobotConfiguration>::rayTrace_(const GridPosition &pos1, const GridPosition &pos2) {

  long int dx = pos1.x > pos2.x ? static_cast<long int>(pos1.x - pos2.x) : static_cast<long int>(pos2.x - pos1.x);
  long int dy = pos1.y > pos2.y ? static_cast<long int>(pos1.y - pos2.y) : static_cast<long int>(pos2.y - pos1.y);
  long int n = 1 + dx + dy;

  long int error = dx - dy;
  dx *= 2;
  dy *= 2;

  size_t x = pos1.x, y = pos1.y;
  std::function<void()> step_x, step_y;
  if (pos2.x > pos1.x)
    step_x = [&x] () {++x;};
  else
    step_x = [&x] () {--x;};

  if (pos2.y > pos1.y)
    step_y = [&y] () {++y;};
  else
    step_y = [&y] () {--y;};

  for (long int i = 0; i < n; ++i) {
    if (!isFreeMapCell_(x, y))
      return false;
    
    if (error > 0) {
      step_x();
      error -= dy;
    } else {
      step_y();
      error += dx;
    }
  }
  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename RobotConfiguration>
void RRT<RobotConfiguration>::initialPoseCallback_(const geometry_msgs::PoseWithCovarianceStamped& msg) {
  RobotConfiguration conf = fromPoseToRobotConfiguration_(msg.pose.pose);
  if (!isValidConfiguration_(conf)) {
    ROS_INFO("Ignoring invalid start!");
    return;
  }
  ROS_INFO("Starting position received!");
  start_node_ = std::make_optional(conf);
  if (start_node_.has_value() && goal_node_.has_value())
    searchPath_();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename RobotConfiguration>
void RRT<RobotConfiguration>::goalCallback_(const geometry_msgs::PoseStamped& msg) {
  RobotConfiguration conf = fromPoseToRobotConfiguration_(msg.pose);
  if (!isValidConfiguration_(conf)) {
    ROS_INFO("Ignoring invalid goal!");
    return;
  }
  ROS_INFO("Goal received!");
  goal_node_ = std::make_optional(conf);
  if (start_node_.has_value() && goal_node_.has_value())
    searchPath_();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename RobotConfiguration>
geometry_msgs::Point RRT<RobotConfiguration>::fromConfigurationToPoint3D_(const RobotConfiguration &configuration) {
  geometry_msgs::Point p;
  p.x = configuration.position.x;
  p.y = configuration.position.y;
  p.z = 0;
  return p;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename RobotConfiguration>
bool RRT<RobotConfiguration>::isGoal_(const RobotConfiguration &pos) {
  if (goal_node_.has_value())
    return pos == goal_node_.value();
  return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename RobotConfiguration>
size_t RRT<RobotConfiguration>::findNearestNode_(const RobotConfiguration &configuration) {
  double min_dist = computeSquaredDistance_(configuration, nodes_[0].configuration);
  size_t min_idx = 0;
  for (size_t i = 1; i < nodes_.size(); ++i) {
    double dist = computeSquaredDistance_(configuration, nodes_[i].configuration);
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }

  return min_idx;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename RobotConfiguration>
std::vector<size_t> RRT<RobotConfiguration>::findNodesWithinRadius_(const RobotConfiguration &pos, const double &radius) {
  std::vector<size_t> nodes_idx;
  double squared_radius = radius * radius;
  for (const auto &node : nodes_)
    if (computeSquaredDistance_(pos, node.configuration) <= squared_radius)
      nodes_idx.push_back(node.id);
  return nodes_idx;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename RobotConfiguration>
void RRT<RobotConfiguration>::visualizePath_() {
  size_t idx = std::numeric_limits<size_t>::max();
  for (size_t i = 0; i < nodes_.size(); ++i) {
    if (isGoal_(nodes_[i].configuration)) {
      idx = i;
      break;
    }
  }
  if (idx == std::numeric_limits<size_t>::max())
    return;
  
  visualization_msgs::Marker path_marker;
  path_marker.header.frame_id = "map";
	path_marker.ns = "path";
	path_marker.action = visualization_msgs::Marker::ADD;
	path_marker.id = 0;
	path_marker.type = visualization_msgs::Marker::LINE_STRIP;
	path_marker.pose.orientation.w = 1;
	path_marker.scale.x = 0.1;
	path_marker.color.r = 1.0;
	path_marker.color.g = 0.0;
	path_marker.color.b = 0.0;
	path_marker.color.a = 1.0;

  while (nodes_.at(idx).parent != idx) {
    for (auto it = nodes_[idx].path_from_parent->rbegin(); it != nodes_[idx].path_from_parent->rend(); ++it)
      path_marker.points.push_back(fromConfigurationToPoint3D_(*it));
    idx = nodes_[idx].parent;
  }

  path_marker.header.stamp = ros::Time::now();
  pub_marker_.publish(path_marker);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename RobotConfiguration>
void RRT<RobotConfiguration>::visualizeTree_() {
  visualization_msgs::Marker tree_marker;
  tree_marker.header.frame_id = "map";
	tree_marker.ns = "tree";
	tree_marker.action = visualization_msgs::Marker::ADD;
	tree_marker.id = 0;
	tree_marker.type = visualization_msgs::Marker::LINE_LIST;
	tree_marker.pose.orientation.w = 1;
	tree_marker.scale.x = 0.03;
	tree_marker.color.r = 0.0;
	tree_marker.color.g = 0.8;
	tree_marker.color.b = 1.0;
	tree_marker.color.a = 1.0;

  tree_marker.points.reserve(2 * nodes_.size()); // in general, more space is needed. Nevertheless, this reserve should be better than nothing...
  for (size_t i = 1; i < nodes_.size(); ++i) {
    for (size_t j = 0; j < nodes_.at(i).path_from_parent->size() - 1; ++j) {
      tree_marker.points.push_back(fromConfigurationToPoint3D_(nodes_[i].path_from_parent->at(j)));
      tree_marker.points.push_back(fromConfigurationToPoint3D_(nodes_[i].path_from_parent->at(j + 1)));
    }
  }

  tree_marker.header.stamp = ros::Time::now();
  pub_marker_.publish(tree_marker);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename RobotConfiguration>
void RRT<RobotConfiguration>::includeAncestors_(std::vector<size_t> &indexes) {
  size_t initial_size = indexes.size();
  indexes.reserve(initial_size * (ancestors_depth_ + 1));

  // add ancestors
  for (size_t i = 0; i < initial_size; ++i) {
    unsigned int counter = 0;
    size_t idx = indexes[i];
    while (counter < ancestors_depth_) {
       indexes.push_back(nodes_[idx].parent);
       idx = nodes_[idx].parent;
       ++counter;
    }
  }

  // remove duplicates
  std::sort(indexes.begin(), indexes.end());
  indexes.erase(std::unique(indexes.begin(), indexes.end()), indexes.end());
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename RobotConfiguration>
double RRT<RobotConfiguration>::computeSquaredDistance_(const RobotConfiguration &conf1, const RobotConfiguration &conf2) {
  double dx = conf1.position.x - conf2.position.x;
  double dy = conf1.position.y - conf2.position.y;
  return dx * dx + dy * dy;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename RobotConfiguration>
RobotConfiguration RRT<RobotConfiguration>::steeringFunction_(const RobotConfiguration &q_nearest, const RobotConfiguration &q_rand) {
  RobotConfiguration q_steer = q_rand;
  double squared_dist = computeSquaredDistance_(q_nearest, q_rand);
  if (squared_dist > steering_distance_ * steering_distance_) {
    double dist = std::sqrt(squared_dist);
    double tmp1 = steering_distance_ / dist;
    double tmp2 = (1.0 / tmp1) - 1.0;
    q_steer.position.x = tmp1 * (tmp2 * q_nearest.position.x + q_rand.position.x);
    q_steer.position.y = tmp1 * (tmp2 * q_nearest.position.y + q_rand.position.y);
    return q_steer;
  } else
    return q_rand;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename RobotConfiguration>
void RRT<RobotConfiguration>::searchPath_() {
  ROS_INFO("Starting search...");
  ros::Time start_time = ros::Time::now();
  size_t goal_idx;
  nodes_.resize(1);
  nodes_[0].configuration = start_node_.value();
  nodes_[0].id = 0;
  nodes_[0].parent = nodes_[0].id;
  nodes_[0].cost = 0.0;
  nodes_[0].path_from_parent.reset(new std::vector<RobotConfiguration>{});

  bool path_found = false;
  unsigned long int iterations = 0;
  while (!path_found || iterations < min_iterations_) {
    RobotConfiguration q_rand = (++iterations % 100 == 0 && !path_found) ? goal_node_.value() : randomRobotConfiguration_();
    size_t nearest_idx = findNearestNode_(q_rand);
    RobotConfiguration q_steer = steeringFunction_(nodes_.at(nearest_idx).configuration, q_rand);
    if (isValidConfiguration_(q_steer)) {
      // connect along a minimum cost path
      auto path = computeMinCostPath_(nodes_.at(nearest_idx).configuration, q_steer);
      if (collisionFree_(path.first)) {
        double c_min = nodes_.at(nearest_idx).cost + path.second;
        size_t id_min = nearest_idx;
        // double radius = std::min(radius_constant_ * std::pow(std::log(nodes_.size()) / nodes_.size(), 1.0 / dimensionality_), steering_distance_);
        double radius = steering_distance_; // set the radius with a simple and effective heuristic
        std::vector<size_t> near_indexes = findNodesWithinRadius_(q_steer, radius);
        includeAncestors_(near_indexes);
        std::unique_ptr<std::vector<RobotConfiguration>> best_path = std::make_unique<std::vector<RobotConfiguration>>(path.first);
        for (const auto &id : near_indexes) {
          auto new_path = computeMinCostPath_(nodes_.at(id).configuration, q_steer);
          double new_cost = new_path.second + nodes_[id].cost;
          if (new_cost < c_min && collisionFree_(new_path.first)) {
            c_min = new_cost;
            id_min = id;
            best_path = std::make_unique<std::vector<RobotConfiguration>>(new_path.first);
          }
        }
        nodes_.push_back(Node{.configuration = q_steer, .id = nodes_.size(), .parent = id_min, .cost = c_min, .path_from_parent = std::move(best_path)});

        // rewire the tree
        std::vector<size_t> candidate_parents = {nodes_.back().id};
        candidate_parents.reserve(ancestors_depth_ + 1);
        for (size_t i = 0; i < ancestors_depth_; ++i)
          candidate_parents.push_back(nodes_.at(candidate_parents.back()).parent);
        for (const auto new_parent_id : candidate_parents) {
          for (const auto &id : near_indexes) {
            auto new_path = computeMinCostPath_(nodes_[new_parent_id].configuration, nodes_.at(id).configuration);
            double new_cost = new_path.second + nodes_[new_parent_id].cost;
            if (new_cost < nodes_[id].cost && collisionFree_(new_path.first)) {
              nodes_[id].cost = new_cost;
              nodes_[id].parent = new_parent_id;
              nodes_[id].path_from_parent = std::make_unique<std::vector<RobotConfiguration>>(new_path.first);
            }
          }
        }

        if (isGoal_(q_steer)) {
          path_found = true;
          goal_idx = nodes_.size() - 1;
          ROS_INFO("Path found!");
        }
      }
    }
  }
  ros::Time end_time = ros::Time::now();
  ROS_INFO("Search took: %f s", (end_time - start_time).toSec());
  ROS_INFO("Total cost is: %f", nodes_[goal_idx].cost);
  ROS_INFO("Tree size is: %lu", nodes_.size());
  ROS_INFO("Total iterations: %lu", iterations);
  ROS_INFO("#################################################");

  visualizeTree_();
  visualizePath_();

}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

// force the template to generate the needed classes
template class RRT<Configuration2D>;
template class RRT<ConfigurationDiffDrive>;
