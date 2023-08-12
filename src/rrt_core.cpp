#include <rrt/rrt_core.h>

#include <rrt/cubic_spline.h>
#include <rrt/dubins.h>
#include <rrt/posq.h>
#include <rrt/tree.h>
#include <rrt/types.h>
#include <rrt/utils.h>

#include <chrono>
#include <cstddef>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

namespace core {

RRTCore::RRTCore() : compute_path_([&](const types::Pose2D &parent, const types::Pose2D &child) {
                        return dubins::computeShortestDubinsPath(parent, child, dubins_params_);
                     })
                      {}

void RRTCore::setMap(const grid::Map &map) {
  grid_ = std::make_unique<grid::Grid>(map);
  pose_generator_ = std::make_unique<RandomPoseGenerator>(map.origin.x,
                                                          map.origin.y,
                                                          map.origin.x + static_cast<double>(map.width) * map.resolution,
                                                          map.origin.y + static_cast<double>(map.height) * map.resolution);
}

bool RRTCore::setExtendFunction(const EXTEND_FUNCTION &function_type) {
  if (function_type == EXTEND_FUNCTION::DUBINS)
    compute_path_ = [&](const types::Pose2D &parent, const types::Pose2D &child) {
      return dubins::computeShortestDubinsPath(parent, child, dubins_params_);
    };
  else if (function_type == EXTEND_FUNCTION::POSQ)
    compute_path_ = [&](const types::Pose2D &parent, const types::Pose2D &child) {
      return posq::computePOSQPath(parent, child, posq_params_);
    };
  else if (function_type == EXTEND_FUNCTION::SPLINE)
    compute_path_ = [&](const types::Pose2D &parent, const types::Pose2D &child) {
      return spline::computeSplinePath(parent, child, spline_params_);
    };
  else
    return false;
  
  return true;
}

bool RRTCore::setSearchParams(const SearchParams &search_params) {
  if (search_params.steering_distance <= 0.0 ||
      search_params.steering_angular_distance <= 0.0 ||
      search_params.radius_constant <= 0.0)
    return false;
  search_params_ = search_params;
  return true;
}

bool RRTCore::setDubinsParams(const dubins::DubinsParams &params) {
  if (params.turning_radius <= 0)
    return false;
  dubins_params_ = params;
  return true;
}

bool RRTCore::setPOSQParams(const posq::POSQParams &params) {
  if (params.K_v <= 0 || params.K_rho <= 0 || params.K_phi >= 0 || params.K_alpha + params.K_phi - params.K_rho * params.K_v <= 0)
    return false;
  posq_params_ = params;
  return true;
}

bool RRTCore::setSplineParams(const spline::SplineParams &params) {
  if (params.tf <= 0 || params.v <= 0)
    return false;
  spline_params_ = params;
  return true;
}

RRTOutput RRTCore::searchPath(const RRTInput &input) {
  auto start_time = std::chrono::high_resolution_clock::now();

  if (!grid_ || !pose_generator_)
    throw std::runtime_error("Can't search a path if map is not set!");

  tree_.initializeTree(input.start);
  tree_.reserve(search_params_.max_iterations);
  
  if (!grid_->isValidPosition(input.start.position) || !grid_->isValidPosition(input.goal.position)){
    std::chrono::duration<double, std::deca> elapsed_time = std::chrono::high_resolution_clock::now() - start_time;
    return RRTOutput{.path = {},
                     .path_cost = std::numeric_limits<double>::infinity(),
                     .nodes = tree_.getTree(),
                     .search_time = elapsed_time.count(),
                     .iterations = 0};
  }

  bool path_found = false;
  unsigned long int iterations = 0;
  double total_cost = std::numeric_limits<double>::infinity();
  tree::Node goal_node;
  while ((!path_found || iterations < search_params_.min_iterations) && iterations < search_params_.max_iterations) {
    types::Pose2D q_rand = (iterations++ % 100 == 0 && !path_found) ? input.goal : pose_generator_->sample();
    node_id nearest_node_id = tree_.findNearestNode(q_rand);
    auto nearest_node = tree_.getNodeById(nearest_node_id);
    auto q_steer = types::steeringFunction(nearest_node.configuration, q_rand, search_params_.steering_distance, search_params_.steering_angular_distance);
    if (grid_->isValidPosition(q_steer.position)) {
      // connect along a minimum cost path
      auto path = compute_path_(nearest_node.configuration, q_steer);
      if (path.second < std::numeric_limits<double>::infinity() && grid_->collisionFree(path.first)) {
        double c_min = nearest_node.cost + path.second;
        node_id id_min = nearest_node_id;
        
        // get candidate parents
        double radius = std::min(search_params_.radius_constant * std::pow(std::log(tree_.getTreeSize()) / static_cast<double>(tree_.getTreeSize()), 1.0 / dimensionality_), search_params_.steering_distance);
        std::vector<node_id> near_nodes_id = tree_.findNodesWithinRadius(q_steer, radius);
        
        // include ancestors
        size_t initial_size = near_nodes_id.size();
        near_nodes_id.reserve(initial_size * (search_params_.ancestors_depth + 1));
        for (size_t i = 0; i < initial_size; ++i) {
          auto tmp_node = tree_.getNodeById(near_nodes_id[i]);
          for (size_t counter = 0; counter < search_params_.ancestors_depth; ++counter) {
            near_nodes_id.push_back(tmp_node.parent);
            tmp_node = tree_.getNodeById(tmp_node.parent);
          }
        }
        // remove duplicates
        std::sort(near_nodes_id.begin(), near_nodes_id.end());
        auto it = std::unique(near_nodes_id.begin(), near_nodes_id.end());
        if (it != near_nodes_id.end())
          near_nodes_id.erase(it);
      
        // find min cost parent and add node to tree
        std::unique_ptr<std::vector<types::Pose2D>> best_path = std::make_unique<std::vector<types::Pose2D>>(path.first);
        for (const auto &id : near_nodes_id) {
          auto new_path = compute_path_(tree_.getNodeById(id).configuration, q_steer);
          double new_cost = new_path.second + tree_.getNodeById(id).cost;
          if (new_cost < c_min && grid_->collisionFree(new_path.first)) {
            c_min = new_cost;
            id_min = id;
            best_path = std::make_unique<std::vector<types::Pose2D>>(new_path.first);
          }
        }
        auto new_node_id = tree_.insertNode(q_steer, id_min, c_min, *best_path);

        // rewire the tree
        std::vector<node_id> candidate_parents(search_params_.ancestors_depth + 1, new_node_id);
        for (size_t i = 1; i < candidate_parents.size(); ++i)
          candidate_parents[i] = tree_.getNodeById(candidate_parents[i - 1]).parent;
        for (const auto new_parent_id : candidate_parents) {
          auto new_parent_node = tree_.getNodeById(new_parent_id);
          for (const auto id : near_nodes_id) {
            auto current_node = tree_.getNodeById(id);
            auto new_path = compute_path_(new_parent_node.configuration, current_node.configuration);
            double new_cost = new_path.second + new_parent_node.cost;
            if (new_cost < current_node.cost && grid_->collisionFree(new_path.first))
              tree_.updateParent(id, new_parent_id, new_cost, new_path.first);
          }
        }
        if (q_steer == input.goal) {
          path_found = true;
          goal_node = tree_.getNodeById(new_node_id);
        }
      }
    }
  }

  // assemble output
  types::Path path_to_goal{};
  if (path_found) {
    total_cost = goal_node.cost;
    auto tmp_node = goal_node;
    while (tmp_node.parent != tmp_node.id) {
      path_to_goal.reserve(path_to_goal.size() + tmp_node.path_from_parent.size());
      for (auto it = tmp_node.path_from_parent.crbegin(); it != tmp_node.path_from_parent.crend(); ++it)
        path_to_goal.push_back(*it);
      tmp_node = tree_.getNodeById(tmp_node.parent);
    }
    // reverse in order to get path from start to goal
    std::reverse(path_to_goal.begin(), path_to_goal.end());
  }
  
  std::chrono::duration<double, std::deca> elapsed_time = std::chrono::high_resolution_clock::now() - start_time;
  return RRTOutput{.path = path_to_goal,
                   .path_cost = total_cost,
                   .nodes = tree_.getTree(),
                   .search_time = elapsed_time.count(),
                   .iterations = iterations};
}

}