#pragma once

#include <rrt/cubic_spline.h>
#include <rrt/dubins.h>
#include <rrt/grid.h>
#include <rrt/posq.h>
#include <rrt/random_pose_generator.h>
#include <rrt/tree.h>
#include <rrt/types.h>

#include <cstddef>
#include <memory>
#include <functional>

namespace core {

struct SearchParams {
  unsigned long int min_iterations;
  unsigned long int max_iterations;
  double steering_distance;
  double steering_angular_distance;
  double radius_constant;
  long unsigned int ancestors_depth;
};

struct RRTInput {
  types::Pose2D start;
  types::Pose2D goal;
};

struct RRTOutput {
  types::Path path;
  double path_cost;
  const std::vector<tree::Node>& nodes;
  double search_time;
  long unsigned int iterations;
};

class RRTCore {
public:
  RRTCore();

  void setMap(const grid::Map &map);

  enum class EXTEND_FUNCTION {
    DUBINS,
    POSQ,
    SPLINE
  };
  bool setExtendFunction(const EXTEND_FUNCTION &function_type);

  bool setSearchParams(const SearchParams &search_params);

  bool setDubinsParams(const dubins::DubinsParams &params);

  bool setPOSQParams(const posq::POSQParams &params);
  
  bool setSplineParams(const spline::SplineParams &params);

  [[nodiscard]] RRTOutput searchPath(const RRTInput &input);


private:
  std::unique_ptr<grid::Grid> grid_;
  std::unique_ptr<RandomPoseGenerator> pose_generator_;
  tree::Tree tree_;

  const unsigned int dimensionality_ = 3;

  SearchParams search_params_{
    .min_iterations = 1200,
    .max_iterations = 20000,
    .steering_distance = 3.0,
    .steering_angular_distance = M_PI_2,
    .radius_constant = 18.0,
    .ancestors_depth = 1
  };

  // dubins extend function param
  dubins::DubinsParams dubins_params_{
    .turning_radius = 1.2
  };
  // posq extend function param
  posq::POSQParams posq_params_{
    .K_rho = 0.6,
    .K_phi = -2.0,
    .K_alpha = 5.0,
    .K_v = 2.0,
  };
  // spline extend function param
  spline::SplineParams spline_params_{
    .tf = 4.0,
    .v = 1.6
  };

  // function used for computing paths (it returns a pair composed by the path from parent to child, and the path's cost)
  std::function<std::pair<std::vector<types::Pose2D>, double>(const types::Pose2D &parent, const types::Pose2D &child)> compute_path_;
};

}