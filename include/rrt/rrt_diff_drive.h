#pragma once

#include <rrt/rrt.h>
#include <rrt/types.h>

#include <functional>

class RRTDiffDrive : public RRT<ConfigurationDiffDrive> {
public:
  RRTDiffDrive(ros::NodeHandle* nodehandle);

private:  
  virtual ConfigurationDiffDrive fromPoseToRobotConfiguration_(const geometry_msgs::Pose &pose) override final;

  virtual ConfigurationDiffDrive randomRobotConfiguration_() override final;

  // function used for computing paths
  std::function<std::pair<std::vector<ConfigurationDiffDrive>, double>(const ConfigurationDiffDrive &parent, const ConfigurationDiffDrive &child)> extend_function_;

  virtual std::pair<std::vector<ConfigurationDiffDrive>, double> computeMinCostPath_(const ConfigurationDiffDrive &parent,
                                                                                     const ConfigurationDiffDrive &child) override final;
};