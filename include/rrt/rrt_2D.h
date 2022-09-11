#pragma once

#include <rrt/rrt.h>
#include <rrt/types.h>

class RRT2D : public RRT<Configuration2D> {
public:
  RRT2D(ros::NodeHandle* nodehandle);

private:  
  virtual Configuration2D fromPoseToRobotConfiguration_(const geometry_msgs::Pose &pose) override final;

  virtual Configuration2D randomRobotConfiguration_() override final;

  virtual std::pair<std::vector<Configuration2D>, double> computeMinCostPath_(const Configuration2D &parent, const Configuration2D &child) override final;
};