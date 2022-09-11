#include <rrt/rrt_2D.h>
#include <rrt/rrt_diff_drive.h>

#include <memory>

int main(int argc, char** argv) {

	ros::init(argc, argv, "rrt_node");
	
  ros::NodeHandle nh("~");

  bool use_diff_drive;
  nh.param("differential_drive", use_diff_drive, false);

  // Is there a better way to do this? Defining a dummy non-templated base class in order to use polymorphism does not drive me crazy
  std::unique_ptr<DummyClass> node;
  if (use_diff_drive)
    node.reset(new RRTDiffDrive{&nh});
  else
    node.reset(new RRT2D{&nh});

	ROS_INFO("Node initializated. Starting to spin");

	ros::spin();

	return 0;
}