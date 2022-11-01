#include <rrt/rrt.h>

#include <memory>

int main(int argc, char** argv) {

	ros::init(argc, argv, "rrt_node");
	
	ros::NodeHandle nh("~");

	rrt::RRT node(&nh);

	ROS_INFO("Node initializated. Starting to spin");

	ros::spin();

	return 0;
}