#include <rrt/rrt_ros_interface.h>

int main(int argc, char** argv) {

	ros::init(argc, argv, "rrt_node");
	
	ros::NodeHandle nh("~");

	ros_interface::RRTRosInterface node(&nh);

	ROS_INFO("Node initializated. Starting to spin");

	ros::spin();

	return 0;
}