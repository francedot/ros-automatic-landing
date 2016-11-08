#include "ros/ros.h"
#include "std_msgs/String.h"

/// Non più necessario!

using namespace std;
using namespace std_msgs;

// a mock callback function
void positionReceived(const std_msgs::Float64ConstPtr &msg) {
	ROS_INFO("Received!");	
}

int main(int argc, char ** argv) {
	ros::init(argc, argv, "transform_bootstrapper");
	ros::NodeHandle nodeHandle;
	
	ROS_INFO("transform_bootstrapper initialized!");

	// create a subscriber object
	// ros::Subscriber sub = nodeHandle.subscribe("camera1/image_raw", 1000, &positionReceived);
	ros::Subscriber sub_x = nodeHandle.subscribe("/x_pose", 1000, &positionReceived);
	ros::Subscriber sub_y = nodeHandle.subscribe("/y_pose", 1000, &positionReceived);
	ros::Subscriber sub_z = nodeHandle.subscribe("/z_pose", 1000, &positionReceived);

	// let ROS take over
	ros::spin();
}
