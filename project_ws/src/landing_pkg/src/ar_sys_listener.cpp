#include "ros/ros.h"
// #include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"

using namespace std;
using namespace std_msgs;
using namespace sensor_msgs;

// a callback function
void imageRawReceived(const Image &msg) {

    // auto seq = header.seq;
    // auto time = header.stamp;
    // auto frame_id = header.frame_id;

	// auto a = 8 + 9; auto not upported by ros gcc

	// Simple commit

	ROS_INFO("Received!");

	Header header = msg.header;
	uint32_t seq = header.seq;
	ros::Time stamp = header.stamp;
	double stamp_sec = stamp.toSec();
	string frame_id = header.frame_id;

	ROS_INFO("Header seq is %u", seq);
	ROS_INFO("Header stamp is %lf", stamp_sec);
	ROS_INFO_STREAM("Header frame_id is " << frame_id);	
}

int main(int argc, char ** argv) {
	ros::init(argc, argv, "ar_sys_listener");
	ros::NodeHandle nodeHandle;
	
	ROS_INFO("Avviato!");

	// create a subscriber object
	// ros::Subscriber sub = nodeHandle.subscribe("camera1/image_raw", 1000, &imageRawReceived);
	ros::Subscriber sub = nodeHandle.subscribe("ardrone/bottom/image_raw", 1000, &imageRawReceived);


	// let ROS take over
	ros::spin();
}
