#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped"

using namespace std;
using namespace std_msgs;
using namespace geometry_msgs;

ros::Publisher chatter_pub_yaw;
ros::Publisher chatter_pub_y;

void positionReceived(const geometry_msgs::PoseStamped &msg) {
	pid::StampedFloat64 sfX;
	pid::StampedFloat64 sfY;

	sfX.header = msg.header;
	sfX.c = msg.pose.position.x;

	sfY.header = msg.header;
	sfY.c = msg.pose.position.y;

	chatter_pub_yaw.publish(sfX);
	chatter_pub_y.publish(sfY);
}

int main(int argc, char ** argv) {
	ros::init(argc, argv, "transform_to_stamped");
	ros::NodeHandle nodeHandle;
	
	// create a subscriber object
	ros::Subscriber sub_x = nodeHandle.subscribe("/x_pose", 1000, &positionReceived);

	ros::chatter_pub_x = nh.advertise<pid::StampedFloat64>("/x_pose", 1000);
	ros::chatter_pub_y = nh.advertise<pid::StampedFloat64>("/y_pose", 1000);
	
	// let ROS take over
	ros::spin();
}
