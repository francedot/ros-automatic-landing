#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <pid/StampedFloat64.h>
#include <tf/transform_datatypes.h>

using namespace tf;
using namespace std;
using namespace pid;
using namespace std_msgs;
using namespace geometry_msgs;

ros::Publisher chatter_pub_yaw;

void positionReceived(const geometry_msgs::PoseStamped &msg) {
    pid::StampedFloat64 sf_yaw;

    sf_yaw.header = msg.header;
    sf_yaw.c = getYaw(msg.pose.orientation);

    chatter_pub_yaw.publish(sf_yaw);
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "transform_to_stamped_yaw");
    ros::NodeHandle nh;

    // create a subscriber object
    ros::Subscriber sub_x = nh.subscribe("/ar_single_board/pose", 1000, &positionReceived);

    chatter_pub_yaw = nh.advertise<pid::StampedFloat64>("/yaw_pose", 1000);

    // let ROS take over
    ros::spin();
}
