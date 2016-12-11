#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <ardrone_autonomy/Navdata.h>

using namespace std;
using namespace std_msgs;
using namespace geometry_msgs;
using namespace ardrone_autonomy;

ros::Publisher quota_pub;

// a callback function
void navdataReceived(const NavdataConstPtr &navdataReceived) {

    // altd given in cm from navdata
    double quota = (double) navdataReceived->altd * 10; // to uniform as mm

    Float64 alt;
    alt.data = quota;

    quota_pub.publish(alt);
    ROS_INFO("quota_monitor: Sent quota %lf", quota);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "quota_monitor");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("ardrone/navdata", 1, &navdataReceived);

    quota_pub = nh.advertise<Float64>("ardrone/cur_quota", 1);

    ros::spin();

    return 0;
}