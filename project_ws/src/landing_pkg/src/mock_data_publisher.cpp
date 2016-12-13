#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>

using namespace std;
using namespace std_msgs;
using namespace geometry_msgs;
using namespace ardrone_autonomy;

unsigned int rand_in_range(unsigned int min, unsigned int max)
{
    double scaled = (double)rand()/RAND_MAX;

    return (max - min +1)*scaled + min;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mock_data_publisher");
    ros::NodeHandle nh;

    // TODO anche Header altrimenti sincronizzazione non va!
    ros::Publisher mock_pose_pub = nh.advertise<PoseStamped>("/ar_single_board/pose", 1);
    ros::Publisher mock_navdata_pub = nh.advertise<Navdata>("/ardrone/navdata", 1);

    ros::Rate r(50);
    while (ros::ok()) {

        int random_x = rand_in_range(0, 20);
        int random_y = rand_in_range(0, 20);
        int random_altd = rand_in_range(198, 200);

        PoseStamped pose;
        pose.pose.position.x = random_x;
        pose.pose.position.y = random_y;

        Navdata navdata;
        navdata.altd = random_altd; // in cm

        mock_pose_pub.publish(pose);
        ROS_INFO("mock_data_publisher: Sent pose (x=%lf, y=%lf)", pose.pose.position.x, pose.pose.position.y);

        mock_navdata_pub.publish(navdata);
        ROS_INFO("mock_data_publisher: Sent navdata (altd=%i)", navdata.altd);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}