// TODO: Make main controller subscribe this topic and send landing message

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <landing_pkg/ErrorStamped.h>

using namespace std;
using namespace std_msgs;
using namespace geometry_msgs;

ErrorStamped cur_pose_error;
Float64 cur_quota;

/* LANDING PARAMETERS */
double center_margin = 0.10; // Accepted distance from (0,0) in m
double speed_margin = 1; // Accepted position variation in m/s
double avg_center_margin = 0.8;
double avg_speed_margin = 0.5;
double landing_quote = 0.6; // Quote for landing signal

void cur_quota_received(const Float64 &cur_quota_) {
    cur_quota = cur_quota_;
    ROS_INFO("lander: Received quota %lf", cur_quota.data);
}

void pose_error_received(const ErrorStampedConstPtr &error) {
    cur_pose_error = *error;
    ROS_INFO(
            "lander: Received pose_error (ex=%lf, ey=%lf, dx=%lf, dy=%lf, avg_ex=%lf, avg_ey=%lf, avg_dx=%lf, avg_dy=%lf)",
            cur_pose_error.ex, cur_pose_error.ey, cur_pose_error.dx, cur_pose_error.dy, cur_pose_error.avg_ex,
            cur_pose_error.avg_ey,
            cur_pose_error.avg_dx, cur_pose_error.avg_dy);
}

bool check_landing_conditions() {
    return ((cur_pose_error.ex < center_margin) && (cur_pose_error.ey < center_margin) &&
            (cur_pose_error.dx < speed_margin) && (cur_pose_error.dy < speed_margin) &&
            (cur_pose_error.avg_ex < avg_center_margin) && (cur_pose_error.avg_ey < avg_center_margin) &&
            (cur_pose_error.avg_dx < avg_speed_margin) && (cur_pose_error.avg_dy < avg_speed_margin));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lander");
    ros::NodeHandle nodeHandle;
    ros::NodeHandle pnh("~");

    pnh.param("lambda",lambda,false);

    // create a subscriber object
    ros::Subscriber sub_reach_quota = nh.subscribe("/ardrone/cur_quota", 1, &cur_quota_received);
    ros::Subscriber sub_pose_error = nh.subscribe("/pose_error", 1, &pose_error_received);

    ros::Rate r(50);
    while (ros::ok()) {

        if (check_landing_conditions()) {
            ROS_INFO("lander: check_landing_conditions=true");
            Float64 z_effort;
            z_effort.data = zeta_land_vel;
            z_effort_pub.publish(z_effort);
            if (cur_quota <= landing_quote) {
                z_effort.data = 0;
                z_effort_pub.publish(z_effort);
                landing_pub.publish(Empty());
            }
        }

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
