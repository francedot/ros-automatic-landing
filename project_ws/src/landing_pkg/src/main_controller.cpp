#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <pid/StampedFloat64.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <landing_pkg/ErrorStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

using namespace tf;
using namespace pid;
using namespace std_msgs;
using namespace geometry_msgs;
using namespace message_filters;
using namespace landing_pkg;

/* STATE VARIABLES */
ros::Publisher tw_effort_pub;
ros::Publisher landing_pub;
ros::Publisher z_effort_pub;
Twist next_twist;
boost::shared_ptr <tf::TransformListener> listener;
tf::StampedTransform transform;
ros::Time last_cmd_vel_time;
float last_z_effort = 0.0f;
bool is_transform_setup = false;
bool is_in_quota = false;
bool is_reaching_quota_mode;

/* PARAMS */
bool enabled = true;
bool has_to_reach_quota = false;
double landing_quota = 2000;
double zeta_land_vel = -1;
double quota_reached_toleration = 250;

/* CALLBACKS */
void pose_error_received(const ErrorStampedConstPtr &error);

void effort_received(const StampedFloat64ConstPtr &x_effort,
                     const StampedFloat64ConstPtr &y_effort
        /*, const StampedFloat64ConstPtr& yawEffort*/);

/* HELPER FUNCTIONS */
void ros_loop_continue(ros::Rate r);

double calculate_linear_z();


int main(int argc, char **argv) {
    ros::init(argc, argv, "main_controller");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    /* PARAMS Definition */
    pnh.param("enabled", enabled, true);
    pnh.param("has_to_reach_quota", has_to_reach_quota, false);
    pnh.param("landing_quota", landing_quota, 2000.0); // millimeters
    pnh.param("zeta_land_vel", zeta_land_vel, 0.2);
    pnh.param("quota_reached_toleration", quota_reached_toleration, 250.0); // millimeters toleration

    listener.reset(new tf::TransformListener);
    tw_effort_pub = nh.advertise<Twist>("cmd_vel", 1);

    /* X,Y Efforts Synchronization */
    message_filters::Subscriber <StampedFloat64> x_effort_sub(nh, "x_effort", 1);
    message_filters::Subscriber <StampedFloat64> y_effort_sub(nh, "y_effort", 1);
    typedef sync_policies::ExactTime <StampedFloat64, StampedFloat64> MySyncPolicy;
    Synchronizer <MySyncPolicy> sync(MySyncPolicy(10), x_effort_sub, y_effort_sub);
    sync.registerCallback(&effort_received);

    is_reaching_quota_mode = has_to_reach_quota;
    if (is_reaching_quota_mode) {
        ROS_INFO("main_controller: Entered Reaching Quota Mode");
    }

    landing_pub = nh.advertise<Empty>("/ardrone/land",1);

    z_effort_pub = nh.advertise<Float64>("/ardrone/z_effort",1);

    ros::Duration timeout;
    timeout = ros::Duration(1, 0);
    ros::Rate r(50);
    while (ros::ok()) {

        if (ros::Time::now() - last_cmd_vel_time > timeout) {
            next_twist.linear.x = 0;
            next_twist.linear.y = 0;
            //next_twist.linear.z = 0;
            next_twist.angular.x = 0;
            next_twist.angular.y = 0;
            next_twist.angular.z = 0;
        }
        tw_effort_pub.publish(next_twist);
        // ROS_INFO("main_controller: Sent Twist (%lf, %lf, %lf)", next_twist.linear.x, next_twist.linear.y, next_twist.linear.z);

        ros_loop_continue(r);
    }
    return 0;
}

void effort_received(const StampedFloat64ConstPtr &x_effort,
                     const StampedFloat64ConstPtr &y_effort
        /*, const StampedFloat64ConstPtr& yawEffort*/) {
    if (!enabled)
        return;

    if (!is_transform_setup) {
        try {
            listener->lookupTransform("ardrone_base_link", "ardrone_base_bottomcam", ros::Time(0), transform);
            is_transform_setup = true;
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            return;
        }
    }
    tf::Vector3 linear(x_effort->c, y_effort->c, 0);
    linear = (transform * linear) - (transform * tf::Vector3(0, 0, 0));
    next_twist.linear.x = linear.x();
    next_twist.linear.y = linear.y();
    last_cmd_vel_time = ros::Time::now();
}

/*double calculate_linear_z() {

    double result = last_z_effort;

    if (is_reaching_quota_mode) {

        double z_error_abs = abs(cur_quota.data - landing_quota);

        is_in_quota = z_error_abs > landing_quota - quota_reached_toleration &&
                      z_error_abs < landing_quota + quota_reached_toleration;

        if (!is_in_quota) {
            bool is_negative_z_effort = cur_quota.data > landing_quota;
            result = (is_negative_z_effort ? -1 : 1) * zeta_land_vel;
        } else {
            is_reaching_quota_mode = false;
        }
    }

    last_z_effort = result;

    return result;
}*/

void ros_loop_continue(ros::Rate r) {
    ros::spinOnce();
    r.sleep();
}