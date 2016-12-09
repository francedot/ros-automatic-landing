#include <ros/ros.h>
#include <pid/StampedFloat64.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <iostream>
#include <sys/time.h>
#include <unistd.h>
#include <std_msgs/Float64.h>

using namespace tf;
using namespace pid;
using namespace std_msgs;
using namespace geometry_msgs;
using namespace message_filters;

ros::Publisher tw_effort_pub;
boost::shared_ptr<tf::TransformListener> listener;
tf::StampedTransform transform;
bool transform_setup = false;
bool is_in_quota = false;

bool is_reaching_quota_mode;
bool is_landing_mode;

float last_z_effort = 0.0f;

struct timeval last_callback, max_time_for_effort;

double calculate_linear_z();

bool is_stop_message_sent = false;

/* PARAMS */
bool enabled = true;
bool has_to_reach_quota = false;
double landing_quota = 2000;
double zeta_land_vel = -0.2;
double quota_reached_toleration = 250;

Float64ConstPtr cur_quota;

void set_max_time_for_effort() {
    // Sets max time before sending an empty twist to the drone
    // TO-DO: Figure out how long should be this interval
    //	    It could be K * AVG[Times_of_efforts_arrivals] with 0 < k < 1
    max_time_for_effort.tv_sec = 0;
    max_time_for_effort.tv_usec = 500000;
}

void send_stop_message() {
    // Sends an empty twist to the drone through the cmd_vel topic
    is_stop_message_sent = true;
    Twist tw;
    tw.linear.x = 0;
    tw.linear.y = 0;
    tw.linear.z = 0;
    tw.angular.x = 0;
    tw.angular.y = 0;
    tw.angular.z = 0;
    tw_effort_pub.publish(tw);
    ROS_INFO("vel_controller: Sent Twist Stop Message");
}

void zEffortReceived(const Float64ConstPtr &zEffort) {
    last_z_effort = zEffort->data;
}

void cur_quota_received(const Float64ConstPtr &cur_quota_) {
    cur_quota = cur_quota;
    ROS_INFO("vel_controller: Received quota %lf", cur_quota->data);
}

/* Movement is specified in meters per second.
 * Maximum value specified should be no more than 0.8.
 * For maximum stability while flying, a value of .5 or lower is suggested.*/
void effortReceived(const StampedFloat64ConstPtr &xEffort,
                    const StampedFloat64ConstPtr &yEffort/*, const StampedFloat64ConstPtr& yawEffort*/) {
    if (!enabled)
        return;
    /*
        Called when an <x,y> control effort arrives.
        Transforms from camera to drone's coordinate system.
        Writes Twist(x,y,0) to cmd_vel.
    */
    gettimeofday(&last_callback, NULL);
    is_stop_message_sent = false;
    if (!transform_setup) {
        try {
            listener->lookupTransform("ardrone_base_link", "ardrone_base_bottomcam", ros::Time(0), transform);
            transform_setup = true;
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            return;
        }
    }
    // Mod
/*  Quaternion fromYaw = tf::createQuaternionFromYaw(yawEffort->c);
  fromYaw = transform * fromYaw;
  double toYaw = getYaw(fromYaw);
*/  //

    tf::Vector3 linear(xEffort->c, yEffort->c, 0);
    linear = (transform * linear) - (transform * tf::Vector3(0, 0, 0));

    Twist tw;
    tw.linear.x = linear.x();
    tw.linear.y = linear.y();
    tw.linear.z = calculate_linear_z();

    tw_effort_pub.publish(tw);
    ROS_INFO("vel_controller: Sent Twist (%lf, %lf, %lf)", tw.linear.x, tw.linear.y, tw.linear.z);

    last_z_effort = tw.linear.z;
}

double calculate_linear_z() {

    double result = last_z_effort;

    if (is_reaching_quota_mode) {

        double z_error_abs = abs(cur_quota->data - landing_quota);

        is_in_quota = z_error_abs > landing_quota - quota_reached_toleration &&
                      z_error_abs < landing_quota + quota_reached_toleration;

        if (!is_in_quota) {
            bool is_negative_z_effort = cur_quota->data > landing_quota;
            result = (is_negative_z_effort ? -1 : 1) * zeta_land_vel;
        } else {
            is_reaching_quota_mode = false;
        }
    }

    last_z_effort = result;

    return result;
}

int timeval_subtract(struct timeval *result, struct timeval *x, struct timeval *y) {
    /* Perform the carry for the later subtraction by updating y. */
    if (x->tv_usec < y->tv_usec) {
        int nsec = (y->tv_usec - x->tv_usec) / 1000000 + 1;
        y->tv_usec -= 1000000 * nsec;
        y->tv_sec += nsec;
    }
    if (x->tv_usec - y->tv_usec > 1000000) {
        int nsec = (x->tv_usec - y->tv_usec) / 1000000;
        y->tv_usec += 1000000 * nsec;
        y->tv_sec -= nsec;
    }

    /* Compute the time remaining to wait.
       tv_usec is certainly positive. */
    result->tv_sec = x->tv_sec - y->tv_sec;
    result->tv_usec = x->tv_usec - y->tv_usec;

    /* Return 1 if result is negative. */
    return x->tv_sec < y->tv_sec;
}

void ros_loop_continue(ros::Rate r){
    ros::spinOnce();
    r.sleep();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "vel_controller");
    ros::NodeHandle nh;

    ros::NodeHandle pnh("~");
    pnh.param("enabled", enabled, true);
    pnh.param("has_to_reach_quota", has_to_reach_quota, false);
    pnh.param("landing_quota", landing_quota, 2000.0); // millimeters
    pnh.param("zeta_land_vel", zeta_land_vel, 0.2);
    pnh.param("quota_reached_toleration", quota_reached_toleration, 250.0); // millimeters toleration

    listener.reset(new tf::TransformListener);
    tw_effort_pub = nh.advertise<Twist>("cmd_vel", 1);

    // ros::Subscriber sub_z_effort = nh.subscribe("ardrone/z_effort", 1000, &zEffortReceived);
    ros::Subscriber sub_reach_quota = nh.subscribe("ardrone/cur_quota", 1, &cur_quota_received);

    message_filters::Subscriber<StampedFloat64> x_effort_sub(nh, "x_effort", 1);
    message_filters::Subscriber<StampedFloat64> y_effort_sub(nh, "y_effort", 1);
    typedef sync_policies::ExactTime<StampedFloat64, StampedFloat64> MySyncPolicy;
    // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), x_effort_sub, y_effort_sub);
    sync.registerCallback(&effortReceived);

    struct timeval time_difference, current_time;
    set_max_time_for_effort();

    is_reaching_quota_mode = true;
    ROS_INFO("vel_controller: Entered Reaching Quota Mode");

    ros::Rate r(50);
    while (ros::ok()) {
        if (!is_stop_message_sent) {
            gettimeofday(&current_time, NULL);
            if (1 == timeval_subtract(&time_difference, &current_time, &last_callback)) {
                ROS_ERROR("vel_controller: time difference should not be negative");
                ros_loop_continue(r);
            }

            if ((time_difference.tv_sec >= max_time_for_effort.tv_sec) &&
                (time_difference.tv_usec > max_time_for_effort.tv_usec)) {
                    send_stop_message();
            }
        }
        ros_loop_continue(r);
    }
    return 0;
}
