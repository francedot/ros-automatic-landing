#include <ros/ros.h>
#include <ros/duration.h>
#include <std_msgs/Header.h>
#include <landing_pkg/ErrorStamped.h>
#include <geometry_msgs/PoseStamped.h>

// error on x and y
double ex;
double ey;

// derivative of x and y
double dx;
double dy;

// Average of the above variables in the last n
// seconds where n is indicated by average_seconds
double avg_ex;
double avg_ey;
double avg_dx;
double avg_dy;
long avg_index;

double last_ex;
double last_ey;

ros::Time prev_time;
ros::Duration delta_t;

//average_seconds set to 5 seconds
ros::Duration average_seconds = ros::Duration(5,0);
ros::Time start_of_average;

ros::Publisher error_pub;

void update_averages(double new_val, double *avg, long *index) {
    (*index) += 1;
    double diff = new_val - (*avg);
    (*avg) = (*avg) + (diff / (*index));
}

void error_update(const geometry_msgs::PoseStampedConstPtr& stamped_msg) {
    if(!prev_time.isZero()) {
        delta_t = ros::Time::now() - prev_time;
        prev_time = ros::Time::now();
        if(0 == delta_t.toSec()){
            ROS_ERROR("delta_t to zero: cannot compute derivative.\n");
            return;
        }
    }else{
        ex = 0;
        ey = 0;
        avg_index = 0;
        start_of_average = ros::Time::now();
        prev_time = ros::Time::now();
        return;
    }

    // Assuming setpoint = (0,0)
    last_ex = ex;
    last_ey = ey;
    ex = stamped_msg->pose.position.x;
    ey = stamped_msg->pose.position.y;

    dx = (ex - last_ex) / delta_t.toSec();
    dy = (ey - last_ey) / delta_t.toSec();

    if(ros::Time::now() - start_of_average >= average_seconds) {
        avg_ex = 0;
        avg_ey = 0;
        avg_dx = 0;
        avg_dy = 0;
        avg_index = 0;
        start_of_average = ros::Time::now();
    }else{
        update_averages(ex, &avg_ex, &avg_index);
        avg_index--;
        update_averages(ey, &avg_ey, &avg_index);
        avg_index--;
        update_averages(dx, &avg_dx, &avg_index);
        avg_index--;
        update_averages(dy, &avg_dy, &avg_index);
    }

    landing_pkg::ErrorStamped output_error;
    output_error.header = stamped_msg->header;
    output_error.ex = ex;
    output_error.ey = ey;
    output_error.dx = dx;
    output_error.dy = dy;
    output_error.avg_ex = avg_ex;
    output_error.avg_ey = avg_ey;
    output_error.avg_dx = avg_dx;
    output_error.avg_dy = avg_dy;

    error_pub.publish(output_error);
}

int main(int argc, char **argv) {
    ROS_INFO("Started Error Monitor node.\n");

    ros::init(argc, argv, "Error Monitor");
    ros::NodeHandle nh;

    ros::Subscriber pose_sub = node.subscribe("/ar_single_board", 1, error_update);
    error_pub = nh.advertise<landing_pkg::ErrorStamped>("/error_topic", 1);

    ros::spin();
    return 0;
}