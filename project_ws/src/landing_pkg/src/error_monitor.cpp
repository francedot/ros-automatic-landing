#include <ros/ros.h>
#include <cmath>
#include <std_msgs/Header.h>
#include <landing_pkg/ErrorStamped.h>
#include <geometry_msgs/PoseStamped.h>

// error on x and y
double ex = 0.0;
double ey = 0.0;

// derivative of x and y
double dx = 0.0;
double dy = 0.0;

// Average of the above variables in the last n
// seconds where n is indicated by average_seconds
double avg_ex = 0.0;
double avg_ey = 0.0;
double avg_dx = 0.0;
double avg_dy = 0.0;
//long avg_index = 0.0;

const int size_of_average = 100;
double avg_ex_samples[size_of_average];
double avg_ey_samples[size_of_average];
double avg_dx_samples[size_of_average];
double avg_dy_samples[size_of_average];

long iteration = 0;

double last_ex = 0.0;
double last_ey = 0.0;

ros::Time prev_time;
ros::Duration delta_t;

//average_seconds set to 7 seconds
ros::Duration average_seconds = ros::Duration(7,0);
ros::Time start_of_average;

ros::Publisher error_pub;

void on_error_update(const geometry_msgs::PoseStampedConstPtr &stamped_msg);
//void update_averages(double new_val, double *avg, long *index);
double update_average(double new_val, double *average_array, const int size);


int main(int argc, char **argv) {

    ros::init(argc, argv, "error_monitor");
    ros::NodeHandle nh;

    ros::Subscriber pose_sub = nh.subscribe("/ar_single_board/pose", 1, on_error_update);
    error_pub = nh.advertise<landing_pkg::ErrorStamped>("/pose_error", 1);

    ros::spin();
    return 0;
}

void on_error_update(const geometry_msgs::PoseStampedConstPtr &stamped_msg) {
    if(!prev_time.isZero()) {
        delta_t = ros::Time::now() - prev_time;
        prev_time = ros::Time::now();
        if(0 == delta_t.toSec()){
            ROS_ERROR("error_monitor: delta_t to zero: cannot compute derivative.\n");
            return;
        }
    }else{
        ex = fabs(stamped_msg->pose.position.x);
        ey = fabs(stamped_msg->pose.position.y);
        //avg_index = 0;
        //start_of_average = ros::Time::now();
        prev_time = ros::Time::now();
        return;
    }

    // Assuming setpoint = (0,0)
    last_ex = ex;
    last_ey = ey;
    ex = fabs(stamped_msg->pose.position.x);
    ey = fabs(stamped_msg->pose.position.y);

    dx = fabs(ex - last_ex) / delta_t.toSec();
    dy = fabs(ey - last_ey) / delta_t.toSec();

    avg_ex = update_average(ex,avg_ex_samples, size_of_average);
    avg_ey = update_average(ey,avg_ey_samples, size_of_average);
    avg_dx = update_average(dx,avg_dx_samples, size_of_average);
    avg_dy = update_average(dy,avg_dy_samples, size_of_average);

    /*

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
*/
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

    iteration++;

    error_pub.publish(output_error);
    ROS_INFO("error_monitor: Sent pose_error (ex=%lf, ey=%lf, dx=%lf, dy=%lf, avg_ex=%lf, avg_ey=%lf, avg_dx=%lf, avg_dy=%lf)",
             output_error.ex, output_error.ey, output_error.dx, output_error.dy, output_error.avg_ex, output_error.avg_ey,
             output_error.avg_dx, output_error.avg_dy);
}

/*
void update_averages(double new_val, double *avg, long *index) {
    (*index) += 1;
    double diff = new_val - (*avg);
    (*avg) = (*avg) + (diff / (*index));
}*/

double update_average(double new_val, double *average_array, const int size) {
    int index = iteration % size;
    average_array[index] = new_val;
    double avg = 0;
    for(int i = 0; i < size; i++) {
        avg = average_array[i];
    }
    return avg/size;
}