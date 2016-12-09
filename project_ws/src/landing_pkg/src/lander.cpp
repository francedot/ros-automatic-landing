#include <std_msgs/Header.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
using namespace std_msgs;
using namespace geometry_msgs;

void ros_loop_continue(ros::Rate r);
double getError(Point point_);

// lambda = error toleration

/** Parameters **/
double lambda = 0.0;
double  = 0.0;


timeval last_landing_time;
PoseStampedConstPtr lastReceivedPose;

// a callback function
void poseReceived(const PoseStampedConstPtr &msg) {

    ROS_INFO("Received!");
    lastReceivedPose = msg;

    Header header = msg->header;
    uint32_t seq = header.seq;
    ros::Time stamp = header.stamp;
    double stamp_sec = stamp.toSec();
    string frame_id = header.frame_id;

    ROS_INFO("Header seq is %u", seq);
    ROS_INFO("Header stamp is %lf", stamp_sec);
    ROS_INFO_STREAM("Header frame_id is " << frame_id);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lander");
    ros::NodeHandle nodeHandle;
    ros::NodeHandle pnh("~");

    pnh.param("lambda",lambda,false);

    // ROS_INFO("Avviato!");

    // create a subscriber object
    ros::Subscriber sub = nodeHandle.subscribe("/ar_single_board/pose", 1, &poseReceived);

    ros::Rate r(50);
    while (ros::ok()) {

        if (lastReceivedPose == nullptr) {
            ROS_ERROR("lastReceivedPose not initialized");
            ros_loop_continue(r);
        }

        // logic for detecting error > lambda
        Point position = lastReceivedPose->pose.position;
        double error = getError(position);
        if (error < lambda) {
            // landing mode
            if (gettimeofday(&last_landing_time, NULL) < 0) {
                ROS_ERROR("Error on getting current time");
                ros_loop_continue(r);
            }
        } else {
            // holding mode
            timeval current_time;
            if (gettimeofday(&current_time, NULL) < 0) {
                ROS_ERROR("Error on getting current time");
                ros_loop_continue(r);
            }

        }

        ros_loop_continue(r);
    }
    return 0;
}


void ros_loop_continue(ros::Rate r){
    ros::spinOnce();
    r.sleep();
}

double getError(Point point_) {

    double distance =
            sqrt((point_.x - point_.x) * (point_.x - point_.x) +
                 (point_.y-point_.y) *(point_.y-point_.y));

    return distance;
}
