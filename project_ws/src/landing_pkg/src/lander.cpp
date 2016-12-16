// TODO: Make main controller subscribe this topic and send landing message

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>
#include <landing_pkg/ErrorStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>


using namespace std;
using namespace std_msgs;
using namespace landing_pkg;
using namespace ardrone_autonomy;
using namespace geometry_msgs;

ErrorStamped cur_pose_error;
double cur_quota;
double zeta_land_vel = -0.5;


// Publisher nodes for landing and switching boards
ros::Publisher switch_board_pub;
ros::Publisher landing_pub;
ros::Publisher z_effort_pub;

bool enable_landing = false;

enum drone_status {
    stabilizing, descending, rising, landing
};

drone_status drone_status;

/* LANDING PARAMETERS */
double center_margin = 0.01; // Accepted distance from (0,0) in m
double speed_margin = 0.02; // Accepted position variation in m/s
double avg_center_margin = 0.02;
double avg_speed_margin = 0.02;
double landing_quote = 200.0; // Quote for landing signal
double recovering_quote = 650.0;
double recovering_speed = 0.2;
double initial_switching_quote = 300.0;
double switching_board_quote = initial_switching_quote;
double switching_margin = 50.0;

void cur_quota_received(const NavdataConstPtr &navdata);

void pose_error_received(const ErrorStampedConstPtr &error);

void landing_boolean_received(const std_msgs::EmptyConstPtr &e);

bool check_landing_conditions();

void switch_board() {
    std_msgs::String board;
    switch (drone_status) {
        case descending: {
            board.data = "inner";
            switching_board_quote = initial_switching_quote + switching_margin;
            } break;
        case rising: {
            board.data = "outer";
            switching_board_quote = initial_switching_quote - switching_margin;
            } break;
        ROS_ERROR("Switch board called in state: %d.\n", drone_status);
    }

    switch_board_pub.publish(board);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lander");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param("center_margin",center_margin,0.01);
    pnh.param("speed_margin",speed_margin,0.02);
    pnh.param("avg_center_margin",avg_center_margin,0.02);
    pnh.param("avg_speed_margin",avg_speed_margin,0.02);
    pnh.param("landing_quote",landing_quote,300.0);

    z_effort_pub = nh.advertise<Float64>("/z_effort",1);
    landing_pub = nh.advertise<Empty>("/ardrone/land",1);
    switch_board_pub = nh.advertise<String>("/switch_board",1);

    ros::Subscriber sub_reach_quota = nh.subscribe("/ardrone/navdata", 1, &cur_quota_received);
    ros::Subscriber sub_pose_error = nh.subscribe("/pose_error", 1, &pose_error_received);
    ros::Subscriber sub_landing_enable = nh.subscribe("/enable_land", 1, &landing_boolean_received);

    // TODO Antonio: Nel momento in cui scendiamo sotto una CERTA quota o saliamo sopra una CERTA quota
    // TODO dobbiamo notificare il nodo modificato ar_sys di switchare rappresentazione della board.
    // TODO A tal fine ho previsto il topic /switch_board in cui mandare questa info. Il topic si aspetta
    // TODO una String che PER ORA ha solo due valori possibili: "outer" e "inner", rispettivamente per la 4x4 esterna
    // TODO e 4x4 interna. Nel file di launch è già tutto configurato quindi è un operazione da fare subito.
    // TODO (anche perchè non ha più senso procedere con la vecchia board)

    /* Aggiunto Fra*/
    // switch_board_pub.publish("outer"); // TODO pubblicare al cambio di altezza

    ros::Rate r(50);
    Float64 z_effort;
    while (ros::ok()) {
        if (enable_landing) {
            switch (drone_status) {
                case stabilizing: {
                    ROS_INFO("Status: Stabilizing. Alt: %lf.",cur_quota);
                    if (check_landing_conditions())
                        drone_status = descending;
                }
                    break;
                case descending: {
                    ROS_INFO("Status: Descending. Alt: %lf.",cur_quota);
                    //if(cur_quota <= switching_board_quote)
                        //switch_board();
                    if (cur_quota <= landing_quote) {
                        drone_status = landing;
                        break;
                    }
                    if (check_landing_conditions()) {
                        z_effort.data = zeta_land_vel;
                        z_effort_pub.publish(z_effort);
                    } else {
                        if(cur_quota <= recovering_quote)
                            drone_status = rising;
                    }
                }   break;
                case landing: {
                    ROS_INFO("Status: Landing. Alt: %lf.",cur_quota);
                    z_effort.data = 0;
                    zeta_land_vel = 0;
                    z_effort_pub.publish(z_effort);
                    landing_pub.publish(Empty());
                }
                    break;
                case rising: {
                    ROS_INFO("Status: Rising. Alt: %lf.",cur_quota);
                    //if(cur_quota > switching_board_quote)
                        //switch_board();
                    if(cur_quota >= recovering_quote) {
                        z_effort.data = 0;
                        drone_status = stabilizing;
                    }
                    else {
                        z_effort.data = recovering_speed;
                    }
                    z_effort_pub.publish(z_effort);
                }
                    break;
            }
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}

void cur_quota_received(const NavdataConstPtr &navdata) {
    cur_quota = navdata->altd;
}

void pose_error_received(const ErrorStampedConstPtr &error) {
    cur_pose_error = *error;
    /*ROS_INFO(
            "lander: Received pose_error (ex=%lf, ey=%lf, dx=%lf, dy=%lf, avg_ex=%lf, avg_ey=%lf, avg_dx=%lf, avg_dy=%lf)",
            cur_pose_error.ex, cur_pose_error.ey, cur_pose_error.dx, cur_pose_error.dy, cur_pose_error.avg_ex,
            cur_pose_error.avg_ey,
            cur_pose_error.avg_dx, cur_pose_error.avg_dy);*/
}

void landing_boolean_received(const std_msgs::EmptyConstPtr &e) {
    enable_landing = !enable_landing;
}



bool check_landing_conditions() {
    double k = 1;
    if(descending == drone_status)
        k = 1.8;
    return ((cur_pose_error.ex < (k * center_margin)) && (cur_pose_error.ey < (k * center_margin)) &&
            (cur_pose_error.dx < (k * speed_margin)) && (cur_pose_error.dy < (k * speed_margin)) &&
            (cur_pose_error.avg_ex < (k * avg_center_margin)) && (cur_pose_error.avg_ey < (k * avg_center_margin)) &&
            (cur_pose_error.avg_dx < (k * avg_speed_margin)) && (cur_pose_error.avg_dy < (k * avg_speed_margin)));
}