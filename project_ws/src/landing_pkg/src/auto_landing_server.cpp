#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <landing_pkg/AutoLandingAction.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

class AutoLandingAction {
public:

    AutoLandingAction(std::string name) :
            as_(nh_, name, false),
            action_name_(name) {
        //register the goal and feeback callbacks
        as_.registerGoalCallback(boost::bind(&AutoLandingAction::goalCB, this));
        as_.registerPreemptCallback(boost::bind(&AutoLandingAction::preemptCB, this));

        //subscribe to the data topic of interest
        sub_ = nh_.subscribe("/drone_landed", 1, &AutoLandingAction::drone_landedCB, this);
        as_.start();
    }

    ~AutoLandingAction(void) {
    }

    void goalCB() {
        // accept the new goal
        toggle_ = as_.acceptNewGoal()->toggle;
        if (toggle_) {
            system("roslaunch landing_pkg bootstrapper.launch");
        }
    }

    void preemptCB() {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
    }

    void drone_landedCB(const std_msgs::Bool::ConstPtr &landed) {
        // make sure that the action hasn't been canceled
        if (!as_.isActive())
            return;

        if (landed) {
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            as_.setSucceeded(result_);
        } else {
            ROS_INFO("%s: Aborted", action_name_.c_str());
            //set the action state to aborted
            as_.setAborted(result_);
        }
    }

protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<landing_pkg::AutoLandingAction> as_;
    std::string action_name_;
    /* bool indicating wether the action server is activated */
    bool toggle_;
    landing_pkg::AutoLandingFeedback feedback_;
    landing_pkg::AutoLandingResult result_;
    ros::Subscriber sub_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "AutoLanding");
    AutoLandingAction AutoLanding(ros::this_node::getName());
    ros::spin();

    return 0;
}
