#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <pid/StampedFloat64.h>
#include <geometry_msgs/Twist.h>

using namespace geometry_msgs;
using namespace pid;
using namespace message_filters;

ros::Publisher chatter_pub;

void effortReceived(const StampedFloat64ConstPtr& xEffort, const StampedFloat64ConstPtr& yEffort) {
Twist tw;
tw.linear.x = xEffort->c;
tw.linear.y = yEffort->c;
ROS_INFO("This is x: %f", tw.linear.x);
ROS_INFO("This is y: %f", tw.linear.y);
//chatter_pub.publish(tw);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vel_controller");

  ros::NodeHandle nh;
  
  chatter_pub = nh.advertise<Twist>("cmd_vel", 1);
  
  message_filters::Subscriber<StampedFloat64> x_effort_sub(nh, "x_effort", 1);
  message_filters::Subscriber<StampedFloat64> y_effort_sub(nh, "y_effort", 1);

  typedef sync_policies::ExactTime<StampedFloat64, StampedFloat64> MySyncPolicy;
  // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), x_effort_sub, y_effort_sub);
  sync.registerCallback(&effortReceived);

  ros::spin();

  return 0;
}
