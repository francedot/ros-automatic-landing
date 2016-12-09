 #include <ros/ros.h>
#include <ros/duration.h>
#include <std_msgs/Float64.h>
#include <pid/StampedFloat64.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <iostream>
#include <sys/time.h>
#include <unistd.h>

using namespace tf;
using namespace pid;
using namespace std_msgs;
using namespace geometry_msgs;
using namespace message_filters;

ros::Publisher chatter_pub;
boost::shared_ptr<tf::TransformListener> listener;
tf::StampedTransform transform;
bool transform_setup = false;
bool enabled = false;

Twist twist;
ros::Time last_cmd_vel_time;

void zEffortReceived(const Float64ConstPtr& zEffort) {
    twist.linear.z = zEffort->data;
}

void effortReceived(const StampedFloat64ConstPtr& xEffort, const StampedFloat64ConstPtr& yEffort/*, const StampedFloat64ConstPtr& yawEffort*/) {
  if(!enabled)
    return;
  if(!transform_setup) {
    try{
      listener->lookupTransform("ardrone_base_link","ardrone_base_bottomcam",ros::Time(0),transform);
      transform_setup = true;
    }
    catch(tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      return;
    }
  }
  tf::Vector3 linear(xEffort->c, yEffort->c, 0);
  linear = (transform * linear) - (transform * tf::Vector3(0,0,0));
  twist.linear.x = linear.x();
  twist.linear.y = linear.y();
  last_cmd_vel_time = ros::Time::now();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "vel_controller");
  ros::NodeHandle nh;

  ros::NodeHandle pnh("~");
  pnh.param("enabled",enabled,false);

  listener.reset(new tf::TransformListener);
  chatter_pub = nh.advertise<Twist>("cmd_vel", 1);

  ros::Subscriber sub = nh.subscribe("ardrone/z_effort", 1000, &zEffortReceived);

  message_filters::Subscriber<StampedFloat64> x_effort_sub(nh, "x_effort", 1);
  message_filters::Subscriber<StampedFloat64> y_effort_sub(nh, "y_effort", 1);
  typedef sync_policies::ExactTime<StampedFloat64, StampedFloat64> MySyncPolicy;
  // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), x_effort_sub, y_effort_sub);
  sync.registerCallback(&effortReceived);

  ros::Duration timeout;
  timeout = ros::Duration(1,0);

  ros::Rate r(50);
  while(ros::ok()) {
      if(ros::Time::now() - last_cmd_vel_time > timeout) {
          twist.linear.x = 0;
          twist.linear.y = 0;
          twist.linear.z = 0;
          twist.angular.x = 0;
          twist.angular.y = 0;
          twist.angular.z = 0;
      }
      chatter_pub.publish(twist);
      ros::spinOnce();
      r.sleep();
  }
  return 0;
}

