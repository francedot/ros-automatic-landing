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

using namespace tf;
using namespace pid;
using namespace geometry_msgs;
using namespace message_filters;

ros::Publisher chatter_pub;
boost::shared_ptr<tf::TransformListener> listener;
tf::StampedTransform transform;
bool transform_setup = false;

struct timeval last_callback, max_time_for_effort;
bool already_sent = false;

void set_max_time_for_effort() {
  // Sets max time before sending an empty twist to the drone
  // TO-DO: Figure out how long should be this interval
  //	    It could be K * AVG[Times_of_efforts_arrivals] with 0 < k < 1
  max_time_for_effort.tv_sec = 0;
  max_time_for_effort.tv_usec = 500000;
}

void send_stop_message() {
  // Sends an empty twist to the drone through the cmd_vel topic
  already_sent = true;
  Twist tw;
  tw.linear.x = 0;
  tw.linear.y = 0;
  tw.linear.z = 0;
  tw.angular.x = 0;
  tw.angular.y = 0;
  tw.angular.z = 0;
  chatter_pub.publish(tw);
}

void effortReceived(const StampedFloat64ConstPtr& xEffort, const StampedFloat64ConstPtr& yEffort/*, const StampedFloat64ConstPtr& yawEffort*/) {
  /*
      Called when an <x,y> control effort arrives.
      Transforms from camera to drone's coordinate system.
      Writes Twist(x,y,0) to cmd_vel.
  */
  gettimeofday(&last_callback, NULL);
  already_sent = false;
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
  // Mod
/*  Quaternion fromYaw = tf::createQuaternionFromYaw(yawEffort->c);
  fromYaw = transform * fromYaw;
  double toYaw = getYaw(fromYaw);
*/  //
  tf::Vector3 linear(xEffort->c, yEffort->c, 0);
  linear = (transform * linear) - (transform * tf::Vector3(0,0,0));

  Twist tw;
  tw.linear.x = linear.x();
  tw.linear.y = linear.y();
  //tw.angular.z = toYaw;
  chatter_pub.publish(tw);
}

int timeval_subtract (struct timeval *result, struct timeval *x, struct timeval *y) {
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

int main(int argc, char** argv) {
  ros::init(argc, argv, "vel_controller");
  ros::NodeHandle nh;
  
  listener.reset(new tf::TransformListener);
  chatter_pub = nh.advertise<Twist>("cmd_vel", 1);

  message_filters::Subscriber<StampedFloat64> x_effort_sub(nh, "x_effort", 1);
  message_filters::Subscriber<StampedFloat64> y_effort_sub(nh, "y_effort", 1);
  typedef sync_policies::ExactTime<StampedFloat64, StampedFloat64> MySyncPolicy;
  // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), x_effort_sub, y_effort_sub);
  sync.registerCallback(&effortReceived);

  struct timeval time_difference, current_time;
  set_max_time_for_effort();
  
  ros::Rate r(50);
  while(ros::ok()) {
    if(!already_sent){
      gettimeofday(&current_time, NULL);
      if(1 == timeval_subtract(&time_difference, &current_time, &last_callback))
	ROS_ERROR("time difference should not be negative"); 
      
      if((time_difference.tv_sec >= max_time_for_effort.tv_sec) && (time_difference.tv_usec > max_time_for_effort.tv_usec))
	send_stop_message();    
    }
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
