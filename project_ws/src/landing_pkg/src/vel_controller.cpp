#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"

using namespace geometry_msgs;
using namespace std_msgs;
// using namespace message_filters;

ros::Publisher chatter_pub;

void xEffortReceived(const Float64ConstPtr &msg) {

	// ROS_INFO("Received %f!", msg->data);

  Twist twist;
	// ROS_INFO("Creato Twist!");

  twist.linear.x = msg->data;
	// ROS_INFO("Inserito x!");

	// ROS_INFO("Inviato Twist su cmd_vel!");
  chatter_pub.publish(twist);
}

void yEffortReceived(const Float64ConstPtr &msg) {

  Twist twist;
  twist.linear.y = msg->data;
  chatter_pub.publish(twist);

}

void zEffortReceived(const Float64ConstPtr &msg) {

  Twist twist;
  twist.linear.z = msg->data;
  chatter_pub.publish(twist);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vel_controller");

  ros::NodeHandle nh;

  ros::Subscriber s1 = nh.subscribe("/x_effort", 1000, &xEffortReceived);
  ros::Subscriber s2 = nh.subscribe("/y_effort", 1000, &yEffortReceived);
  ros::Subscriber s3 = nh.subscribe("/z_effort", 1000, &zEffortReceived);
  
  chatter_pub = nh.advertise<Twist>("/cmd_vel", 1000);

  // A questo punto dovremmo sottoscrivere vel_controller ai topic x_effort, y_effort e z_effort
  // in modo da generare il Twist opportuno sul topic cmd_vel.

  // Tuttavia il messaggio originario PoseStamped generato dal nodo ar_single_board
  // è stato suddiviso in tre messaggi di tipi Float64 (e quindi non Stamped - cioè senza Header)
  // attraverso i 3 nodi pose_to_x, pose_to_y, pose_to_z (con topic_tools transform come consigliato a ricevimento).

  // Avendo perso l'informazione circa il timestamp originario non possiamo quindi adoperare i meccanismi di 
  // sincronizzazione di messaggi descritti a http://wiki.ros.org/message_filters
  // Infatti non possiamo impiegare nè una ExactTime Policy nè una ApproximateTime Policy.

  // C'è qualche altro modo per procedere o dobbiamo creare dei nodi e messaggi custom?

  // ApproximateTime Policy
  // but how to synchronize std_msgs/Float64 messages?

  // message_filters::Subscriber<Image> image1_sub(nh, "image1", 1);
  // message_filters::Subscriber<Image> image2_sub(nh, "image2", 1);

  // typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
  // // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  // Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image1_sub, image2_sub);
  // sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
