#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>

using namespace sensor_msgs;
using namespace message_filters;

// void callback(const ImageConstPtr& image1, const ImageConstPtr& image2)
// {
//   // Solve all of perception here...
// }

void effortReceived(const std_msgs::Float64ConstPtr &msg) {
	ROS_INFO("Received %f!", msg->data);	
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vel_controller");

  ros::NodeHandle nh;

  ros::Subscriber sub_x = nh.subscribe("/x_effort", 1000, &effortReceived);
	ros::Subscriber sub_y = nh.subscribe("/y_effort", 1000, &effortReceived);
	ros::Subscriber sub_z = nh.subscribe("/z_effort", 1000, &effortReceived);

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
