#include <ros/ros.h>
#include <std_msgs/Float64.h>


/* ROS-related Constants */
const std::string NODE_NAME        = "stop_sign_commander";
const std::string SUBSCRIBE_TOPIC    = "stop_sign_state";
const int MSG_QUEUE_SIZE = 20;	// # of possible messeges in buffer before deleting the old ones
const int NAP_TIME = 500000;

void callBack(const std_msgs::Float64ConstPtr& float64Msg);

int main(int argc, char *argv[])
{
   ROS_INFO("Starting %s ...", NODE_NAME.c_str());	
   ros::init(argc, argv, NODE_NAME);
   ros::NodeHandle n;
   ros::Subscriber dataSub = n.subscribe(SUBSCRIBE_TOPIC, MSG_QUEUE_SIZE, callBack);
   //ros::Publisher dataPub = n.advertise<sb_msgs::VisionInfo>(PUBLISH_TOPIC, MSG_QUEUE_SIZE);

   while (ros::ok())
     ros::spin();

   ROS_INFO("Shutting down %s!", NODE_NAME.c_str());
   return 0;
}

void callBack(const std_msgs::Float64ConstPtr& float64Msg)
{
  ROS_INFO("I received something...\n");
}
