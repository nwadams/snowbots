#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include "ros/file_log.h"
#include "roslib/Log.h"


using namespace ros;

/* Global Variables */
Publisher dataPub;

/* ROS-Related Constants */
const std::string NODE_NAME        = "rosout_tester";
const std::string SUBSCRIBE_TOPIC  = "/rosout";
const int MSG_QUEUE_SIZE = 20;

void rosoutCallBack(const roslib::LogConstPtr& logmsg);

int main(int argc, char** argv)
{
  std::string filename;

  if(argc < 1)
    filename = argv[1];

  /* Starting and initializing this node */
  ROS_INFO("Starting %s ...", NODE_NAME.c_str());	
  init(argc, argv, NODE_NAME);	
  NodeHandle n;

  /* Defining subscriber and publisher */
  Subscriber dataSub = n.subscribe(SUBSCRIBE_TOPIC, MSG_QUEUE_SIZE, rosoutCallBack);
//  dataPub = n.advertise<geometry_msgs::Twist>(PUBLISH_TOPIC, MSG_QUEUE_SIZE);
	
  /* Main loop */
  while (ok())
	spin();

  /* Shuttiing down this node */
  ROS_INFO("Shutting down %s!", NODE_NAME.c_str());
}

/* this method is called whenever a new messegae is arrived */
void rosoutCallBack(const roslib::LogConstPtr& logmsg)
{
  std::cout << "-----------------------------" << std::endl;
  std::cout << " ROS Log--------------------- " << std::endl;
  std::cout << " msg: "  << logmsg->msg << std::endl;
  std::cout << " node: " << logmsg->name << std::endl;
}
