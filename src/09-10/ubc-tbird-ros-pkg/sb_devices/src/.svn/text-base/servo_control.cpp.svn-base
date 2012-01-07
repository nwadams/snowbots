#include <iostream> 

#include <boost/program_options.hpp>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <sb_config/config_file.h>
#include <sb_msgs/ServoCommand.h>
#include <sb_devices/ServoController.h>

using namespace sb_devices;

/* ROS-RELATED CONSTANTS */

/** The name of this ROS node.  This will be remapped by the shell script
 * src/servo_control to "servo_control_<servo_name>", where <servo_name>
 * is the name of the servo (as specified in the config file) that this 
 * node will control. */
const std::string NODE_NAME = "servo_control";

/** The topic on which to receive high-level servo commands.  This will also
 * be remapped by the servo_control shell script, as described above. */
const std::string SUB_TOPIC = "sub_topic";

/** The topic on which to publish low-level servo commands */
const std::string PUB_TOPIC = "servo_command";

/** Number of messages to keep in the queue before discarding old ones. */
const int MSG_QUEUE_SIZE = 20;


/* GLOBAL VARIABLES */
ServoController* servo_ptr;
ros::Publisher pub;


/**
 * This function gets called when the robot wants to change the
 * target speed.
 */
void onUpdate(const std_msgs::Float64ConstPtr& in_msg)
{
  double position = in_msg->data;
  sb_msgs::ServoCommand out_msg;
  out_msg.id = servo_ptr->getId();
  out_msg.value = servo_ptr->getRawValue(position);
  pub.publish(out_msg);
  ROS_INFO("%.2f => %i", position, out_msg.value);
}

int main(int argc, char** argv)
{
  /* Initialize node */
  ROS_INFO("Starting %s", NODE_NAME.c_str());
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle node;

  /* Get servo name and instantiate ServoController */
  if ( argc < 2 ) {
    ROS_FATAL("Missing required param servo_name");
    ros::shutdown();
  }
  servo_ptr = new ServoController(argv[1]);
  ROS_INFO("Controlling servo '%s'", argv[1]);

  /* Load configuration file */
  std::string cfgfile;
  if (! sb_config::findConfigFile(argc, argv, cfgfile) ) {
    ROS_FATAL("Can't find configuration file.");
    ros::shutdown();
  }
  ROS_INFO("Loading configuration in %s", cfgfile.c_str());
  servo_ptr->loadConfig(cfgfile);

  /* Create publisher and subscribers */
  pub = node.advertise<sb_msgs::ServoCommand>(
      PUB_TOPIC, MSG_QUEUE_SIZE
  );
  ros::Subscriber sub = node.subscribe(
      SUB_TOPIC, MSG_QUEUE_SIZE, onUpdate
  );

  /* Run the main loop */
  while (ros::ok()) {
    ros::spin();
  }
  ROS_INFO("Shutting down %s", NODE_NAME.c_str());
  delete servo_ptr;
}

