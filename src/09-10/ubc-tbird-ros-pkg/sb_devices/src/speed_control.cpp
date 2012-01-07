/**
 * This node controls both the "throttle" and "steering" servos (which
 * must be defined in your config file).  It takes a geometry_msgs/Twist
 * message on the "cmd_vel" topic and converts the linear.x into a
 * throttle servo position and the angular.z into a steering servo 
 * position.
 *
 * The linear.x is interpreted as the desired speed in meters per second;
 * this node does predictive PID control to try and maintain that speed.
 *
 * The angular.z is interpreted as a servo position between -1.0 (full 
 * right) and 1.0 (full left), and is simply translated into the 
 * corresponding raw servo value.  A fancier version of this node would
 * actually interpret the angular.z properly as angular momentum and
 * do the appropriate calculations, but that is for another node and
 * another day.
 *
 * @author Ian Phillips <ianfp@freeshell.org>
 */

#include <iostream> 

#include <boost/program_options.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <sb_config/config_file.h>
#include <sb_util/range.h>
#include <sb_msgs/RobotState.h>
#include <sb_msgs/ServoCommand.h>

#include <sb_devices/ServoController.h>
#include <sb_devices/Odometer.h>
#include <sb_devices/SpeedController.h>

using namespace sb_devices;

/* ROS-RELATED CONSTANTS */
/** The name of this ROS node */
const std::string NODE_NAME = "speed_control";
/** The topic to listen for Furious state updates (eg, odometer) */
const std::string STATE_TOPIC = "furious_state";
/** The topic to listen for speed commands (eg, 1.0m/s) */
const std::string SPEED_TOPIC = "cmd_vel";
/** The topic to publish servo commands */
const std::string CMD_TOPIC = "servo_command";
/** Number of messages to keep in the queue before discarding old ones. */
const int MSG_QUEUE_SIZE = 20;


/** The throttle is limited for safety purposes.  These constants should
 * be tuned to ensure reasonable limits for your vehicle. */
const double MAX_THROTTLE = 0.5;
const double MIN_THROTTLE = 0.25;


/* GLOBAL VARIABLES */
ServoController throttle_ctl("throttle");
ServoController steering_ctl("steering");
Odometer odometer;
SpeedController controller;
ros::Publisher cmd_pub;
sb_msgs::ServoCommand throttle_cmd;
sb_msgs::ServoCommand steering_cmd;


/**
 * This function gets called when the robot wants to change the
 * target speed or direction.
 */
void onVelocityUpdate(const geometry_msgs::TwistConstPtr& new_msg)
{
  /* Set the target speed for the PID controller. */
  ROS_INFO("New target speed is %.2f", new_msg->linear.x);
  controller.setTarget(new_msg->linear.x);

  /* Update the steering value, if it has changed.  Remember that, as
   * described above, this node intentionally "misinterprets" the
   * angular.z value as a servo position for simplicity. */
  int new_steer_value = steering_ctl.getRawValue(new_msg->angular.z);
  if ( steering_cmd.value != new_steer_value ) {
    steering_cmd.value = new_steer_value;
    cmd_pub.publish(steering_cmd);
    ROS_INFO("New steering value is %.2f", new_msg->angular.z);
  }
}


/**
 * This function gets called when the microcontroller publishes new
 * state information.
 */
void onStateUpdate(const sb_msgs::RobotStateConstPtr& new_state)
{
  odometer.onNewReading(new_state->odometer);
  double time = ros::Time::now().toSec();///replace with t.to_nsec  for nono seconds 
  double distance = odometer.getDistance();
  double throttle = controller.onNewReading(distance, time);
  if (throttle < 0.0) {
    throttle = sb_util::clamp(throttle, -MAX_THROTTLE, -MIN_THROTTLE);
  }
  else {
    throttle = sb_util::clamp(throttle, MIN_THROTTLE, MAX_THROTTLE);
  }
  throttle_cmd.value = throttle_ctl.getRawValue(throttle);
  cmd_pub.publish(throttle_cmd);
  ROS_INFO("%5.2f, %.2f, %.2f, %i", 
      time, distance, throttle, throttle_cmd.value);
}


int main(int argc, char** argv)
{
  /* Initialize node */
  ROS_INFO("Starting %s", NODE_NAME.c_str());
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle node;

  /* Load configuration file */
  std::string cfgfile;
  if (! sb_config::findConfigFile(argc, argv, cfgfile) ) {
    ROS_FATAL("Can't find configuration file.");
    ros::shutdown();
  }
  ROS_INFO("Loading configuration in %s", cfgfile.c_str());
  throttle_ctl.loadConfig(cfgfile);
  steering_ctl.loadConfig(cfgfile);
  odometer.loadConfig(cfgfile);
  controller.loadConfig(cfgfile);

  /* Initialize the ServoCommand messages. */
  throttle_cmd.id = throttle_ctl.getId();
  throttle_cmd.value = 0;
  steering_cmd.id = steering_ctl.getId();
  steering_cmd.value = 0;

  /* Create publisher and subscribers */
  cmd_pub = node.advertise<sb_msgs::ServoCommand>(
      CMD_TOPIC, MSG_QUEUE_SIZE
  );
  ros::Subscriber spd_sub = node.subscribe(
      SPEED_TOPIC, MSG_QUEUE_SIZE, onVelocityUpdate
  );
  ros::Subscriber state_sub = node.subscribe(
      STATE_TOPIC, MSG_QUEUE_SIZE, onStateUpdate
  );

  /* Run the main loop */
  ROS_INFO("time, distance, throttle, servo");
  while (ros::ok()) {
    ros::spin();
  }
  ROS_INFO("Shutting down %s", NODE_NAME.c_str());
}
