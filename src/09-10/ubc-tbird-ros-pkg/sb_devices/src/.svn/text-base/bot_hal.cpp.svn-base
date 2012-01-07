/**
 * This node acts as a "hardware abstraction layer" that
 * translates RobotState messages into useful high-level messages
 * and vice-versa.
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
#include <sb_rangefinders/Rangefinder.h>

using namespace sb_devices;
using sb_rangefinders::Rangefinder;
using sb_msgs::ServoCommand;

/* ROS-RELATED CONSTANTS */
/** The name of this ROS node */
const std::string NODE_NAME = "bot_hal";
/** The topic to listen for Furious state updates (eg, odometer) */
const std::string STATE_TOPIC = "robot_state";
/** The topic to listen for speed commands (eg, 1.0m/s) */
const std::string SPEED_TOPIC = "cmd_vel";
/** The topic to publish servo commands */
const std::string SERVO_TOPIC = "servo_command";
/** The topic to publish laser scan data */
const std::string LASER_TOPIC = "base_scan";
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
SpeedController spd_ctrl;
Rangefinder ir_array;
ros::Publisher servo_pub;
ros::Publisher laser_pub;
ServoCommand throttle_cmd;
ServoCommand steering_cmd;


/**
 * This function gets called when the robot wants to change the
 * target speed or direction.
 */
void onVelocityUpdate(const geometry_msgs::TwistConstPtr& new_msg)
{
  /* Set the target speed for the speed controller. */
  ROS_INFO("New target speed is %.2f", new_msg->linear.x);
  spd_ctrl.setTarget(new_msg->linear.x);

  /* Update the steering value, if it has changed.  Remember that, as
   * described above, this node intentionally "misinterprets" the
   * angular.z value as a servo position for simplicity. */
  int new_steer_value = steering_ctl.getRawValue(new_msg->angular.z);
  if ( steering_cmd.pwm != new_steer_value ) {
    steering_cmd.pwm = new_steer_value;
    servo_pub.publish(steering_cmd);
    ROS_INFO("New steering value is %.2f", new_msg->angular.z);
  }
}


/**
 * This function gets called when the microcontroller publishes new
 * state information.
 */
void onStateUpdate(const sb_msgs::RobotStateConstPtr& new_state)
{
  /* update throttle settings */
  odometer.onNewReading(new_state->odometer);
  double time = ros::Time::now().toSec();
  double distance = odometer.getDistance();
  double throttle = spd_ctrl.onNewReading(distance, time);
  if (throttle < 0.0) {
    throttle = sb_util::clamp(throttle, -MAX_THROTTLE, -MIN_THROTTLE);
  }
  else if (throttle > 0.0) {
    throttle = sb_util::clamp(throttle, MIN_THROTTLE, MAX_THROTTLE);
  }
  throttle_cmd.pwm = throttle_ctl.getRawValue(throttle);
  servo_pub.publish(throttle_cmd);
  ROS_INFO("%5.2f, %.2f, %.2f, %i", 
      time, distance, throttle, throttle_cmd.pwm);

  /* broadcast new LaserScan message */
  double ranges[ ir_array.getNumRays() ];  //this array stores distances
  ir_array.getDistances(new_state->analog, ranges);
  sensor_msgs::LaserScan laser_msg = ir_array.LaserScanMsgGenerator(ranges);
  laser_pub.publish(laser_msg);
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
  spd_ctrl.loadConfig(cfgfile);
  ir_array.loadConfig(cfgfile);

  /* Initialize the ServoCommand messages. */
  throttle_cmd.id = throttle_ctl.getId();
  throttle_cmd.pwm = 0;
  steering_cmd.id = steering_ctl.getId();
  steering_cmd.pwm = 0;

  /* Create publisher and subscribers */
  servo_pub = node.advertise<sb_msgs::ServoCommand>(
      SERVO_TOPIC, MSG_QUEUE_SIZE
  );
  laser_pub = node.advertise<sensor_msgs::LaserScan>(
      LASER_TOPIC, MSG_QUEUE_SIZE
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
