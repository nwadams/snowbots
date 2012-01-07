/**
 * This node acts as a "hardware abstraction layer" that
 * translates RobotState messages into useful high-level messages
 * and vice-versa.
 *
 * @author Ian Phillips <ianfp@freeshell.org>
 */

#include <iostream> 
#include <stdio.h>

#include <boost/program_options.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <sb_config/config_file.h>
#include <sb_util/range.h>
#include <sb_msgs/RobotState.h>
#include <sb_msgs/CarCommand.h>
#include <sb_msgs/ServoCommand.h>

#include <sb_devices/ServoController.h>
#include <sb_devices/SpeedController.h>
#include <blizzard/Odometer.h>

using namespace blizzard;
using sb_devices::ServoController;
using sb_devices::SpeedController;
using sb_msgs::CarCommand;

/* ROS-RELATED CONSTANTS */
/** The name of this ROS node */
const std::string NODE_NAME = "bot_hal";
/** The topic to listen for Furious state updates (eg, odometer) */
const std::string STATE_SUB_TOPIC = "robot_state";
/** The topic to listen for speed commands (eg, 1.0m/s) */
const std::string SPEED_SUB_TOPIC = "car_command";
/** The topic to publish servo commands */
const std::string CAR_PUB_TOPIC = "cmd_vel";
/** The topic to publish laser scan data */
/*const std::string LASER_PUB_TOPIC = "base_scan";*/
/** Number of messages to keep in the queue before discarding old ones. */
const int MSG_QUEUE_SIZE = 20;


/** The throttle is limited for safety purposes.  These constants should
 * be tuned to ensure reasonable limits for your vehicle. */
const double MAX_THROTTLE = 0.25;
const double MIN_THROTTLE = 0.05;


/* GLOBAL VARIABLES */
ServoController throttle_ctl("throttle");
ServoController steering_ctl("steering");
Odometer odometer;
SpeedController spd_ctrl;
ros::Publisher car_pub;
CarCommand nav_cmd;

double throttle;
double steering;


/**
 * This function gets called when the robot wants to change the
 * target speed or direction.
 *
void onVelocityUpdate(const geometry_msgs::TwistConstPtr& new_msg)
{
  /* Set the target speed for the speed controller. 
  ROS_INFO("New target speed is %.2f", new_msg->linear.x);
  spd_ctrl.setTarget(new_msg->linear.x);

  /* Update the steering value, if it has changed.  Remember that, as
   * described above, this node intentionally "misinterprets" the
   * angular.z value as a servo position for simplicity. 
  int new_steer_value = steering_ctl.getRawValue(new_msg->angular.z);
  if ( steering_cmd.pwm != new_steer_value ) {
    steering_cmd.pwm = new_steer_value;
    servo_pub.publish(steering_cmd);
    ROS_INFO("New steering value is %.2f", new_msg->angular.z);
  }
}*/

void onVelocityUpdate(const sb_msgs::CarCommandConstPtr& new_msg)
{
  /* Set the target speed for the speed controller. */
  //TODO ROS_INFO("New target speed is %.2f", new_msg->throttle);
  spd_ctrl.setTarget(new_msg->throttle);
  throttle = new_msg->throttle;
  steering = new_msg->steering;

  /* Update the steering value, if it has changed.  Remember that, as
   * described above, this node intentionally "misinterprets" the
   * angular.z value as a servo position for simplicity. */
  if ( nav_cmd.steering != steering ) {
    nav_cmd.steering = steering;
    car_pub.publish(nav_cmd);
  //TODO ROS_INFO("New steering value is %.2f", new_msg->steering);
  }
}


/**
 * This function gets called when the microcontroller publishes new
 * state information.
 */
void onStateUpdate(const sb_msgs::RobotStateConstPtr& new_state)
{
  /* update throttle settings */
  std::cout << "Odometer: " << new_state->odometer << std::endl;
  std::cout << "throttle: " << throttle << std::endl;
  odometer.onNewReading(new_state->odometer, throttle);
  double time = ros::Time::now().toSec();
  double distance = odometer.getDistance();

  /*
  printf("%f, %f, %f, %f\n", distance, spd_ctrl.getLastDistance(),
                                    time, spd_ctrl.getLastTime());
  double speed = odometer.calcSpeed(distance, spd_ctrl.getLastDistance(),
                                    time, spd_ctrl.getLastTime());

  printf("Speed: %f\n", speed);
  */


  double throttle = spd_ctrl.onNewReading(distance, time);
  if (throttle < 0.0) {
    throttle = sb_util::clamp(throttle, -MAX_THROTTLE, -MIN_THROTTLE);
  }
  else if (throttle > 0.0) {
    throttle = sb_util::clamp(throttle, MIN_THROTTLE, MAX_THROTTLE);
  }
  

  nav_cmd.throttle = throttle;
  nav_cmd.steering = steering;
  car_pub.publish(nav_cmd);
  ROS_INFO("%5.2f, %.2f, %.2f, %.2f", 
      time, distance, throttle, steering);
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

  /* Initialize the ServoCommand messages. */
  nav_cmd.steering = 0;
  nav_cmd.throttle = 0;

  /* Create publisher and subscribers */
  car_pub = node.advertise<sb_msgs::CarCommand>(
      CAR_PUB_TOPIC, MSG_QUEUE_SIZE
  );

  ros::Subscriber spd_sub = node.subscribe(
      SPEED_SUB_TOPIC, MSG_QUEUE_SIZE, onVelocityUpdate
  );
  ros::Subscriber state_sub = node.subscribe(
      STATE_SUB_TOPIC, MSG_QUEUE_SIZE, onStateUpdate
  );

  /* Run the main loop */
  ROS_INFO("time, distance, throttle, servo");
  while (ros::ok()) {
    ros::spin();
  }
  ROS_INFO("Shutting down %s", NODE_NAME.c_str());
}
