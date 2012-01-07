#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <furious/FuriousState.h>
#include <furious/ServoCommand.h>

/* ROS-RELATED CONSTANTS */
/** The name of this ROS node */
const char* NODE_NAME = "speedControl";
/** The topic to listen for Furious state updates (eg, odometer) */
const char* STATE_TOPIC = "furiousState";
/** The topic to listen for speed commands (eg, 1.0m/s) */
const char* SPEED_TOPIC = "driveSpeed";
/** The topic to publish servo commands (eg, 80%) */
const char* CMD_TOPIC = "servoCommand";
/** Number of messages to keep in the queue before discarding old ones. */
const int MSG_QUEUE_SIZE = 20;


/* TUNABLE CONSTANTS */
/** PID control values */
const double KP = 25.0;
const double KI = 0.0;
const double KD = 0.0;
/** Maximum throttle we're willing to send, to prevent the robot
 * from taking off like a bat out of hell. */
const int MAX_THROTTLE = 50;
/** Minimum throttle we're willing to send; servo values below this
 * are insufficient to move the vehicle. */
const int MIN_THROTTLE = 25;
/** Number of seconds to "look ahead" when predicting what our speed
 * will be. */
const double PREDICT_SECS = 0.3;
/** How long do we allow between odometer updates.  If we wait longer 
 * than this, assume we're not moving. */
const double TIMEOUT = 1.0;

int clamp(int val, int min, int max)
{
  if      (val > max) return max;
  else if (val < min) return min;
  return val;
}

/**
 * This does the PID calculation to determine what the throttle setting
 * should be.
 */
class SpeedController
{
private:
  /** PID control constants */
  double kp, ki, kd;
  double target_speed;
  double last_dist, last_speed, last_error, last_time;
  double total_error;
  int last_throttle;

public:
  /**
   * Constructor.  Takes the PID control constants as parameters.
   */
  SpeedController(double new_kp, double new_ki, double new_kd) :
    kp(new_kp), ki(new_ki), kd(new_kd),
    target_speed(0.0), last_dist(0.0), last_speed(0.0), last_error(0.0),
    last_time(0.0), total_error(0.0), last_throttle(0)
  { /* empty */ }

  /**
   * Updates the target speed (ie, the speed we want the robot to go).
   */
  void setTarget(double new_target)
  {
    ROS_INFO("New target speed is %.2f", new_target);
    target_speed = new_target;
  }

  /**
   * Called when a new odometer reading comes in.  Returns the 
   * recommended throttle setting.
   */
  int onNewReading(double new_dist)
  {
    if (last_time == 0.0) {
      ROS_INFO("Initializing clock");
      last_time = ros::Time::now().toSec();
      return 0;
    }
    if (target_speed == 0.0) {
      return 0;
    }
    
    double new_time = ros::Time::now().toSec();
    double delta_time = new_time - last_time;
    
    if (new_dist == last_dist && delta_time < TIMEOUT) {
//      ROS_INFO("No odometer update.");
      return last_throttle;
    }

    double speed = (new_dist - last_dist) / delta_time;
    double accel = (speed - last_speed) / delta_time;
    // Use acceleration to predict how fast we'll be going
    double predicted = speed + (PREDICT_SECS * accel);
    double error = target_speed - predicted;
    double delta_error = error - last_error;
    total_error += error;

    // Use PID to modify throttle setting.
    int throttle = last_throttle 
        + int((KP * error) 
        + (KI * total_error) 
        + (KD * delta_error)
    );
    throttle = clamp(throttle, MIN_THROTTLE, MAX_THROTTLE);
    ROS_INFO(
        "dTime: %.2f, speed: %.2f, accel: %.2f, predicted: %.2f, target: %.2f, throttle: %i",
        delta_time, speed, accel, predicted, target_speed, throttle
    );

    // Save away current values for next time.
    last_dist = new_dist;
    last_speed = speed;
    last_error = error;
    last_throttle = throttle;
    last_time = new_time;
    return throttle;
  }
};


/* GLOBAL VARIABLES */
SpeedController controller(KP, KI, KD);
ros::Publisher cmd_pub;


/**
 * This function gets called when the robot wants to change the
 * target speed.
 */
void onSpeedUpdate(const std_msgs::Float64ConstPtr& new_speed)
{
  controller.setTarget(new_speed->data);
}

typedef boost::shared_ptr<furious::FuriousState const> 
  FuriousStateConstPtr;

/**
 * This function gets called when the Furious board publishes new
 * state information.
 */
void onStateUpdate(const FuriousStateConstPtr& new_state)
{
  int throttle = controller.onNewReading(new_state->odometer);
  furious::ServoCommand cmd;
  cmd.servo = "throttle";
  cmd.value = throttle;
  cmd_pub.publish(cmd);
}


int main(int argc, char** argv)
{
  /* Initialize node */
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle node;

  /* Create publisher and subscribers */
  cmd_pub = node.advertise<furious::ServoCommand>(
      CMD_TOPIC, MSG_QUEUE_SIZE
  );
  ros::Subscriber spd_sub = node.subscribe(
      SPEED_TOPIC, MSG_QUEUE_SIZE, onSpeedUpdate
  );
  ros::Subscriber state_sub = node.subscribe(
      STATE_TOPIC, MSG_QUEUE_SIZE, onStateUpdate
  );

  /* Run the main loop */
  while (ros::ok()) {
    ros::spin();
  }
}
