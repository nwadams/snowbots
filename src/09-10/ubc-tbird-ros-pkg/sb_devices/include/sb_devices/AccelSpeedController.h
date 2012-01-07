#ifndef SB_DEVICES_SPEED_CONTROLLER_H
#define SB_DEVICES_SPEED_CONTROLLER_H

#include <ostream>
#include <sb_devices/ServoController.h>
#include <string>

namespace sb_devices
{
  /**
   * This does the PID calculation to determine what the throttle setting
   * should be.
   */
  class AccelSpeedController
  {
  private:
    /* TUNABLE CONSTANTS */
    /** PID control constants */
    double kp, ki, kd;

    /* VARIABLES */
    double target_speed;
    double last_accel, last_speed, last_error, last_time;
    double total_error;
    double last_throttle;

  public:
    /**
     * Constructor.
     */
    AccelSpeedController();

    void loadConfig(std::string filename);

    /**
     * Updates the target speed (ie, the speed we want the robot to go).
     */
    void setTarget(double new_target);

    /**
     * 
     */
    double getSpeed(double accel, double time);
    
    double getThrottle(double speed);
  };

} // end namespace sb_devices

# endif


