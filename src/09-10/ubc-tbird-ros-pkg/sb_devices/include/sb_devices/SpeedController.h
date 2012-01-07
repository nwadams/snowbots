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
  class SpeedController
  {
  private:
    /* TUNABLE CONSTANTS */
    /** PID control constants */
    double kp, ki, kd;

    /** Number of seconds to "look ahead" when predicting what our speed
     * will be. */
    double predict_secs;

    double meters_per_tick;

    /** How long do we allow between odometer updates.  If we wait longer 
     * than this, assume we're not moving. */
    double timeout;

    /* VARIABLES */
    double target_speed;
    double last_dist, last_speed, last_error, last_time;
    double total_error;
    double last_throttle;

  public:
    /**
     * Constructor.
     */
    SpeedController();

    void loadConfig(std::string filename);

    /**
     * Updates the target speed (ie, the speed we want the robot to go).
     */
    void setTarget(double new_target);

    /**
     * Called when a new odometer reading comes in.  Returns the 
     * recommended throttle setting.
     */
    double onNewReading(double distance, double time);

    double getLastDistance();

    double getLastTime();


  };

} // end namespace sb_devices

# endif


