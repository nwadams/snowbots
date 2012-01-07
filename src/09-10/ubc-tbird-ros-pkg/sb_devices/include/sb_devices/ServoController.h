# ifndef SB_DEVICES_SERVO_CONTROLLER_H
# define SB_DEVICES_SERVO_CONTROLLER_H

#include <string>

namespace sb_devices
{
  class ServoController
  {
  private:
    std::string name;
    int id, min, center, max;
  public:
    /** Constructor
     * @param servo_name The name of this servo, which it will use to
     *   read in configuration values. */
    ServoController(std::string servo_name);
    
    /** Reads in the configuration options in the given file.  It reads
     * the section [servo.<servo_name>], where <servo_name> is the name
     * of this servo, as given to the constructor.
     * @param filename The name of the .ini-style config file. */
    void loadConfig(std::string filename);

    /** @return The id of this servo. */
    int getId();

    /** @param position The desired servo position, between -1.0 and 1.0.
     * @return The raw value that will put the servo in the desired 
     * position. */
    int getRawValue(double position);
  };

}  // end namespace sb_devices

# endif


