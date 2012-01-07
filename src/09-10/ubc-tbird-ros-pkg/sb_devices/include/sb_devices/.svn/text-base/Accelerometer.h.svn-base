#ifndef SB_DEVICES_ACCELEROMETER_H
#define SB_DEVICES_ACCELEROMETER_H

#include <string>

namespace sb_devices
{
  class Accelerometer
  {
    private:
      double zero;
      double slope;
    public:
      Accelerometer();
      void loadConfig(std::string filename);
      double getAccel( double voltage );
  };
} // end namespace sb_devices

#endif

