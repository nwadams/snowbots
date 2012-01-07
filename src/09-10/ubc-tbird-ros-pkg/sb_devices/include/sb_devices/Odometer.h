#ifndef SB_DEVICES_ODOMETER_H
#define SB_DEVICES_ODOMETER_H

#include <string>

namespace sb_devices
{
  class Odometer
  {
    private:
      int total_ticks;
      double meters_per_tick;
    public:
      Odometer();
      void loadConfig(std::string filename);
      void onNewReading(int ticks);
      double getDistance();
  };
} // end namespace sb_devices

#endif

