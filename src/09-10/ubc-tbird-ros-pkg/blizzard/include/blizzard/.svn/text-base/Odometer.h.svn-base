#ifndef BLIZZARD_ODOMETER_H
#define BLIZZARD_ODOMETER_H

#include <string>

namespace blizzard
{
  class Odometer
  {
    private:
      int total_ticks;
      double current_throttle;
      double meters_per_tick;
    public:
      Odometer();
      void loadConfig(std::string filename);
      void onNewReading(int new_ticks, double new_throttle);
      double calcSpeed(double dist1, double dist2, double time1, double time2);
      double getDistance();
  };
} // end namespace blizzard

#endif

