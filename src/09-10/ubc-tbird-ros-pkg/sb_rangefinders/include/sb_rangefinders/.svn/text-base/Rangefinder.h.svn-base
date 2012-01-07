#ifndef SB_RANGEFINDERS_RANGEFINDER_H
#define SB_RANGEFINDERS_RANGEFINDER_H

#include <ostream>
// #include <sb_rangefinders/Rangefinder.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <string>
#include <vector>

namespace sb_rangefinders
{

  class Rangefinder
  {
    private:
    /* Fields */
    int id_start;
    int id_increment;
    int num;
    double center;
    double angle_increment;
    std::string type;

    public:
    /* Constructor */
    Rangefinder();

    /* Methods */
    int getNumRays();
    double getMinRange();
    double getMaxRange();
    void loadConfig(std::string filename);
    void getDistances(const std::vector<double> &analog, double* ranges);
    sensor_msgs::LaserScan LaserScanMsgGenerator(double* ranges);
    geometry_msgs::Twist vectorCalculator(double throttle, double steering);
    double sharpenSteering(double steering);
  };

} // end namespace sb_rangefinders

# endif


