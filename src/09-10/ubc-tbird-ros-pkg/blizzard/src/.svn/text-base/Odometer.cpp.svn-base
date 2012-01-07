#include <stdio.h>
#include <stdlib.h>
#include <boost/program_options.hpp>
#include <fstream>
#include <blizzard/Odometer.h>

const int WHEELS_DIAMETER = 0.465;	//in centimeters
const int RESOLUTION      = 10;		//10 thicks per round

/* Global Variables */
int previous_ticks = -1;
int backward_ticks = 0;

namespace po = boost::program_options;

namespace blizzard
{
  Odometer::Odometer() : total_ticks(0.0), current_throttle(-2.0),
  meters_per_tick(WHEELS_DIAMETER/RESOLUTION)
  { /* empty */ }

  void Odometer::loadConfig(std::string filename)
  {
    // Describe the valid options
    po::options_description desc;
    desc.add_options()
      ("odometer.meters_per_tick", po::value<double>());

    // Create a place to store the option values
    po::variables_map vm;
    // Load the options into the map
    std::ifstream filestream(filename.c_str());
    po::store(po::parse_config_file(filestream, desc, true), vm);
    po::notify(vm);

    // Assign the values to the appropriate places
    this->meters_per_tick = vm["odometer.meters_per_tick"].as<double>();
  }

  void Odometer::onNewReading(int new_ticks, double target_throttle)
  {
    int ticks = 0;		// # of ticks from last update

    if(previous_ticks == -1)
      ticks = new_ticks;
    else
      ticks = new_ticks - previous_ticks;

    if(current_throttle < -1.0)
      current_throttle = target_throttle;

    printf("new_ticks: %i\n", new_ticks);
    printf("ticks: %i\n", ticks);
    printf("current_throttle: %f\n", current_throttle);
    printf("target_throttle: %f\n", target_throttle);

    if(current_throttle > 0){
      printf("1st POSSIBILITY\n");
      this->total_ticks = this->total_ticks + ticks;
    }

    else if(current_throttle < 0){
      printf("2nd POSSIBILITY\n");
      backward_ticks += ticks;
    }
    else
      printf("3rd Possibility");

    previous_ticks = this->total_ticks + backward_ticks;
    current_throttle = target_throttle;

    //std::cout << "total_ticks: " << this->total_ticks << std::endl;
    printf("total_ticks: %i\n", this->total_ticks);
    printf("previous_ticks: %i\n", previous_ticks);
    printf("backward_ticks: %i\n", backward_ticks);
  }

  double Odometer::calcSpeed(double dist1, double dist2, double time1, double time2)
  {
    double speed = 0;

    if (time2 - time1 == 0){
      speed = -1;
    }
    else
      speed = (dist2 - dist1)/(time2 - time1);

    return speed;
  }

  double Odometer::getDistance()
  {
    return double(this->total_ticks) * this->meters_per_tick;
  }
} // end namespace sb_devices
