#include <boost/program_options.hpp>
#include <fstream>
#include <sb_devices/Odometer.h>

namespace po = boost::program_options;

namespace sb_devices
{
  Odometer::Odometer() : total_ticks(0.0), meters_per_tick(0.33)
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

  void Odometer::onNewReading(int ticks)
  {
    this->total_ticks += ticks;
  }

  double Odometer::getDistance()
  {
    return double(this->total_ticks) * this->meters_per_tick;
  }
} // end namespace sb_devices

