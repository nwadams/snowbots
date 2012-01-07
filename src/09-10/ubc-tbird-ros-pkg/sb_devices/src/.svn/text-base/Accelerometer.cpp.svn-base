#include <boost/program_options.hpp>
#include <fstream>
#include <sb_devices/Accelerometer.h>

namespace po = boost::program_options;

namespace sb_devices
{
  Accelerometer::Accelerometer(): zero(1.5),slope(0.38)
  { /* empty */ }

  void Accelerometer::loadConfig(std::string filename)
  {
    // Describe the valid options
    po::options_description desc;
    desc.add_options()
      ("accelerometer.zero", po::value<double>( &this->zero ))
      ("accelerometer.slope", po::value<double>( &this->slope ));

    // Create a place to store the option values
    po::variables_map vm;
    // Load the options into the map
    std::ifstream filestream(filename.c_str());
    po::store(po::parse_config_file(filestream, desc, true), vm);
    po::notify(vm);
  }

  double Accelerometer::getAccel(double voltage)
  {
    return ( (slope / 9.8) * (voltage - zero));
  }
} // end namespace sb_devices

