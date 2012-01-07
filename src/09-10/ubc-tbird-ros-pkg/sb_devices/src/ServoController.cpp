#include <boost/program_options.hpp>
#include <fstream>
#include <sb_devices/ServoController.h>
#include <sb_util/range.h>

namespace po = boost::program_options;

namespace sb_devices
{
  ServoController::ServoController(std::string servo_name) :
    name(servo_name), id(-1), min(0), center(0), max(0)
    { /* empty */ }

  void ServoController::loadConfig(std::string filename)
  {
    // Define the config options we are going to read, which depend
    // on the name of this servo.
    std::ostringstream idstr, maxstr, ctrstr, minstr;
    idstr  << "servo." << this->name << ".id";
    maxstr << "servo." << this->name << ".max";
    ctrstr << "servo." << this->name << ".center";
    minstr << "servo." << this->name << ".min";

    // Describe the valid options
    po::options_description desc;
    desc.add_options()
      (idstr.str().c_str(),  po::value<int>())
      (maxstr.str().c_str(), po::value<int>())
      (ctrstr.str().c_str(), po::value<int>())
      (minstr.str().c_str(), po::value<int>());

    // Create a place to store the option values
    po::variables_map vm;
    // Load the options into the map
    std::ifstream filestream(filename.c_str());
    po::store(po::parse_config_file(filestream, desc, true), vm);
    po::notify(vm);

    // Assign the values to the appropriate places
    this->id     = vm[idstr.str().c_str()].as<int>();
    this->max    = vm[maxstr.str().c_str()].as<int>();
    this->center = vm[ctrstr.str().c_str()].as<int>();
    this->min    = vm[minstr.str().c_str()].as<int>();
  }

  int ServoController::getId()
  {
    return this->id;
  }

  int ServoController::getRawValue(double position)
  {
    if (position < 0.0) {
      return int(sb_util::saturate(
            position, -1.0, 0.0, this->min, this->center));
    }
    else {
      return int(sb_util::saturate(
            position, 0.0, 1.0, this->center, this->max));
    }
  }
} // end namespace sb_devices

