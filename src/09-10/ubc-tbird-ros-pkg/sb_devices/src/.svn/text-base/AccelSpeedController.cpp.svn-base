#include <boost/program_options.hpp>
#include <fstream>
#include <iostream>
#include <sb_devices/AccelSpeedController.h>

namespace po = boost::program_options;

namespace sb_devices
{
  AccelSpeedController::AccelSpeedController() :
      kp(1.0), ki(0.0), kd(0.0), 
      target_speed(0.0), last_accel(0.0), last_speed(0.0), last_error(0.0),
      last_time(0.0), total_error(0.0), last_throttle(0.0)
  { /* empty */ }


  void AccelSpeedController::loadConfig(std::string filename)
  {
    // Describe the valid options
    po::options_description desc;
    desc.add_options()
      ("speed_control.kp", po::value<double>( &this->kp ))
      ("speed_control.ki", po::value<double>( &this->ki ))
      ("speed_control.kd", po::value<double>( &this->kd ));

    // Create a place to store the option values
    po::variables_map vm;
    // Load the options into the map
    std::ifstream filestream(filename.c_str());
    po::store(po::parse_config_file(filestream, desc, true), vm);
    po::notify(vm);
  }

  void AccelSpeedController::setTarget(double new_target)
  {
    this->target_speed = new_target;
  }

// initializing the setup
  double AccelSpeedController::getSpeed(double accel, double time)
  {
    if (this->last_time == 0.0) {
      this->last_time = time;
      this->last_accel = accel;
      return 0.0;
    }
    
    double delta_accel = accel - this->last_accel;
    double delta_time = time - this->last_time;
  
    if ( delta_time == 0.0 ) {
      return this->last_speed;
    }

    double speed = (delta_accel * delta_time) + this->last_speed;

	  //save current data for next calculation 
   	this->last_speed = speed;
   	this->last_accel = accel;
   	this->last_time = time;
   	return speed;
  }
  
  double AccelSpeedController::getThrottle(double speed)
  {
    if (this->target_speed == 0.0) {
      return 0.0; //if where we wanna go is "don't move", don't bother running
    }
    
    double error = target_speed - speed;
    double delta_error = error - this->last_error;
    this->total_error += error;

    // Use PID to modify throttle setting.
    double throttle = this->last_throttle 
        + (this->kp * error) 
        + (this->ki * total_error) 
        + (this->kd * delta_error);
        
    this->last_error = error;
   	this->last_throttle = throttle;
   	return throttle;
  }

} // end namespace sb_devices


