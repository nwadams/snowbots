#include <boost/program_options.hpp>
#include <fstream>
#include <iostream>
#include <sb_devices/SpeedController.h>

namespace po = boost::program_options;

namespace sb_devices
{
  SpeedController::SpeedController() :
      kp(1.0), ki(0.0), kd(0.0), 
      predict_secs(0.3), timeout(1.0),
      target_speed(0.0), last_dist(0.0), last_speed(0.0), last_error(0.0),
      last_time(0.0), total_error(0.0), last_throttle(0.0)
  { /* empty */ }


  void SpeedController::loadConfig(std::string filename)
  {
    // Describe the valid options
    po::options_description desc;
    desc.add_options()
      ("speed_control.kp", po::value<double>())
      ("speed_control.ki", po::value<double>())
      ("speed_control.kd", po::value<double>())
      ("speed_control.predict_seconds", po::value<double>())
      ("speed_control.timeout", po::value<double>());

    // Create a place to store the option values
    po::variables_map vm;
    // Load the options into the map
    std::ifstream filestream(filename.c_str());
    po::store(po::parse_config_file(filestream, desc, true), vm);
    po::notify(vm);

    // Assign the values to the appropriate places
    this->kp = vm["speed_control.kp"].as<double>();
    this->ki = vm["speed_control.ki"].as<double>();
    this->kd = vm["speed_control.kd"].as<double>();
    this->predict_secs = vm["speed_control.predict_seconds"].as<double>();
    this->timeout = vm["speed_control.timeout"].as<double>();
  }

  void SpeedController::setTarget(double new_target)
  {
    this->target_speed = new_target;
  }


  double SpeedController::onNewReading(double distance, double time)
  {
    if (this->last_time == 0.0) {
      this->last_time = time;
      this->last_dist = distance;
      return 0.0;
    }
    if (this->target_speed == 0.0) {
      return 0.0;
    }
    
    double delta_dist = distance - this->last_dist;
    double delta_time = time - this->last_time;
/*
    std::cout << "distance " << distance << "; last_dist " << last_dist
      << "; delta_dist " << delta_dist << std::endl;
    std::cout << "time " << time << "; last_time " << last_time
      << "; delta_time " << delta_time << std::endl;
    std::cout << "last throttle " << this->last_throttle 
      << "; timeout " << this->timeout << std::endl;
*/
    
    if ( delta_time == 0.0 ) {
      return this->last_throttle;
    }
    if ((delta_dist == 0.0) && (delta_time < this->timeout)) {
      return this->last_throttle;
    }

    double speed = delta_dist / delta_time;
    double accel = (speed - this->last_speed) / delta_time;
    // Use acceleration to predict how fast we'll be going
    double predicted = speed + (this->predict_secs * accel);
    double error = target_speed - predicted;
    double delta_error = error - this->last_error;
    this->total_error += error;

    // Use PID to modify throttle setting.
    double throttle = this->last_throttle 
        + (this->kp * error) 
        + (this->ki * total_error) 
        + (this->kd * delta_error);

    /*
    ROS_INFO(
        "dTime: %.2f, speed: %.2f, accel: %.2f, predicted: %.2f, target: %.2f, throttle: %.2f",
        delta_time, speed, accel, predicted, target_speed, throttle
    ); */

    // Save away current values for next time.
    this->last_dist = distance;
    this->last_speed = speed;
    this->last_error = error;
    this->last_throttle = throttle;
    this->last_time = time;
    return throttle;
  }

  double SpeedController::getLastDistance()
  {
     return this->last_dist;
  }


  double SpeedController::getLastTime()
  {
     return this->last_time;
  }
} // end namespace sb_devices


