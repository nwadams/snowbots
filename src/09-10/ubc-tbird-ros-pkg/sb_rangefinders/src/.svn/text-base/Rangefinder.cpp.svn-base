#include <boost/program_options.hpp>
#include <math.h>
#include <fstream>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <sb_rangefinders/Rangefinder.h>


namespace po = boost::program_options;

namespace sb_rangefinders
{
  Rangefinder::Rangefinder() :
      id_start(0), id_increment(0), num(0), 
      center(0.0), angle_increment(0.0), 
      type("long")
  { /* empty */ }

  /**
   * This function reads the config file and gets the required information
   * out of it and then assigns the values to the fields
   */
  void Rangefinder::loadConfig(std::string filename)
  {
    // Describe the valid options
    po::options_description desc;
    desc.add_options()
    ("ir.id_start", po::value<int>( &this->id_start ))
    ("ir.id_increment", po::value<int>( &this->id_increment ))
    ("ir.num", po::value<int>( &this->num ))
    ("ir.center", po::value<double>( &this->center ))
    ("ir.angle_increment", po::value<double>( &this->angle_increment ))
    ("ir.type", po::value<std::string>( &this->type ));

    // Create a place to store the option values
    po::variables_map vm;
    // Load the options into the map
    std::ifstream filestream(filename.c_str());
    po::store(po::parse_config_file(filestream, desc, true), vm);
    po::notify(vm);
  }

  /**
   * This function converts the analog values stored in the
   * "analog" array to meaningful distances (in meters) and
   * sores them in the "ranges" array.
   */
  void Rangefinder::getDistances(
      const std::vector<double> &analog, double* ranges)
  {
    double c2 = 0.3629;
    double c1 = -1.5906;
    double c0 = 1.9580;
    if ( this->type == "short" )
    {
      c2 = 0.2705;
      c1 = -1.0695;
      c0 = 1.1440;
    }

    // Reading the analog values off the RobotState message  
    for (int i = 0,      id = this->id_start; 
         i < this->num; 
         i++,            id += this->id_increment)
    {
      ranges[i] = ( c2 * analog[id] * analog[id]) +
                  ( c1 * analog[id]) + c0;
    }

      // y = A*exp(-Bx)+C
      // A = 3.719, B = -0.02893, C = 0.3673
      //ranges[i] = 3.719*exp(-0.02893*analog[i])+0.3673;
    //TODO: Make sure the formula returns the right values
  }

  /**
   * This function generates a LaserScan message using the information
   * stored in the global variables and also stores in the array that
   * it has been passed.
   */
  sensor_msgs::LaserScan 
  Rangefinder::LaserScanMsgGenerator(double *myRanges)
  {
      sensor_msgs::LaserScan laserScan;

      // Calculating the minimum (right-most) and maximum
      // (left-most) angles
      double angle_min = this->center - (this->num/2) * this->angle_increment;
      double angle_max = this->center + (this->num/2) * this->angle_increment;

      // Assigning the LaserScan message values
      laserScan.angle_min = M_PI * angle_min / 180.0;
      laserScan.angle_max = M_PI * angle_max / 180.0;
      laserScan.angle_increment = M_PI * this->angle_increment / 180.0;
      laserScan.time_increment = 0; // TODO: populate time values
      laserScan.scan_time = 0;
      laserScan.range_min = this->getMinRange();
      laserScan.range_max = this->getMaxRange();
      // Initialize the ranges vector/array to the correct size
      laserScan.ranges.resize(this->num);

      for (int i = 0; i < this->num; i++)
      {
        laserScan.ranges[i] = myRanges[i];
      }

      //laserScan.intensities does not need to be filled for IRs

      return laserScan;
  }

  /**
   * This function gets throttle and steering as parameters and
   * and returns a Twist message with regard to those values
   */
  geometry_msgs::Twist 
  Rangefinder::vectorCalculator(double throttle, double steering)
  {
      //Creating instances of Twist and Vector3 classes	
      geometry_msgs::Twist twistReturn;
      geometry_msgs::Vector3 linearVel;
      geometry_msgs::Vector3 angularVel;

      //Since the robot can move in only one direction (x-axis),
      //we set the other two to zero	
      linearVel.x = throttle;
      linearVel.y = 0;
      linearVel.z = 0;

      //Since the robot can turn around z-axis only,
      //we set the other two to zero
      angularVel.x = 0;
      angularVel.y = 0;
      angularVel.z = steering;

      twistReturn.linear = linearVel;
      twistReturn.angular = angularVel;

      return twistReturn;
  }

  /**
   * This function gets the steering value as its parameter and 
   * uses sqrt function to make the steering more effective (i.e.
   * sharpen it when close to objects and smoothen it when far away
   */
  double Rangefinder::sharpenSteering(double steering)
  {
      double newSteering;

      if (steering >= 0)
        newSteering = sqrt(steering);
      else
        newSteering = sqrt(-steering)*(-1);

      if(newSteering > 1)
        newSteering = 1;
      if(newSteering < -1)
        newSteering = -1;

      return newSteering;
  }

  double Rangefinder::getMinRange()
  {
    if (this->type == "short") {
      return 0.1;
    }
    return 0.2;
  }

  double Rangefinder::getMaxRange()
  {
    if (this->type == "short") {
      return 0.8;
    }
    return 1.5;
  }

  /**
   * Returns the number of rangefinders in the array.
   */
  int Rangefinder::getNumRays()
  {
      return this->num;
  }
} // end namespace sb_rangefinders


