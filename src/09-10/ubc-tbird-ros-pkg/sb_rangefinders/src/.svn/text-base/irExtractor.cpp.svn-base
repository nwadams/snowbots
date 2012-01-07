/** 
 * irExtractor ROS Node
 * This node subscribes to the robot_state topic and takes the ir
 * analog values via a RobotState message and then reads the necessary
 * information off the config file (e.g: robot.cfg). It then combines
 * these information to create a LaserScan message and publishes this
 * message on the ir_info topic.
 * 
 * @author: Navid Fattahi <navid.fattahi@gmail.com>
 */


#include <boost/program_options.hpp>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sb_config/config_file.h>
#include <sb_msgs/RobotState.h>

/* Constants */
const std::string NODE_NAME 	  = "ir_extractor";
const std::string SUBSCRIBE_TOPIC = "robot_state";
const std::string PUBLISH_TOPIC   = "ir_info";
const int MSG_QUE_SIZE = 20;

/* Global Variables */
int id;
int num;
double center;
double angle_increment;
double min_range;
double max_range;

namespace po = boost::program_options;

/* Method Declarations */
void loadConfig(std::string filename);
void callBack(const sb_msgs::RobotStateConstPtr&);
void getDistances(double* analog, int num, double* ranges);
sensor_msgs::LaserScan LaserScanMsgGenerator(double* ranges, int num);


using namespace ros;

Publisher dataPub;

int main (int argc, char** argv)
{
	/* Initializing this node */
	ROS_INFO("Starting %s", NODE_NAME.c_str());
	init(argc, argv, NODE_NAME);

	/* Opening and loading the Config file */
	std::string cfgFile;
	if (! sb_config::findConfigFile(argc, argv, cfgFile))
	{
		ROS_FATAL("Can't find configuration file.");
		shutdown();
	}
  	ROS_INFO("Loading configuration in %s", cfgFile.c_str());
	loadConfig(cfgFile);

	std::cout << "id = " << id << std::endl;
	std::cout << "num = " << num << std::endl;
	std::cout << "angle_increment = " << angle_increment << std::endl;
	std::cout << "center = " << center << std::endl;
	std::cout << "min_range = " << min_range << std::endl;
	std::cout << "max_range = " << max_range << std::endl;
	NodeHandle n;

	/* Creating the subscriber and publisher */
	Subscriber dataSub = n.subscribe(SUBSCRIBE_TOPIC, MSG_QUE_SIZE, callBack);
	dataPub = n.advertise<sensor_msgs::LaserScan>(PUBLISH_TOPIC, MSG_QUE_SIZE);

	/* Main loop */
	while (ok())
		spin();

	/* Informing the user when getting shut down */
	ROS_INFO("Shutting Down %s", NODE_NAME.c_str());
}

/**
 * This function get called whenever a new messaged is arrived.
 * When the LaserScan message is ready, this function publishes
 * the message out.
 */
void callBack(const sb_msgs::RobotStateConstPtr& robotStateMsg)
{
	double irAnalog[num];	//this array stores raw analog voltages
	double ranges[num];	//this array stores distances
	double myId = id;	//port id

	// Reading the analog values off the RobotState message	
	for(int i = 0; i < num; i++)
	{
		irAnalog[i] = robotStateMsg->analog[myId];
		myId++;
	}

	//Converting analog values to meaningful voltages
	getDistances(irAnalog, num, ranges);

	//Creating a LaserScan message
	sensor_msgs::LaserScan laserScanMsg = LaserScanMsgGenerator(ranges, num);

	//Publishing the LaserScan message
	dataPub.publish(laserScanMsg);
}

/**
 * This function converts the analog values stored in the
 * "analog" array to meaningful distances (in meters) and
 * sores them in the "ranges" array.
 */
void getDistances(double* analog, int length, double* ranges)
{
	for (int i = 0; i < length; i++)
	{
		// y = A*exp(-Bx)+C
		// A = 3.719, B = -0.02893, C = 0.3673
		ranges[i] = 3.719*exp(-0.02893*analog[i])+0.3673;
	}
	//TODO: Make sure the formula returns the right values
}

/**
 * This function generates a LaserScan message using the information
 * stored in the global variables and also stores in the array that
 * it has been passed.
 */
sensor_msgs::LaserScan LaserScanMsgGenerator(double *myRanges, int length)
{
	sensor_msgs::LaserScan laserScan;

	// Calculating the minimum (right-most) and maximum
	// (left-most) angles
	double angle_min = center - (num/2) * angle_increment;
	double angle_max = center + (num/2) * angle_increment;

	// Assigning the LaserScan message values
	laserScan.angle_min = angle_min;
	laserScan.angle_max = angle_max;
	laserScan.angle_increment = angle_increment;
	laserScan.time_increment = 0;
	laserScan.scan_time = 0;
	laserScan.range_min = min_range;
	laserScan.range_max = max_range;
	
	for (int i = 0; i < length; i++)
	{
		laserScan.ranges[i] = myRanges[i];
	}

	//laserScan.intensities does not need to be filled ir for IRs
	
	return laserScan;
}

/**
 * This function reads the config file and gets the required information
 * out of it and then assign the values to the global variables
 */
void loadConfig(std::string filename)
{
	// Describe the valid options
	po::options_description desc;
	desc.add_options()
	("ir.id", po::value<int>())
	("ir.num", po::value<int>())
	("ir.center", po::value<double>())
	("ir.angle_increment", po::value<double>())
	("ir.min_range", po::value<double>())
	("ir.max_range", po::value<double>());

	// Create a place to store the option values
	po::variables_map vm;
	// Load the options into the map
	std::ifstream filestream(filename.c_str());
	po::store(po::parse_config_file(filestream, desc, true), vm);
	po::notify(vm);

	// Assign the values to the appropriate places
	id 		= vm["ir.id"].as<int>();
	num 		= vm["ir.num"].as<int>();
	center 		= vm["ir.center"].as<double>();
	angle_increment = vm["ir.angle_increment"].as<double>();
	min_range 	= vm["ir.min_range"].as<double>();
	max_range 	= vm["ir.max_range"].as<double>();
}
