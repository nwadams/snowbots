/**
 * @date Sept 17th 2008
 * @author Matthew Baumann
 * @brief tbrclient is a class that can be inserted into a C++ program 
 * that will handle UDP communication with tbrprobe.  Thus, any 
 * client program can treat this module as an abstraction of the 
 * tbrprobe-controlled hardware.
 */


#ifndef TBRCLIENT_H
#define TBRCLIENT_H

/***************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <iostream>
#include <ostream>

/* udp_constants.h is part of tbrprobe that enumerates the various device ids,
  which have to be sent in text format in the UDP messages. */
#include <tbrprobe/udp_constants.h>

/***************************************************************************************/

#define UDP_PORT 1225
#define LOCALHOST_IP "127.0.0.1"
#define BUF_SIZE 100
#define SENSOR_ERROR_VAL -1.0

/** Represents a reading from the odometer.  Contains four members, all 
    of which are doubles: distance, velocity, acceleration, and jerk. */
typedef struct {
	double distance;
	double velocity;
	double acceleration;
	double jerk;
} OdometerReading;

namespace tbrprobe07{
	/**
	 * tbrclient class
	 * @author Matthew Baumann
	 */
	class tbrclient{
		public:
			tbrclient();
			~tbrclient();
		
			/**
			 * Opens the UDP port and establishes communication with the
			 * tbrprobe server.
			 * @param port The port number that the tbrprobe server is
			 * listening on.
			 * @param addr The IP address of the tbrprobe server.
			 */
			bool initialize(int port, const char* addr);
			bool finalize();
			
			
			/**
			Indicates if communication with the tbrprobe server has
			been established.
			@return true if communications are online and ready to transmit;
			false otherwise.
			**/
			bool ready();
			
			/** Sends a UDP packet to tbrprobe commanding the steering to move
			 * to the position indicated.  The value will be clamped to the
			 * range: [-100,100] to abstractly represent the servo's range of
			 * motion.
			 * @param value The steering value, between -100 and 100
			 * @return REturns a boolean stating success or failure
			 */
			bool setSteering(int value);
			
			
			///sends a UDP packet to tbrprobe commanding the throttle to set
			///to the value indicated.  The value will be clamped to the range:
			///[-100,100] to abstractly represent the servo's range of motion.
			bool setThrottle(int value);
			
			/// Get the last steering value that was sent to this object
			int getSteering();
			
			/// Get the last throttle value that was sent to this object
			int getThrottle();
			
            /// set the tilt and pan of the camera mount turret
			bool setTurret(int tilt, int pan);
            /// Get the current pan of the camera mount turret
            int getPan();
            /// Get the current tilt of the camera mount turret
            int getTilt();
			
			/// Requests sonar range information from the server.  Suffixes
			/// indicate the direction of the sonar: 
			/// NW = northwest (front left sonar)
			/// S = south (rear center sonar)
			double getSonarN();
			double getSonarNW();
			double getSonarNE();
			double getSonarS();
			
			/// Request odometer information from the server.
			double getOdometerDistance();
			double getOdometerVelocity();
			double getOdometerAcceleration();
			
			//request infrared information form the server
			double getInfraredN();
			double getInfraredNW();
			double getInfraredNE();
			double getInfrared(int device_id);
			
			//get the kill switch value
			double getKillSwitchVal();
			
			///sets the observer stream.  All status and debug messages will pass
			///to the observer.
			void setObserver(std::ostream* obs);
		
		private:
			/// Helper method to send commands to the server
			bool sendCommand(char *cmd);
			
			/// Helper method to receive sensor data from the server.
			/// Sensor data is written into reply field.
			/// @return true if data successfully received, false otherwise
			bool receiveSensorData();
			
			/// Requests a reading for the given device id from
			/// the server.
			/// @param sonar_device_id The device id of the
			/// sonar whose data is requested
			/// @return the reading from that sonar, in meters
			double requestSonarData(int sonar_device_id);
			
			
			double requestADData(int ir_device_ID);
			
			double correctIRData(double val);
			
			/**
			Requests a reading from the odometer and stores
			it in the odometer field.
			@return true if it got a reading successfully, false otherwise
			**/
			bool requestOdometerData();
		
			/// The state of this object.  0 is normal, -1 is inactive
			int state;
			
			///buffer for commands to send to tbrprobe
			char command[BUF_SIZE];
			
			/// buffer for replies from tbrprobe
			char reply[BUF_SIZE];
			
			/// buffer to store odometer readings
			OdometerReading odometer; 
			
			//socket file descriptor
			int sockfd;
			
			///connector's address information
			struct sockaddr_in their_addr;
			
			///number of bytes sent
			int numbytes;
			
			///observer ostream pointer.  If not NULL receives all debug messages
			std::ostream* observer;
			
			/// current throttle value
			int current_throttle;
			/// current steering value
			int current_steering;
            /// current pan value of the camera mount turret
            int current_pan;
            /// current tilt value of the camera mount turret
            int current_tilt;

	};
	
}

#endif

