#include <iostream>
#include <vector>
#include <sensor_msgs/LaserScan.h>
using namespace std;

#ifndef CONEPOSITION_H
#define CONEPOSITION_H

struct Cone
{
	double x;
	double y;
	double angle;
	double distance;
};

struct Coordinate
{
	double x;
	double y;
};

class ConePosition
{
	private:
		double min_angle;
		double max_angle;
		double d_angle;
		double last_data;
		double last_angle;
		double max_distance;
		double min_distance;
		int num_vals;
		
		void DebugIdentifyCones(double, double);
		double ConvertToRadians(double);
		void addCone(const vector<float> ranges, int count, int i);
		double calculateAngle(int i);

		void addToRightWall(Coordinate point);
		void addToLeftWall(Coordinate point);

		//do insertion sort
		void sortCones();
		
	public:
		
		vector<double> distances;
		vector<double> angles;

		int coneCount;
		
		vector<Cone> LeftWall; 
		vector<Cone> RightWall;

		vector<Coordinate> points;
		
		ConePosition();
		
		void GetConstantsFromLidar(const sensor_msgs::LaserScanConstPtr& msg_ptr, bool swap); // 1st
		void IdentifyCones(const vector<float> ranges); //3rd 
		void separateCones(); //4th

		void PrintAngles();
		void PrintConeCount();
		void PrintDataFromLidar();	
		void PrintDistances();
		void PrintConePositions();
		void PrintLeftCones();
		void PrintRightCones();
		void PrintXPosition();
		void PrintYPosition();
		
};
#endif



