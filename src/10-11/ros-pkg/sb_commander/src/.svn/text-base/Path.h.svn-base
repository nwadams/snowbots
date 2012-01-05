/*
Navigation 1.4

Code used during Robot Racing 2011 Competition

Name: Edward Han
Date: July 23, 2011

*/

#include <iostream>
#include <string>

using namespace std;

struct Car
{
	string state;
	string flag;

	double throttle;
	double steering;
};

struct Cone;
struct Sample
{
	double distance;
	double angle;
};


class Path
{
private:
	Sample left, right, front;
	
	int blizzard_steering, stopCount;
	bool stopFlag, adjustFlag, dragStopFlag;
	int drag_race;
	double speed, steering, steering_adjust, front_max, front_min, front_col, side_min, speed_adjust, distance_inc; // circuit
	double drag_front_max, drag_front_min, drag_tolerance, drag_speed, drag_steering, drag_brake;
	int my_second;
	string constants[30][2];

	void GetValues();
	void Reverse();
	void ReadValues();
	void AssignValues();
	void DetermineState(); // circuit 
	void DetermineDragState(); // drag
	void GetDragValues();

	
public:

	Path(); 
	//----------Point--------------

	void FindClosestPoint(vector<Cone>, vector<Cone>, vector<Cone>); // left, front, right
	void PrintClosestPointStage();
	void PrintClosestPoint();

	//---------End Point-----------

	void DeterminePath();
	void Clear();

	Car car;
};
