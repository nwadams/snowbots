/*
Navigation 1.4

Code used during Robot Racing 2011 Competition

Name: Edward Han
Date: July 23, 2011

*/

#include <iostream>
#include <string>
#include <vector>
using namespace std;

struct Cone
{
	double distance;
	double angle;
};

class ConeLocation
{
	private:	
	
		int coneCount, total, queueCount;
		int inner_view_angle, full_view_angle, cent;
		bool new_cone;
		double maxDistance, minDistance, maxAngle, minAngle, angleInc, last_distance, last_angle;
		double avg_distance, avg_angle;
		string text_distance;
		string text_angle;
		Cone cone, point;

		vector<Cone> cone_list, point_list;
		vector<double> distances, angles, queue_distance, queue_angle;
		
		void AddToConeList(double, double);
		void AddToPointList(double, double);

		void GetAverage(vector<double>, string);

		void ItIsTheFirstCone(double, double);
		bool ItIsANewCone(double, double);
		bool ItIsTheSameCone(double, double);
		void ItIsTheLastCone(double, double);
	
		void StoreQueue(double, double);
		void ClearQueue();

		double ToDegrees(double);

		void PrintAverage(double);
		void PrintQueue(vector<double>);

	
	public:

		ConeLocation();
		void GetConstants(double, double, double, double, double);
		void GetMyData(double[]);
		void GetDistanceAngle();
		void IdentifyCone();
		void SeparateCone();

		void IdentifyPoint();
		void SeparatePoint();
		void PrintPointList();
		
		void Clear();
		
		void PrintConstants();
		void PrintDistance();
		void PrintAngle();
		void PrintDistanceAngle();
		void PrintConeCount();
		void PrintMyData();
		void PrintConeList();
		void PrintConeStage();
		void PrintCone();
		
		double my_data[550];
		vector<Cone> middle_left, left, middle_right, right, front;
	
};
