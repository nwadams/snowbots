#include <iostream>
#include <sstream>
#include <string>

using namespace std;

struct LidarData
{
	//polar coordinates
	double polar[1000];
	double theta[1000];
	
	//cartesian coordinates
	double x[1000];
	double y[1000];
};
