#include "course_map.h"
#include <vector>
#include <fstream>
#include <iostream>

using namespace std;

bool loadMap(const char *mapfile, vector<CourseSection> &map)
{
	fstream fin(mapfile, ifstream::in);
	if (! fin.is_open() )
	{
		cout << "Could not open " << mapfile << endl;
		return false;
	}
//	cout << "Alright, " << mapfile << " opened." << endl;
	while ( fin )
	{
//		cout << "Reading line from " << mapfile << endl;
		double length;
		int turn;
		if ( fin >> length >> turn )
		{
			CourseSection sec = { length, turn };
			map.push_back(sec);
		}
	}
	fin.close();
	return true;
}
