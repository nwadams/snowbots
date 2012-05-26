#include "course_map.h"
#include <iostream>
#include <vector>
#include <unistd.h>
#include <highgui.h>
#include <tbrclient.h>

#define MAIN_WINDOW "CourseMap"

using namespace std;

bool running = true;
int throttle = 0;

void processKey(char key)
{
	if ( key == 27 ) running = false;
	else if ( key == ' ' ) throttle = 0;
	else if ( key == '=' ) throttle += 5;
	else if ( key == '-' ) throttle -= 5;
}

int clamp(int val, int high = 100, int low = -100)
{
	if ( val > high ) return high;
	else if ( val < low ) return low;
	return val;
}


int main(int argc, char** argv)
{
	cout << "Starting up..." << endl;
	tbrprobe07::tbrclient client;
	vector<CourseSection> map;

	// Just use the window for capturing keypresses.
	cvNamedWindow(MAIN_WINDOW);
	cvResizeWindow(MAIN_WINDOW, 10, 10);

	const char *mapfile = "segmap.txt";

	// command-line args
	for ( int i = 1; i < argc; i++ )
	{
		mapfile = argv[i];
	}

	cout << "Connecting..." << endl;
	client.initialize(1225, "127.0.0.1");
	if (! loadMap(mapfile, map) ) exit(1);
	
	int section = 0;
	while ( running )
	{
		double sec_start = client.getOdometerDistance();
		cout << sec_start << endl;
		double distance;
		int turn = map[section].turn;

		do
		{
			client.setThrottle(clamp(throttle));
			client.setSteering(clamp(turn));
			distance = client.getOdometerDistance() - sec_start;
			cout << distance << " " << section << " " << turn << " " << throttle << endl;
			char key = cvWaitKey(100);
			processKey(key);
		}
		while ( running && distance < map[section].length );

		section++;
		if ( section == map.size() )
		{
			cout << "==== new lap ====" << endl;
			section = 0;
		}
	}
	client.setThrottle(0);
	client.setSteering(0);
}

