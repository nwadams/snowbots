#include "sonar.h"
#include <pid/pid.h>
#include <iostream>
#include <unistd.h>
#include <highgui.h>
#include <tbrclient.h>

#define MAIN_WINDOW "Sonar"
#define KILL_OFF 0.0
#define LOOP_PERIOD 100
#define STEER_KP 192
#define STEER_KI 0
#define STEER_KD 0
#define STEER_SP 0.5
#define FRONT_MIX 0.1

using namespace std;

int throttle = 0;
bool running = true;

void processKey(char key)
{
	if      ( key == ' ' ) throttle = 0;
	else if ( key == 27  ) running = false;
	else if ( key == '=' )
	{
		if (throttle < 20) throttle = 20;
		else throttle += 5;
	}
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
	cout << "Starting up." << endl;
	tbrprobe07::tbrclient client;
	sonar::SonarFilter filter_r;
	sonar::SonarFilter filter_f;
	vision::PID steer(STEER_KP, STEER_KI, STEER_KD, STEER_SP); 

	// Just use the window for capturing keypresses.
	cvNamedWindow(MAIN_WINDOW);
	cvResizeWindow(MAIN_WINDOW, 10, 10);

	cout << "Connecting..." << endl;

	client.initialize(1225, "127.0.0.1");
	client.setTurret(-100, 0);
	
	while ( running )
	{
		// figure out turn
		double right = filter_r.filter(client.getSonarNE());
		double front = filter_f.filter(client.getSonarN());
		double rstatus = (FRONT_MIX * front) + ((1.0 - FRONT_MIX) * right);
		int turn = steer.step(rstatus, LOOP_PERIOD / 1000);
		//int turn = steer.step(right, LOOP_PERIOD / 1000);

		cout << front << " " << right << "    ";
		
		// figure out throttle
		if ( client.getKillSwitchVal() < KILL_OFF ) throttle = 0;

		cout << turn << " " << throttle << endl;
		
		// set them
		client.setSteering(clamp(turn));
		client.setThrottle(clamp(throttle));
		
		// prepare for next iteration
		char key = cvWaitKey(LOOP_PERIOD);
		processKey(key);
	}
	client.setThrottle(0);
	client.setSteering(0);
}

