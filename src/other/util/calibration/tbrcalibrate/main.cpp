

#include <iostream>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <tbrclient/tbrclient.h>

bool running = true;
bool pollsensors = false;

tbrprobe07::tbrclient TC;

int steering = 100;
int throttle = 100;

int turretpan = 100;
int turrettilt = 100;



void callback_trackbar1(int cval) {
	TC.setSteering(steering-100);
};

void callback_trackbar2(int cval) {
	TC.setThrottle(throttle-100);
};

void callback_trackbar3(int cval) {
	TC.setTurret(turrettilt-100,turretpan-100);
};

void callback_trackbar4(int cval) {
	TC.setTurret(turrettilt-100,turretpan-100);
};


void processKey(char key)
{
	switch(key)
	{
		case 27:
			//esc key
			running = false;
		break;
	}
}

int main(int argc, char* argv[]){
	char key;
	
	char input;
	std::cout << "Enable sensor polling? y/n" << std::endl;
	std::cin >> input;
	pollsensors = (input == 'y' || input == 'Y');
		
	
	
	TC.setObserver(&std::cout);
	// initialize() makes TC (a tbrclient object) talk to the tbrprobe process
	// listening on UDP_PORT
  	TC.initialize(UDP_PORT,LOCALHOST_IP);
  	
  	const char* window1 = "ControlPanel";
  	const char* trackbar1 = "Steering";
  	const char* trackbar2 = "Throttle";
  	const char* trackbar3 = "Turret Pan";
  	const char* trackbar4 = "Turret Tilt";
  	
  	cvNamedWindow(window1,1);
  	
  	cvCreateTrackbar(trackbar1,window1,&steering,200,callback_trackbar1);
  	cvCreateTrackbar(trackbar2,window1,&throttle,200,callback_trackbar2);
  	cvCreateTrackbar(trackbar3,window1,&turretpan,200,callback_trackbar3);
  	cvCreateTrackbar(trackbar4,window1,&turrettilt,200,callback_trackbar4);
	
	while(running){
		
		if(pollsensors){
			double dist = TC.getOdometerDistance();
			double vel = TC.getOdometerVelocity();
			double accel = TC.getOdometerAcceleration();
			
			double sn = TC.getSonarN();
			double snw = TC.getSonarNW();
			double sne = TC.getSonarNE();
			double ss = TC.getSonarS();
			
			std::cout << "[dist,vel,accel] = ["<<dist<<","<<vel<<","<<accel<<"]";
			std::cout << "sonars: [N,NW,NE,S] = ["<<sn<<","<<snw<<","<<sne<<","<<ss<<"]"<<std::endl;
		}
		key = (char) cvWaitKey(5);
		processKey(key); 
	
	}
	
	TC.finalize();
	cvDestroyWindow("Camera");
	
	return 0;
}
