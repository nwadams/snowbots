#include <iostream>
#include "tbrclient.h"

int main(int argc, char* argv[])
{
	std::cout << "Program Start." << std::endl;
	
	int steps = 0;
	
	double pos = 0;
	double step = 0.001;
	double max = 100;
	double min = -100;
	
	tbrprobe07::tbrclient TC;
	
	TC.setObserver(&std::cout);
	
	TC.initialize(UDP_PORT,LOCALHOST_IP);
	
	while(steps < 4)
	{
		if(pos >= max) {step = -step; steps++;}
		if(pos <= min) {step = -step; steps++;}
		
		//for(int i = 0; i < 1000000; i++);
		pos += step;
		
		if(TC.ready())
		{
			if(TC.setSteering((int) pos)) {
				std::cout << "Success" << std::endl;
			}
			std::cout << "SonarNE " << TC.getSonarNE() << std::endl;
			std::cout << "SonarNW " << TC.getSonarNW() << std::endl;
			
			std::cout << "Odom dist" << TC.getOdometerDistance() << std::endl;
			std::cout << "Odom velo" << TC.getOdometerVelocity() << std::endl;
		}
	}
	
	TC.finalize();
	
	std::cout << "Program Complete." << std::endl;
	return 0;
}
