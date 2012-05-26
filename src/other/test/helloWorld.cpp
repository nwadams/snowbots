#include <tbrclient/tbrclient.h>
#include <stdio.h>
#include <iostream>

using namespace tbrprobe07;

const int PORT = 1225;
const char *HOST = "127.0.0.1";

int main()
{	
	tbrclient client;
	if (client.initialize(PORT, HOST)== false)
	{
		std::cout << "Connection Failed!" << std::endl;
	}
	else
	{
	//	std::cout << "Connection Established!" << std::endl;
		std::cout << "Starting distance is " << 
			client.getOdometerDistance() << std::endl;
		
		client.setThrottle(75);
		
		while (client.getOdometerDistance() < 2.00)
		{
			usleep(10000);
			std::cout << "Distance is " << 
			client.getOdometerDistance() << std::endl;
		}

		//client.setSteering(95);
		
		client.setThrottle(10);
	}
	return 0;
}
