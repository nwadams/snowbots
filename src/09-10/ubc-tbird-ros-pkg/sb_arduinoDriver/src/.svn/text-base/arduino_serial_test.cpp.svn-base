#include <iostream>
#include <sstream>
#include "SerialCommunication.h"
#include <string>

using namespace std;

int main(void)
{
	SerialCommunication link(115200, "/dev/ttyUSB0");
    stringstream line;
	string data;
	int error = 0;
	for(unsigned char i = 0;;i++)
	{
        data = i;
        for(int j = 0; j < 10; j++)
            data += i;
        string data2;
        cout << (int)data[0] << (int)data[1] << (int)data[2] << (int)data[10] << endl;;
		link.writeData(data);
		usleep(20000);
		
		data2 = link.readData(65);
		cout << (int)data2[0] << (int)data2[1] << (int)data2[2] << (int)data2[60]<< endl << endl;
		cout << data2.length() << endl;
		if((int)data[0] != (int)data2[0] )
		{   
		    error++;
		    if (error > 5)
		        
		        return 0;
		    cout << error << endl;
		}
		link.clearBuffer();
		usleep(10000);
	}
	return 0;
}
