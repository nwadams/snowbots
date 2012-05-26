#include "serialBridge.h"

namespace rSerial{

serialBridge::serialBridge(){
	sprintf(portpath,"");
	baudrate = 0;
	state = false;
	obs = NULL;
}

serialBridge::~serialBridge(){
	if(state)
	{
		close();
	}
}

bool serialBridge::init(char* path, int baud){
	
	sprintf(portpath,"%s",path);
	baudrate = baud;
	
	Py_Initialize();
    PyRun_SimpleString("import serial");
    PyRun_SimpleString("import time");
    
    char initString[STRING_SIZE];
    sprintf(initString,"ser = serial.Serial('%s',%d)",portpath,baudrate);
    PyRun_SimpleString(initString);
    
    if(obs != NULL)
	{
		*obs << initString << std::endl;
	}
    
    state = true;
    
    return true;
}

bool serialBridge::close(){
	Py_Finalize();
	state = false;
	return true;
}

bool serialBridge::sendString(char* str){
	char msgString[STRING_SIZE];
	sprintf(msgString,"ser.write('%s')",str);
	PyRun_SimpleString(msgString);
	if(obs != NULL)
	{
		*obs << msgString << std::endl;
	}
	return true;
}

bool serialBridge::delay(double seconds){
	char msgString[STRING_SIZE];
	sprintf(msgString,"time.sleep(%f)",seconds);
	PyRun_SimpleString(msgString);
	if(obs != NULL)
	{
		*obs << msgString << std::endl;
	}
	return true;
}

bool serialBridge::ready(){
	return state;
}

bool serialBridge::setObserverStream(std::ostream* observer){
	obs = observer;
	return true;
}
}