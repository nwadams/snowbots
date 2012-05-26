#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <string.h>
//The Steer servo requires the defined macros in robot_config.h to operate
#include "../robot_config.h" 
#include "pos_battery.h" 

#define BATTERY_SCALE_RESOLUTION 100.0
//note GCC type casting limits % binops... thus we dupe...
#define BATTERY_SCALE_RESOLUTION_INT 100


//Set power gauges for battery (or use linear estimation if undefined batery is used):
#if (BATTERY_TYPE == VENOM_4600)
	//if selected we use the rendered Characterized data for Venom-4600 6-cell NiMh (better fuel estimates)
	//8.1v Battery level
	#define BATTERY_MOTOR_FULL_VOLTAGE_BITVALUE  150.0
	//5.6v
	#define BATTERY_MOTOR_DEPLETED_VOLTAGE_BITVALUE 125.0
	//8.4v
	#define BATTERY_LOGIC_BOARD_FULL_VOLTAGE_BITVALUE 150.0
	//6.5v
	#define BATTERY_LOGIC_BOARD_DEPLETED_VOLTAGE_BITVALUE 110.0
#else
	#define LINEAR_BATTERY_POWER_ESTIMATE
	//8.1v Battery level
	#define BATTERY_MOTOR_FULL_VOLTAGE_BITVALUE  150.0
	//5.6v
	#define BATTERY_MOTOR_DEPLETED_VOLTAGE_BITVALUE 125.0
	//8.4v
	#define BATTERY_LOGIC_BOARD_FULL_VOLTAGE_BITVALUE 150.0
	//6.5v
	#define BATTERY_LOGIC_BOARD_DEPLETED_VOLTAGE_BITVALUE 110.0
#endif

//values of each bit 	
#define BATTERY_MOTOR_BITSTEP_VALUE (BATTERY_SCALE_RESOLUTION/(BATTERY_MOTOR_FULL_VOLTAGE_BITVALUE - BATTERY_MOTOR_DEPLETED_VOLTAGE_BITVALUE))
#define BATTERY_LOGIC_BOARD_BITSTEP_VALUE (BATTERY_SCALE_RESOLUTION/(BATTERY_LOGIC_BOARD_FULL_VOLTAGE_BITVALUE - BATTERY_LOGIC_BOARD_DEPLETED_VOLTAGE_BITVALUE))	


	
//current battery levels
static int POS_Battery_Logic_Board_Value;
static int POS_Battery_Motor_Value;


//////////
//These items are static globals to allow persistance during multi-threaded use
// In this case it is faster than inter thread meassage systems, but could be implemented better.
static pthread_mutex_t POS_Battery_Write_Lock  = PTHREAD_MUTEX_INITIALIZER;
	
//------------------------------------------------------------------------------------------------------------------------------------------------------
//setup
void Init_POS_Battery(void)	//reset the steering to default ready state
{
	//reset mutexs
	pthread_mutex_unlock(&POS_Battery_Write_Lock);
	POS_Battery_Logic_Board_Value = 0;
	POS_Battery_Motor_Value = 0;
}

//------------------------------------------------------------------------------------------------------------------------------------------------------
//get state
int Get_Battery_Level(void)		//corrected percent for battery
{
	return POS_Battery_Logic_Board_Value;
}


//------------------------------------------------------------------------------------------------------------------------------------------------------
//get state
int Get_Battery_Level_Motor(void)
{
	return POS_Battery_Motor_Value;
}



//------------------------------------------------------------------------------------------------------------------------------------------------------
//This lookup table converts the Vbatt sensor data into a fuel gauge
int getBatteryLevel(int digiVolts){

//These calibration values must be found by monitoring the battery with a Voltmeter:
//
// 1.) FULLY DISCHARGE the battery pack
// 2.) FULLY CHARGE the battery pack
// 3.) Let the the battery pack sit disconnected for 60 min
// 4.) Connect the battery to constant load for 1 min to read the correct battery upper limit
// 5.) Read the battery pack bitvalues for every 0.05V voltage-level change and mark the time in minutes
//      this usually equates to 1 reading for every A/D value change from the voltage divider battery sensor.
// 6.) Repeat step 5.)  until Vbatt=6.00V
//
//_________________________________________________________
//Venom 6-Cell 7.2V 4600mAh NiMH Battery pack discharge characteristics:
//_________________________________________________________
//Note: a NiMh  cell has a 1V per cell practical stall when discharging.
//   Thus at around 6V the battery is close to completely depleted.
/*
Voltage_batt,Time_min,AN_Sensor_Level,%Linear
8.10,00,150,100
7.90,30,146,97
7.85,40,145,95
7.80,48,144,93
7.75,60,143,91
7.70,74,142,89
7.65,96,141,87
7.60,204,140,85
7.55,260,139,83
7.50,320,138,81
7.45,359,137,79
7.40,391,136,77
7.35,418,135,75
7.30,438,134,72
7.25,458,133,70
7.20,475,132,68
7.15,489,131,66
7.10,496,130,64
7.05,506,129,62
//rapid drop off typical for NiMh under load
7.00,509,128,60 
6.95,510,127,58
6.90,511,126,56
6.85,512,125,54
6.80,512,124,52
6.75,512,123,50 
6.70,513,122,48
6.65,513,121,46
6.60,514,120,44
6.55,514,119,42
6.45,514,118,40
6.40,514,117,38
6.35,514,116,36
6.30,514,115,34
6.25,514,114,32
6.20,514,113,30
6.15,514,112,28
6.10,514,111,26
6.05,514,110,24
6.00,514,109,22 
//errors in Vref if Logic board in this zone
5.95,515,108,20
5.90,515,107,18
5.85,515,106,16
5.80,516,105,14
5.75,518,104,12
5.70,520,103,10
5.65,522,102,8
5.60,529,101,6
5.55,536,100,4
5.50,549,99,2
5.45,553,98,0
5.40,556,97,-2
5.35,558,96,-4 
//mcu on Logic Board crashes
5.30,560,95,-6
5.25,560,94,-8
5.20,560,93,-10
5.15,560,92,-12
5.10,560,91,-14
5.05,560,90,-16
5.00,560,88,-18
*/
//_________________________________________________________	
	
switch(digiVolts){
	//These values were created with the Excel Doc battery characterization data
	//Any data gaps in  the cases are assumed linear and interpolated
	
	//format:  case(voltageLevel):{return powerLevelApprox;}.      //based on time spent under load
#if (BATTERY_TYPE == VENOM_4600)	
	case(150):{ return 100;}	//Venom 6-Cell 7.2V 4600mAh NiMH Battery pack discharge characteristics:
	case(149):{ return 99;}
	case(148):{ return 98;}
	case(147):{ return 96;}
	case(146):{ return 94;}
	case(145):{ return 92;}
	case(144):{ return 91;}
	case(143):{ return 88;}
	case(142):{ return 86;}
	case(141):{ return 81;}
	case(140):{ return 60;}
	case(139):{ return 49;}
	case(138):{ return 38;}
	case(137):{ return 30;}
	case(136):{ return 24;}
	case(135):{ return 19;}
	case(134):{ return 15;}
	case(133):{ return 11;}
	case(132):{ return 8;}
	case(131):{ return 5;}
	case(130):{ return 4;}
	case(129):{ return 2;}
	case(128):{ return 1;}
	case(127):{ return 1;}
	case(126):{ return 1;}
#endif
	default:{ return 0;}
}
return 0;	//gcc =)	
}


//------------------------------------------------------------------------------------------------------------------------------------------------------
//set abstract state
void Update_Battery_Level(int rawAN7data, int rawAN6data)		//setting abtract control of  Servo Steer device -50% to 0 to 50%
{
	while(pthread_mutex_lock(&POS_Battery_Write_Lock) != 0)	//prevent thread lock up
	{		
			usleep(1027+(rawAN7data%10));	//odd number for timer
	}

	#if defined(LINEAR_BATTERY_POWER_ESTIMATE)
		//calculate estimates...
		POS_Battery_Logic_Board_Value = (BATTERY_LOGIC_BOARD_BITSTEP_VALUE*(rawAN7data - BATTERY_LOGIC_BOARD_DEPLETED_VOLTAGE_BITVALUE));
		POS_Battery_Motor_Value = (BATTERY_MOTOR_BITSTEP_VALUE*(rawAN6data - BATTERY_MOTOR_DEPLETED_VOLTAGE_BITVALUE)); 
			
		if(POS_Battery_Logic_Board_Value > BATTERY_SCALE_RESOLUTION_INT){
		POS_Battery_Logic_Board_Value = (BATTERY_SCALE_RESOLUTION_INT);
		}
		
		if(POS_Battery_Motor_Value > BATTERY_SCALE_RESOLUTION_INT){
		POS_Battery_Motor_Value =  (BATTERY_SCALE_RESOLUTION_INT); 
		}
	#else
		//Limit battery characterization data   (0% if <= min,    100% if >= max) 
		if(rawAN7data > BATTERY_LOGIC_BOARD_FULL_VOLTAGE_BITVALUE ){
			rawAN7data = BATTERY_LOGIC_BOARD_FULL_VOLTAGE_BITVALUE;
		}
		if(rawAN6data > BATTERY_MOTOR_FULL_VOLTAGE_BITVALUE){
			rawAN6data =  BATTERY_MOTOR_FULL_VOLTAGE_BITVALUE; 
		}
		//get characterized sample from tested data...
		POS_Battery_Logic_Board_Value = getBatteryLevel(rawAN7data);		//assumes both battery packs are the same type
		POS_Battery_Motor_Value = getBatteryLevel(rawAN6data);
	#endif

		
	if((POS_Battery_Logic_Board_Value <= 0) || (POS_Battery_Motor_Value <= 0))
	{
		printf("\33[10;1HBattery Error: Board=%i(%i%%), Motor=%i(%i%%)",rawAN7data,POS_Battery_Logic_Board_Value,rawAN6data,POS_Battery_Motor_Value);
		//Todo: consider 3 strikes safe shutdown 
	}else{
		printf("\33[10;1HBattery Levels: Board=%i%%, Motor=%i%%",POS_Battery_Logic_Board_Value,POS_Battery_Motor_Value);
	}
	

	pthread_mutex_unlock(&POS_Battery_Write_Lock);	//enable other threads to write state
}
