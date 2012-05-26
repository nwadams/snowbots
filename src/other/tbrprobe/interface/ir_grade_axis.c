//////////////TODO:  FIX AFTER MOTOR CONTROL

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
//trig functions
#include <math.h>

//Requires defined macros in robot_config.h to operate
#include "../robot_config.h"
#include "ir_grade_axis.h"

//sets the noise ignore level
#define IR_NOISE_THRESHOLD 20 
//sets the
#define IR_WALL_TIGGER_THRESHOLD 20

//////////
//These items are static globals to allow persistance during multi-threaded use
// In this case it is faster than inter thread meassage systems, but could be implemented better.
static pthread_mutex_t Grade_IR_Write_Lock = PTHREAD_MUTEX_INITIALIZER;
//ir calibration done?
static char ir_grade_axis_flag;

//data vars for raw sensor distances
static double Grade_IR_Front_MID;
static double Grade_IR_Front_LEFT;
static double Grade_IR_Front_RIGHT;
static double Grade_IR_Back_MID;
//calibrated offsets
static double Grade_IR_Front_MID_Zero;
static double Grade_IR_Front_LEFT_Zero;
static double Grade_IR_Front_RIGHT_Zero;
static double Grade_IR_Back_MID_Zero;
//tilter
static int Grade_IR_ROLL;
static int Grade_IR_YAW;
static int Grade_IR_PITCH;
//alarms
static double Grade_IR_Barrier_Front_MID;
static double Grade_IR_Barrier_Front_LEFT;
static double Grade_IR_Barrier_Front_RIGHT;
static double Grade_IR_Barrier_Back_MID;
static char Grade_IR_Barrier_Type[ 64 ];


//////////////////////////////////////////////////////////////////////////////////////////

void Init_IR_Grade( void )
{
	//reset mutexs
	pthread_mutex_unlock( &Grade_IR_Write_Lock );
	ir_grade_axis_flag = 'n';					//setup the ir sensors again...
	Grade_IR_ROLL = 0;
	Grade_IR_YAW = 0;
	Grade_IR_PITCH = 0;
	Grade_IR_Barrier_Front_MID = 0;
	Grade_IR_Barrier_Front_LEFT = 0;
	Grade_IR_Barrier_Front_RIGHT = 0;
	Grade_IR_Barrier_Back_MID = 0;

	Grade_IR_Front_MID = 0;
	Grade_IR_Front_LEFT = 0;
	Grade_IR_Front_RIGHT = 0;
	Grade_IR_Back_MID = 0;
	Grade_IR_Front_MID_Zero = 10;		// 1/2  way to max range (they WILL all differ)
	Grade_IR_Front_LEFT_Zero = 10;
	Grade_IR_Front_RIGHT_Zero = 10;
	Grade_IR_Back_MID_Zero = 10;

	strcpy( Grade_IR_Barrier_Type, "n/a" );

}


//calculate new axis

void Update_IR_Grade( int Front_Mid, int Front_Left, int Front_Right, int Rear_Mid ) 	//IR data is derived for a polled sample
{
	int tempValue;
	double trigWadjacent;
	double trigEoposite;
	double trigRadjacent;
	double trigTemp;


	while ( pthread_mutex_lock( &Grade_IR_Write_Lock ) != 0 ) 	//prevent thread lock up
	{
		usleep( 1027 + ( Front_Mid % 10 ) );	//odd number for timer
	
	}

	
	pthread_mutex_unlock( &Grade_IR_Write_Lock );	//enable other threads to write state
	return;
	//the IR liniarization constants for 10 bit A/D,  5 Vref, and GP2D12
	//Todo: Replace with a lookup table for the IR sensor with a >>precise<< distance to V conversion
	
	//update local static vars, and note sensor models may be mixed in the future
	if(Front_Mid > 82)
	{
		Grade_IR_Front_MID = (6787.0 / (Front_Mid - 3.0)) - 4.0;	//true centre sensor
	}else{
		Grade_IR_Front_MID = 81;	//max range of GP2D12 set to 80 cm
	}
	
	if(Front_Left > 82)
	{
		Grade_IR_Front_LEFT = (6787.0 / (Front_Left - 3.0)) - 4.0;	//true L sensor
	}else{
		Grade_IR_Front_LEFT = 81;	//max range of GP2D12 set to 80 cm
	}
	
	if(Front_Right > 82)
	{
		Grade_IR_Front_RIGHT = (6787.0 / (Front_Right - 3.0)) - 4.0;	//true R sensor
	}else{
		Grade_IR_Front_RIGHT = 81;	//max range of GP2D12 set to 80 cm
	}
	
	if(Rear_Mid > 82)
	{
		Grade_IR_Back_MID = (6787.0 / (Rear_Mid - 3.0)) - 4.0;	//true Rear Mid sensor
	}else{
		Grade_IR_Back_MID = 81;	//max range of GP2D12 set to 80 cm
	} 


	if ( ir_grade_axis_flag == 'n' ) 	//if first time called than calibrate sensors
	{
		Set_IR_Grade_Zero_Setting( (Grade_IR_Front_MID), (Grade_IR_Front_LEFT), (Grade_IR_Front_RIGHT), (Grade_IR_Back_MID) );
		return ;
	}
	
	//update sensor abstractions
	Grade_IR_Barrier_Front_MID = ( ( Grade_IR_Front_LEFT + Grade_IR_Front_RIGHT ) / 2 );	//average of 30' down sensors
	Grade_IR_Barrier_Front_LEFT = Grade_IR_Front_MID;
	Grade_IR_Barrier_Front_RIGHT = Grade_IR_Front_MID;
	Grade_IR_Barrier_Back_MID = Grade_IR_Back_MID;

	//Is front mid a drop off?
	if (
	    ( ( Grade_IR_Front_MID > ( Grade_IR_Front_MID_Zero + IR_NOISE_THRESHOLD ) ) &&
	      ( Grade_IR_Front_LEFT > ( Grade_IR_Front_LEFT_Zero + IR_NOISE_THRESHOLD ) ) &&
	      ( Grade_IR_Front_RIGHT > ( Grade_IR_Front_RIGHT_Zero + IR_NOISE_THRESHOLD ) ) &&
	      //	 (Grade_IR_Back_MID < (Grade_IR_Back_MID_Zero + IR_NOISE_THRESHOLD)) &&
	      ( Grade_IR_Front_MID_Zero < ( Grade_IR_Front_MID + IR_NOISE_THRESHOLD ) ) &&
	      ( Grade_IR_Front_LEFT_Zero < ( Grade_IR_Front_LEFT + IR_NOISE_THRESHOLD ) ) &&
	      ( Grade_IR_Front_RIGHT_Zero < ( Grade_IR_Front_RIGHT + IR_NOISE_THRESHOLD ) ) ) ||
	    ( Grade_IR_Back_MID_Zero < ( Grade_IR_Back_MID + IR_NOISE_THRESHOLD ) )
	)
	{
		strcpy( Grade_IR_Barrier_Type, "drop-off" );
	}
	else
	{

		//Is front on a hill?
		tempValue = abs( Grade_IR_Barrier_Front_MID - Grade_IR_Front_MID );	//diff between two angled sensors
		if ( ( IR_NOISE_THRESHOLD < tempValue ) )
		{
			if ( ( IR_WALL_TIGGER_THRESHOLD < tempValue ) )
			{
				strcpy( Grade_IR_Barrier_Type, "wall" );
			}
			else
			{

				//if not a hazard, calculate pitch and yaw
				strcpy( Grade_IR_Barrier_Type, "hill" );
				//IR_distances_top_angle_center_deviation_Theta
				//IR_distances_top_angle_center_deviation_Theta_Not
				//IR_distances_vertical_angle_difference_Ef
				//Grade_IR_Front_MID
				//Grade_IR_Front_LEFT
				//Grade_IR_Front_RIGHT
				//Grade_IR_Back_MID

				trigWadjacent = ( Grade_IR_Front_LEFT * Grade_IR_Front_LEFT ) + ( Grade_IR_Front_RIGHT * Grade_IR_Front_RIGHT )
				                - 2 * ( Grade_IR_Front_RIGHT * ( Grade_IR_Front_LEFT * ( cos( IR_distances_top_angle_center_deviation_Theta_Not + IR_distances_top_angle_center_deviation_Theta ) ) ) );

				trigTemp = ( sqrt( trigWadjacent ) / 2 );
				trigEoposite = ( Grade_IR_Front_LEFT * Grade_IR_Front_LEFT ) + ( trigTemp * trigTemp )
				               - 2 * ( trigTemp * ( Grade_IR_Front_LEFT * ( cos( IR_distances_top_angle_center_deviation_Theta_Not ) ) ) );

				trigRadjacent = ( Grade_IR_Front_MID * Grade_IR_Front_MID ) + ( trigEoposite )
				                - 2 * ( sqrt ( trigTemp ) * ( Grade_IR_Front_MID * ( cos( IR_distances_vertical_angle_difference_Ef ) ) ) );

				trigTemp = ( trigWadjacent + ( Grade_IR_Front_LEFT * Grade_IR_Front_LEFT ) - ( Grade_IR_Front_RIGHT * Grade_IR_Front_RIGHT ) )
				           / ( 2 * ( Grade_IR_Front_LEFT * ( sqrt ( trigWadjacent ) ) ) );
				Grade_IR_YAW = ( 180.0 - IR_distances_top_angle_center_deviation_Theta_Not ) - acos( trigTemp ) ;

				trigTemp = ( trigRadjacent + ( Grade_IR_Front_MID * Grade_IR_Front_MID ) - ( trigEoposite * trigEoposite ) )
				           / ( 2 * ( Grade_IR_Front_MID * ( sqrt ( trigRadjacent ) ) ) );
				Grade_IR_PITCH = ( 180.0 - ( IR_distances_vertical_angle_difference_Ef / 2.0 ) ) - acos( trigTemp ) ;

				//TODO: Add distance to center of sensor grid

				//TODO: Add angle check with rear sensor to prevent tip overs, and high centers.

			}
		}
		else
		{
			strcpy( Grade_IR_Barrier_Type, "n/a" );
		}
	}

#if defined(DEBUG_LEVEL_0)
	printf ( "\33[13;1H IR Array: ROLL=%i YAW= %i PITCH= %i Barrier=%f %s",
	         Grade_IR_ROLL,
	         Grade_IR_YAW,
	         Grade_IR_PITCH,
	         Grade_IR_Barrier_Front_MID,
	         Grade_IR_Barrier_Type
	       );
#endif

	pthread_mutex_unlock( &Grade_IR_Write_Lock );	//enable other threads to write state
}


////////
//Zero sensors to flat ground

void Set_IR_Grade_Zero_Setting( int Front_Mid, int Front_Left, int Front_Right, int Rear_Mid )
{
	while ( pthread_mutex_lock( &Grade_IR_Write_Lock ) != 0 ) 	//prevent thread lock up
	{
		usleep( 1223 + ( Rear_Mid % 10 ) );	//odd number for timer
	}

	Grade_IR_Front_MID_Zero = Front_Mid;		// 1/2  way to max range (they WILL all differ)
	Grade_IR_Front_LEFT_Zero = Front_Left;
	Grade_IR_Front_RIGHT_Zero = Front_Right;
	Grade_IR_Back_MID_Zero = Rear_Mid;

	ir_grade_axis_flag = 'y';

	pthread_mutex_unlock( &Grade_IR_Write_Lock );	//enable other threads to write state
}

//getters that strip precision, as the noise will be greater than the information in the fractional component.

int Get_ROLL( void )
{
	return Grade_IR_ROLL;
}

int Get_YAW( void )
{
	return Grade_IR_YAW;
}

int Get_PITCH( void )
{
	return Grade_IR_PITCH;
}

int Get_Barrier( void )
{
	return Grade_IR_Barrier_Front_MID;
}

//getters that strip precision, but should return a linear estimate of the raw IR sensor distances

int Get_IR_Distance_Mid(void)
{
	return Grade_IR_Front_MID;
}

int Get_IR_Distance_Left(void)
{
	return Grade_IR_Front_LEFT;
}

int Get_IR_Distance_Right(void)
{
	return Grade_IR_Front_RIGHT;
}

int Get_IR_Distance_Rear(void)
{
	return Grade_IR_Back_MID;
}

