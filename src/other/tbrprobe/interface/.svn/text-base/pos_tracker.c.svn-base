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
//The speedometer/odometer requires defined macros in robot_config.h to operate
#include "../robot_config.h"
#include "pos_tracker.h"

#define THROTTLE_SCALE_RESOLUTION 100.0 
//note GCC type casting limits % binops... thus we dupe...
#define THROTTLE_SCALE_RESOLUTION_INT 100

//////////
//cast constants
#define EVILMILLION 1000000.0
#define EVILKM 1000.0

//min re-retry intervals in sec.
#define SAMPLE_RATE_MIN 0.100

//set the intervals timeout for diff sec.
#define SAMPLE_RATE_MAX (2.0 *TIRE_ROTATIONS_PER_METER)

//////////
//These items are static globals to allow persistance during multi-threaded use
// In this case it is faster than inter thread meassage systems, but could be implemented better.
static pthread_mutex_t POS_Write_Lock = PTHREAD_MUTEX_INITIALIZER;
//timers that help perform the math
static double POS_Time_Elapsed_Clock;
static struct timeval POS_Timer_Last_Updated_Clock;
static struct timeval POS_Timer_Current_Clock;
//velocity vars
static double POS_Velocity;
static double POS_Rot_Accumulator;
//acceleration vars
static double POS_Acceleration;
static double POS_Velocity_Temp;
//jerk vars
static double POS_Jerk;
static double POS_Acceleration_Temp;
//distance vars
static double POS_Distance_m;
static double POS_Distance_km;

/////////////
//These items control the abstract throttle direction and the -100%-0-100% drive settings
static int POS_Thottle_Percent;

//set the acceleration for direction changes (100/10)= 10 steps
#define THROTTLE_Attack 10
#define THROTTLE_Decay 10


//////////////////////////////////////////////////////////////////////////////////////////

void Init_POS_Tracker(void)
{
	//reset mutexs
	pthread_mutex_unlock(&POS_Write_Lock);
	POS_Thottle_Percent = 0;
	POS_Time_Elapsed_Clock = 0.0;
	POS_Rot_Accumulator = 0.0;
	POS_Distance_m = 0.0;
	POS_Distance_km = 0.0;
	POS_Velocity = 0.0;
	POS_Jerk = 0.0;
	POS_Acceleration = 0.0;
	gettimeofday(&POS_Timer_Current_Clock, NULL);
	gettimeofday(&POS_Timer_Last_Updated_Clock, NULL);

}


void Update_Odometer(int rots)	//Speedometer data is derived for time-stamps between polled samples
{
	// Get elapsed timestamp.  We'll need this to determine whether we've just stopped
	// or whether the sample rate is too slow.
	gettimeofday(&POS_Timer_Current_Clock, NULL);
	POS_Time_Elapsed_Clock = (((POS_Timer_Current_Clock.tv_sec - POS_Timer_Last_Updated_Clock.tv_sec) * EVILMILLION) +
							  (POS_Timer_Current_Clock.tv_usec - POS_Timer_Last_Updated_Clock.tv_usec)) / EVILMILLION;
	// If too much time has passed with no rotations, assume we've stopped.
	if ( POS_Time_Elapsed_Clock > 1.0 )
	{
			POS_Rot_Accumulator = 0;	//overflow error
			POS_Velocity = 0.0;
			POS_Acceleration = 0.0;
			POS_Jerk = 0.0;
	}
	
	if (rots != 0 )	//no movement detected (or sampling error)... abort...
	{
		while (pthread_mutex_lock(&POS_Write_Lock) != 0)	//prevent thread lock up
		{
			usleep(1027 + (rots % 10));	//odd number for timer
		}

		// Get the elapsed timestamp again because we might have just waited for the
		// mutex to become available.
		gettimeofday(&POS_Timer_Current_Clock, NULL);
		POS_Time_Elapsed_Clock = (((POS_Timer_Current_Clock.tv_sec - POS_Timer_Last_Updated_Clock.tv_sec) * EVILMILLION) +
		                          (POS_Timer_Current_Clock.tv_usec - POS_Timer_Last_Updated_Clock.tv_usec)) / EVILMILLION;

		//time independent distance corrections
		POS_Distance_m += rots * (1.0 / TIRE_ROTATIONS_PER_METER);
		if (POS_Distance_m > EVILKM)	//overflow to km resolution
		{
			POS_Distance_km += 1.0;
			POS_Distance_m -= EVILKM;
		} else if (POS_Distance_m < -EVILKM)
		{
			POS_Distance_km -= 1.0;
			POS_Distance_m += EVILKM;
		}


		// is sample rate too fast to calculate velocity?
		if ((POS_Time_Elapsed_Clock <= SAMPLE_RATE_MIN))
		{
			POS_Rot_Accumulator += rots;  //overflow for next sample call
#if defined(DEBUG_LEVEL_0)
			printf ( "\33[25;30HWarning: Please increase sampling delay interval!");
#endif
			pthread_mutex_unlock(&POS_Write_Lock);
			return ;
		}

		// is sample rate too slow to calculate velocity?
		if ((POS_Time_Elapsed_Clock >= SAMPLE_RATE_MAX))
		{
			POS_Rot_Accumulator = 0;	//overflow error
			POS_Velocity = 0.0;
			POS_Acceleration = 0.0;
			POS_Jerk = 0.0;
#if defined(DEBUG_LEVEL_0)
			printf("\33[24;30HWarning: Please decrease sampling delay interval!");
#endif
			POS_Timer_Last_Updated_Clock = POS_Timer_Current_Clock;
			pthread_mutex_unlock(&POS_Write_Lock);
			return ;
		}


		//perform recall of too fast samples?
		if (POS_Rot_Accumulator != 0.0 )
		{
			POS_Rot_Accumulator += rots;	//correct counter?
		}
		else
		{
			POS_Rot_Accumulator = rots;	//just counter
		}

		//log time marker for next sample
		POS_Timer_Last_Updated_Clock = POS_Timer_Current_Clock;
		//get velocity states
		POS_Velocity_Temp = POS_Velocity;
		POS_Velocity = ((POS_Rot_Accumulator / TIRE_ROTATIONS_PER_METER) / POS_Time_Elapsed_Clock);
		//get Acceleration
		POS_Acceleration_Temp = POS_Acceleration;
		POS_Acceleration = POS_Velocity - POS_Velocity_Temp;
		//get jerk
		POS_Jerk = POS_Acceleration - POS_Acceleration_Temp;

		//correct for speed limit
		if (((POS_Velocity) > MAX_VELOCITY_IN_METERS_PER_SECOND) ||
		        ((POS_Velocity) < ( -MAX_VELOCITY_IN_METERS_PER_SECOND)) )		//correct for throttle limits?
		{
#if defined(DEBUG_LEVEL_1)
			printf ( "\33[23;30HThrottling: %.2fm/s exceeds the %.2fm/s limit",
			         POS_Velocity,
			         MAX_VELOCITY_IN_METERS_PER_SECOND);
#endif

#if defined(govern_velocity)
			//speed correction
			if (POS_Thottle_Percent > ((THROTTLE_Decay)))
			{
				POS_Thottle_Percent -= (THROTTLE_Decay);
			}
			else
			{
				if (POS_Thottle_Percent < ( -(THROTTLE_Attack)))
				{
					POS_Thottle_Percent += (THROTTLE_Attack);
				}
			}
#endif
		}

		//correct for acceleration limit
		if (((POS_Acceleration) > MAX_ACCELERATION_IN_METERS_PER_SECOND_PER_SECOND) ||
		        ((POS_Acceleration) < ( -MAX_ACCELERATION_IN_METERS_PER_SECOND_PER_SECOND)) )	//correct for limits?
		{
#if defined(DEBUG_LEVEL_1)
			printf ( "\33[24;30HAcceleration: %.2fm/ss exceeds %.2fm/ss limit",
			         POS_Acceleration,
			         MAX_ACCELERATION_IN_METERS_PER_SECOND_PER_SECOND);
#endif
#if defined(govern_velocity)

			//Decrease velocity attack/decay rates
			if (POS_Thottle_Percent > ((THROTTLE_Decay / 2)))
			{
				POS_Thottle_Percent -= (THROTTLE_Decay / 2);
			}
			else
			{
				if (POS_Thottle_Percent < ( -(THROTTLE_Attack / 2)))
				{
					POS_Thottle_Percent += (THROTTLE_Attack / 2);
				}
			}
#endif
		}

		POS_Rot_Accumulator = 0.0;	 //reset recall counter

	}
#if defined(DEBUG_LEVEL_0)

	printf ("\33[14;30HOdometer:" );		//move cursor to row 18 and  column 30
	printf ( "\33[15;30HSample Interval=%.2f s\33[16;30HVelocity=%.2f m/s\33[17;30HAcceleration= %.2f m/ss\33[18;30HJerk= %.2f m/sss\33[19;30HTotal Distance=%.1fkm %.2f m",
	         POS_Time_Elapsed_Clock,
	         POS_Velocity,
	         POS_Acceleration,
	         POS_Jerk,
	         POS_Distance_km,
	         POS_Distance_m
	       );

#endif

	pthread_mutex_unlock(&POS_Write_Lock);	//enable other threads to write state
}


////////
//Set speed and Getters to report last known state

void Set_Throttle_Setting(int rawdata)		//correct setting for abtract control of ESC device
{

	while (pthread_mutex_lock(&POS_Write_Lock) != 0)	//prevent thread lock up
	{
		usleep(1027 + (rawdata % 13));	//odd number for timer
	}


	//abstract input for steering may be inverted with THROTTLE_CONTROL_INVERTED = -1 for some platforms like traxxas
	rawdata = (THROTTLE_CONTROL_INVERTED * rawdata);

	//filter out bad data on input
	if (rawdata < -THROTTLE_SCALE_RESOLUTION_INT)
	{
		rawdata = -THROTTLE_SCALE_RESOLUTION_INT;
	}

	if (rawdata > THROTTLE_SCALE_RESOLUTION_INT)
	{
		rawdata = THROTTLE_SCALE_RESOLUTION_INT;
	}


	//make sure esch comes to a stop before reversing
	if ((rawdata < 0) && (POS_Thottle_Percent > 0))
	{
		if (POS_Thottle_Percent > (THROTTLE_Decay))
		{
			POS_Thottle_Percent -= THROTTLE_Decay;
		}
		else
		{
			POS_Thottle_Percent = 0;
			return ;
		}

	}
	else
	{
		//make sure esch comes to a stop when reversing before going forwards
		if ((rawdata > 0) && (POS_Thottle_Percent < 0))
		{
			if (POS_Thottle_Percent < ( -THROTTLE_Attack))
			{
				POS_Thottle_Percent += THROTTLE_Attack;
			}
			else
			{
				POS_Thottle_Percent = 0;
				return ;
			}
		}
	}

	//otherwise attempt to change power
	POS_Thottle_Percent = rawdata;
	pthread_mutex_unlock(&POS_Write_Lock);	//enable other threads to write state
	return ;
}

int Get_Throttle_Setting(void)		//Result in servo corrected setting (ESC may not be symetric)
{
	//todo: add speed limit check

	if (POS_Thottle_Percent > 0)	//return a real speed
	{
		return (POS_Thottle_Percent * THROTTLE_FORDWARD_INDEX) + MOTOR_DRIVE_SERVO_STOP;
	}else
	{
		return (POS_Thottle_Percent * THROTTLE_BACKWARD_INDEX) + MOTOR_DRIVE_SERVO_STOP;
	}

	return 0;	//?dumb compiler
}

double Get_Distance(void)		//Result in Meters
{
	return ((POS_Distance_km*EVILKM) + POS_Distance_m);

}

double Get_Velocity(void)		//Result in Meters Per Second
{
	return POS_Velocity;

}

double Get_Acceleration(void)		//Result in Meters Per Second Per Second
{
	return POS_Acceleration;

}

double Get_Jerk(void)		//Result in Meters Per Second Per Second Per Second
{
	return POS_Jerk;

}


