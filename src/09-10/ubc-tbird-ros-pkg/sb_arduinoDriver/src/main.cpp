#include <iostream>//standard cpp library
#include <stdio.h> //standard c library
#include <unistd.h> // alows for unix terminal functions
#include <fcntl.h>  //used for the function open()
#include <termios.h> //unix serial setup

#include <dirent.h>//used to get acces to the directory on unix systems I'm jusing it to search for serial ports

#include <string>//controling stirngs

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sb_msgs/RobotState.h>
#include <sb_msgs/IMU.h>
#include <sb_msgs/CarCommand.h>
#include <sb_msgs/TurretCommand.h>
#include <sb_msgs/ServoCommand.h>
#include <unistd.h>

using namespace std;

const string NODE_NAME = "arduinoDriver";
const string PUB_TOPIC = "robot_state";
#define STARTCHAR 'B'
#define ENDCHAR 'G'

//Global variables
float pan = 0.0;
float tilt = 0.0;
float throttle = 0.0;
float steering = 0.0;
bool eStop = 0;

int stop; //(1=stoped, 2=wtf, 0 =start)

int fd;//status indicator

struct sensor_data
{
	int x;
	int y;
	int z;
	unsigned long count;
	int ir1;
	int ir2;
	int ir3;
};

void eStop_callback(const std_msgs::BoolConstPtr& msg_ptr)
{
	//eStop call back
	eStop = msg_ptr->data;
	cout << "stop" <<endl;
}

void servo_command_callback(const sb_msgs::ServoCommandConstPtr& msg_ptr)
{
	//callback for servo command.
	//to control servos directly (for calibration)
}

void car_command_callback(const sb_msgs::CarCommandConstPtr& msg_ptr)
{
//	ROS_INFO("driving");
	throttle = msg_ptr->throttle;
	steering = msg_ptr->steering;
	ROS_INFO("I Recieved: Throttle: %f, Steering: %f", throttle, steering);
}


void turret_command_callback(const sb_msgs::TurretCommandConstPtr& msg_ptr)
{
	//callback for turret command
	//to control pan/tilt unit
	pan = msg_ptr->pan;
	tilt = msg_ptr->tilt;
}

void Output(int fd,int throttle[],int steering[], int pan[], int tilt[]);
//sends takes two floats(between -1 and 1) and convert them to two ints 
//(between 0 and 180 but is curently limited to prevent damage) for the arduino

void getline(int fd,char *buffer, int bufsize);
//gets a string of data from the arduino

void init_port(int *fd, unsigned int baud);
//sets the baud rate and some other variables (this is the most black box like code)

bool Serial_Read( int fd, unsigned char *buf, int n, sb_msgs::IMU &imu, sb_msgs::RobotState &state );
//reads the serial port and returns a struct that has the sensor data

void scanDevIDs( char name[], int skipIndex ,char directory[]);
//looks for usb device names on macs

static int one (struct dirent *unused );
//used for the dirent.h library functions

int main(int argc, char** argv) 
{ 
	int fd;
	unsigned char buffer[24];
	char USBPort[1000];
	//char USBPort[]="/dev/tty.usbserial-A900aeEQ";
	//int count=0;
	sensor_data data;
	int end=0;
	//char ask;
	int steer[]={90};
	int throt[]={90};
	int pan_a[] = {90};
	int tilt_a[] = {90};
	int pan_init[] = {90};
	int tilt_init[] = {90};
	int throttle_init[]={90};
	int steer_init[]={90};
	int skip=0;
	char directory1[]="ttyUSB";
	char directory2[]="tty.usb";
	
	
	
	
	
	cout << endl << endl << endl;
	cout << "********************************************************************************" << endl;
	cout << "|                Arduino Serial Comunication and Blizzard Comander             |" << endl;
	cout << "|                          Jarek I.M. May 18, 2010                             |" << endl;
	cout << "********************************************************************************" << endl <<endl ;
	
	
	//scanDevIDs( USBPort, skip, directory1 );
	fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
	if(fd==-1)
	{
		//scanDevIDs( USBPort, skip, directory2 );
		fd = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NDELAY);
	}
	
	//conects to the serialport for comunication and sets a cupple of varialbles
	
    if(fd == -1) 
	{
		cout << endl;
		perror("open_port: unable to open port"); 
		return 0;
	}

	init_port(&fd,38400);         
	//set serial port to 38400buad
	
	ros::init(argc, argv, "arduinoDriver");
	ros::NodeHandle n;
	
	ros::Subscriber Car_Command = n.subscribe("car_command", 1, car_command_callback);
	ros::Subscriber Turret_Command = n.subscribe("TurretCommand", 1, turret_command_callback);
	ros::Subscriber Servo_Command = n.subscribe("ServoCommand", 1, servo_command_callback);
	ros::Subscriber eStop_Command = n.subscribe("eStop", 1, eStop_callback);
	
	ros::Publisher Robot_State = n.advertise<sb_msgs::RobotState>(PUB_TOPIC,1);
	ros::Publisher IMU = n.advertise<sb_msgs::IMU>("IMU",1);
	
	ros::Rate loop_rate(20);
	
	ROS_INFO("arduinoDriver ready");

	Output(fd,throt,steer,pan_a,tilt_a);//sennds comands 
	usleep(1000000);
	
		//set init stuff
	
	sb_msgs::RobotState state;
	sb_msgs::IMU imu;


	while(ros::ok())
	{
		//stringstream data;
		string input;
		int send_throttle, send_steer, send_pan, send_tilt;
		int eStop_count = 0;
		
		if(!eStop)
		{
			//normal operation
			eStop_count = 0;
			send_throttle = throttle*90 + 90;//
			//send_throttle = 97;
			send_steer = -steering*60 + 90;
			send_pan = pan*90 + 90;
			send_tilt = tilt*90+90 ;
		} else {
			//eStop
			if(eStop_count > 30)
			{
				send_throttle = 90;
			} else {
				send_throttle = 90;
			}
			send_steer = 90;
			send_pan = 90;
			send_tilt = 90;
			
			eStop_count++;	
		}
		
		throt[0] = send_throttle;
		steer[0] = send_steer;
		pan_a[0] = send_pan;
		tilt_a[0] = send_tilt;
		cout << throt[0]<< steer[0] << endl;
		Output(fd,throt,steer, pan_a, tilt_a);//sennds comands 
		usleep(10000);
		if(Serial_Read(fd,buffer,4, imu, state))
		{	
			Robot_State.publish(state);
			IMU.publish(imu);
			ROS_INFO("I published RobotState and IMU");	
		}
	    ros::spinOnce();
		loop_rate.sleep();
		
	}
	ROS_INFO("Shutting down %s", NODE_NAME.c_str());
	Output(fd,throttle_init,steer_init,pan_init,tilt_init);
	return (0); 
}


bool Serial_Read( int fd, unsigned char *buf, int n, sb_msgs::IMU &imu, sb_msgs::RobotState &state )//not used but this will be the main reading function latter (I think...)
{
	read ( fd, buf, n );
	

	/*imu.c_x=(buf[1]<<8|buf[0]);
	imu.c_y=(buf[3]<<8|buf[2]);
	imu.c_z=(buf[5]<<8|buf[4]);
	
	imu.g_x=(buf[7]<<8|buf[6]);
	imu.g_y=(buf[9]<<8|buf[8]);
	imu.g_z=(buf[11]<<8|buf[10]);
	
	imu.a_x=(buf[13]<<8|buf[12]);
	imu.a_y=(buf[15]<<8|buf[14]);
	imu.a_z=(buf[17]<<8|buf[16]);
	*/
	//cout << imu.c_x << " " << imu.c_y << endl;
	state.odometer = (buf[1] << 8|buf[0]);

	std::cout << "odometer: " << state.odometer << std::endl;
	std::cout << "buf[0]: " << static_cast<int>(buf[0]) << std::endl;
	std::cout << "buf[1]: " << static_cast<int>(buf[1]) << std::endl;
	//std::cout << "odometer: " << state.odometer << std::endl;

	tcflush ( fd, TCIFLUSH );
	return 1;
	
}



void Output(int fd,int throttle[],int steering[], int pan[], int tilt[])// this function is all my work and I'm realy happy about it 
{
	int B[] = {66};
//	write(fd, B, 1);
//	write(fd, B, 1);
	write(fd, throttle, 1);
	write(fd, steering, 1);
	write(fd, pan, 1);
	write(fd, tilt, 1);
}

void init_port(int *fd, unsigned int baud) 
{ 
    struct termios options; //I hate learning about this structure
    tcgetattr(*fd,&options);
	
    switch(baud) //sets the baud rate I'll add more options later but works
    { 
		case 9600: cfsetispeed(&options,B9600); 
			cfsetospeed(&options,B9600); 
			break; 
		case 19200: cfsetispeed(&options,B19200); 
			cfsetospeed(&options,B19200); 
			break; 
		case 38400: cfsetispeed(&options,B38400); 
			cfsetospeed(&options,B38400); 
			break; 
		case 115000: cfsetispeed(&options,B115200); 
			cfsetospeed(&options,B115200); 
			break;
		default:cfsetispeed(&options,B9600); 
			cfsetospeed(&options,B9600); 
			break; 
    } 
	//to tell you the truth I don't know enufe abaout this to tell you what it does but 
	//I'd sugest leaving it well enufe alone.
    options.c_cflag |= (CLOCAL | CREAD); 
    options.c_cflag &= ~PARENB; 
    options.c_cflag &= ~CSTOPB; 
    options.c_cflag &= ~CSIZE; 
    options.c_cflag |= CS8; //send 8 bits in a byte
    tcsetattr(*fd,TCSANOW,&options); 
} 

static int one (struct dirent *unused )//in linux add const infront of struct
{
	return 1;
}

/*void scanDevIDs( char name[], int skipIndex,char directory[])
{
	struct dirent **eps;
	int n;
	int cnt;
	char* filterResult;
	
	name[ 0 ] = '\0';
	
	n = scandir ( "/dev", &eps, one, alphasort );	//open dir struct
	
	if ( n >= 0 )
	{
		for ( cnt = 0; cnt < n; ++cnt )
		{
			strcpy( name, eps[ cnt ]->d_name );	//list dir file
			
			
			filterResult = strstr( name, directory );	//is there a usb device attached??
			
			
			if ( filterResult != NULL )
			{
				if ( skipIndex < 1 ) 	//write device name and path after skipping prior devices on list
				{
					strcpy( name, "/dev/" );	//list path
					strcat( name, eps[ cnt ]->d_name );  //list dev
					cout <<endl<<endl<< "Attaching to Serial Port: "<<name<<endl<<endl;
					return ;
				}
				skipIndex--;
			}
			else
			{
				strcpy( name, "/dev/ttyACM0" );	//clobber file name
			}
		}
	}
	else
	{
		strcpy( name, "/dev/ttyACM0" );
		perror ( "Couldn't open the directory" );
	}
	
}*/
