
// \\//\\//\\//\\/\\//\\//\\//\\//\\//\\/\\//\\//\\//\\//\\//\\/\\//\\//\\//\\//\\//\\/\\//
//______________________________________________________________________________________
//                          Furious MOTOR CONTROL 
//______________________________________________________________________________________
//Command packet Structure sent to Furious control: 
// i,j,k,l,m,n,o,p,q,r,s,t,u,G
//
// i = ignore (0),
//     read srf08/srf10 sonar(id1 to id16),
//     and broadcast sonar id with 17(1st sonar id) to 32(16th sonar id) 
//     MAKE SURE ONLY 1 SONAR IS PLUGGED IN to the i2c bus when assigning
//	Note that the sonars blink their bus id when first powered on.
//
// j = 0  n/a		*more features may be added later
// k = 0  n/a
// l = 0  n/a
// m = 0  n/a
// n = Servo Motor 0  Camera Pan/Yaw position (pin 33)
// o = Servo Motor 1  Camera Tilt/Pitch position (pin 34)
// p = Servo Motor 2  position (or speed if continuous rotation) 
// q = Servo Motor 3  position (or speed if continuous rotation) 
// r = Servo Motor 4  track drive Left throttle position (pin 36)
// s = Servo Motor 5  track drive Right throttle position (pin 37)
// t = Servo Motor 6  drive thottle position (pin 38) 
// u = Servo Motor 7  steer position (pin 40)
// 
// If a packet is sent the device will then report its state back.
// Report Packet sent to PC host:
// a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u,G
#define FURIOUS_SHARP_IR_FRONT_MID_INDEXPOS 0
#define FURIOUS_SHARP_IR_FRONT_LEFT_INDEXPOS 1
#define FURIOUS_SHARP_IR_FRONT_RIGHT_INDEXPOS 2
#define FURIOUS_SHARP_IR_BACK_MID_INDEXPOS 3

#define FURIOUS_LOGIC_BOARD_BATTERY_INDEXPOS 6
#define FURIOUS_MOTOR_BATTERY_INDEXPOS 7
// 
// 
// a,b,c,d,e,f,g,h = Unsigned Analog values from 0 (0v) to 1024 (5v)
// i =  sonar result in cm (or -32768 if no device was found)
#define FURIOUS_SONAR_INDEXPOS 8
// j = odometer counter for poll interval ( signed integer)
#define FURIOUS_ODOMETER_INDEXPOS 9
// k = Unique serial number ID code  
// l = 7    n/a		*more features may be added later
// m = 7    n/a
// n = Servo Motor 0  position (or speed if continuous rotation) 
// o = Servo Motor 1  position (or speed if continuous rotation) 
// p = Servo Motor 2  position (or speed if continuous rotation) 
// q = Servo Motor 3  position (or speed if continuous rotation) 
// r = Servo Motor 4  position (or speed if continuous rotation) 
// s = Servo Motor 5  position (or speed if continuous rotation) 
// t = Servo Motor 6  position (or speed if continuous rotation) 
// u = Servo Motor 7  position (or speed if continuous rotation) 
// 

#define Furious_KeySig 'G'
void* FuriousThread();
void Furious_Send_Command(char arr[], int *Port_fd );		//send packet

