

// \\//\\//\\//\\/\\//\\//\\//\\//\\//\\/\\//\\//\\//\\//\\//\\/\\//\\//\\//\\//\\//\\/\\//\\//\\//\\//\\//\\/\\//\\//
//______________________________________________________________________________________
//                          SERVO MOTOR CONTROL 
//-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//______________________________________________________________________________________
//Command packet Structure sent to motor control: 
// i,j,k,l,m,n,o,p,q,r,s,t,u,S
//
// i = ignored pad as 0
// j = Motor 1 direction ( 0 is stop, 1 is forward, and 2 is reverse)
// k = Motor 1 duty cycle of PWM power output as a  0 (0%) to 255 (100%) 
// l = Motor 2 direction ( 0 is stop, 1 is forward, and 2 is reverse)
// m = Motor 2 duty cycle of PWM power output as a  0 (0%) to 255 (100%) 
// n = Servo Motor 0  position (or speed if continuous rotation) 
// o = Servo Motor 1  position (or speed if continuous rotation) 
// p = Servo Motor 2  position (or speed if continuous rotation) 
// q = Servo Motor 3  position (or speed if continuous rotation) 
// r = Servo Motor 4  position (or speed if continuous rotation) 
// s = Servo Motor 5  position (or speed if continuous rotation) 
// t = Servo Motor 6  position (or speed if continuous rotation) 
// u = Servo Motor 7  position (or speed if continuous rotation) 
// 
// If a packet is sent the device will then report its state back.
// Report Packet sent to PC host:
// a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u,S
// 
// 
// a,b,c,d,e,f,g,h = Unsigned Analog values from 0 (0v) to 1024 (5v)
// i =  ignored pad as 0
// j = Motor 1 direction ( 0 is stop, 1 is forward, and 2 is reverse)
// k = Motor 1 duty cycle of PWM power output as a  0 (0%) to 255 (100%) 
// l = Motor 2 direction ( 0 is stop, 1 is forward, and 2 is reverse)
// m = Motor 2 duty cycle of PWM power output as a  0 (0%) to 255 (100%) 
// n = Servo Motor 0  position (or speed if continuous rotation) 
// o = Servo Motor 1  position (or speed if continuous rotation) 
// p = Servo Motor 2  position (or speed if continuous rotation) 
// q = Servo Motor 3  position (or speed if continuous rotation) 
// r = Servo Motor 4  position (or speed if continuous rotation) 
// s = Servo Motor 5  position (or speed if continuous rotation) 
// t = Servo Motor 6  position (or speed if continuous rotation) 
// u = Servo Motor 7  position (or speed if continuous rotation) 
// 
#define Servo_KeySig 'S'
void* ServoThread(void *arg);
