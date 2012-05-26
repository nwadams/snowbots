
// \\//\\//\\//\\/\\//\\//\\//\\//\\//\\/\\//\\//\\//\\//\\//\\/\\//\\//\\//\\//\\//\\/\\//\\//\\//\\//\\//\\/\\//\\//
//______________________________________________________________________________________
//                          MOTOR CONTROL 
//______________________________________________________________________________________
//Command packet Structure sent to motor control: 
// i,j,k,l,m,X
//
// i = output power port control line pattern as a  0 to 255 (all on)
// j = Motor 1 direction ( 0 is stop, 1 is forward, and 2 is reverse)
// k = Motor 1 duty cycle of PWM power output as a  0 (0%) to 255 (100%) 
// l = Motor 2 direction ( 0 is stop, 1 is forward, and 2 is reverse)
// m = Motor 2 duty cycle of PWM power output as a  0 (0%) to 255 (100%) 
// 
// If a packet is sent the device will then report its state back.
// Report Packet sent to PC host:
// a,b,c,d,e,f,g,h,i,j,k,l,m,X
// 
// 
// a,b,c,d,e,f,g,h = Unsigned Analog values from 0 (0v) to 1024 (5v)
// i = output power port control line pattern as a  0 to 255 (all on)
// j = Motor 1 direction ( 0 is stop, 1 is forward, and 2 is reverse)
// k = Motor 1 duty cycle of PWM power output as a  0 (0%) to 255 (100%) 
// l = Motor 2 direction ( 0 is stop, 1 is forward, and 2 is reverse)
// m = Motor 2 duty cycle of PWM power output as a  0 (0%) to 255 (100%) 
// 
#define Motor_KeySig 'X'
void* MotorThread();
