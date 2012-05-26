//\\//\\//\\//\\/\\//\\//\\//\\//\\//\\/\\//\\//\\//\\//\\//\\/\\//\\//\\//\\//\\//\\/\\//\\//\\//\\//\\//\\/\\//\\//
// ______________________________________________________________________________________
//                                 nIMU CONTROL
// ______________________________________________________________________________________
// Command Structure sent to nIMU control unit: 
// Z
// s
// If a packet is sent the device will then report its state back.
// Report Packet sent to PC host:
// a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,Z
// 
// 
// a,b,c,d,e,f,g,h = Unsigned Analog values from 0 (0v) to 1024 (5v)
// i = Acceleration X
// j = Acceleration Y
// k = Acceleration Z
// l = Angular Rate X
// m = Angular Rate Y
// n = Angular Rate Z
// o = Magnetometer X
// p = Magnetometer Y
// q = Magnetometer Z
// 
// *NOTE: ALL Values are the raw value from the nIMU and will require conversion.
// (read the nIMU document for the exact numbers and binary data formula.)
// 
#define nIMU_KeySig 'Z'
void* nIMUThread(void *arg);
