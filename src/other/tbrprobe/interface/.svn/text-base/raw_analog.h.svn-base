/* This requires the defined macros in robot_config.h and 
furious_control.h to operate */

//Assume robot has these setup with IR sensors
#define AD0 0
#define AD1 1
#define AD2 2

//for whatever...
#define AD3 3
#define AD4 4

//Matt's pause switch is assigned here by convention
#define AD5 5

//setup
void Init_Raw_Analog(void);	//reset the sensors to default state
double Get_Raw_Analog(int indexid);
void Update_Raw_Analog(int rawdata, int indexid);	//raw data is passed in
