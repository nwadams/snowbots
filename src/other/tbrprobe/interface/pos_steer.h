//The steer control requires the defined macros in robot_config.h to operate

//setup
void Init_POS_Steer(void);	//reset the steering to default ready state

int Get_Steer_Angle(void);		//correct setting for control of Servo Steer device
void Set_Steer_Angle(int rawdata);		//setting abtract control of  Servo Steer device -50% to 0 to 50%
