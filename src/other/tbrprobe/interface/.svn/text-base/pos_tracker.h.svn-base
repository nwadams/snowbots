//The speedometer/odometer requires the defined macros in robot_config.h to operate

//setup
void Init_POS_Tracker(void);	//reset the tracker to default ready state
//poll
void Update_Odometer(int rots);	//Speedometer data is derived for time-stamps between samples
//get results
double Get_Distance(void);		//Result in Meters
double Get_Velocity(void);		//Result in Meters Per Second
double Get_Acceleration(void);	//Result in Meters Per Second Per Second
double Get_Jerk(void);		//Result in Meters Per Second Per Second Per Second

//get/set control of ESC speed
void Set_Throttle_Setting(int rawdata);		//correct setting for abtract control of ESC device -100% to 0 to 100%
int Get_Throttle_Setting(void);		//Result in servo corrected setting to send
