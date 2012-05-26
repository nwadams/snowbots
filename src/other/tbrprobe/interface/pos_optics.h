//The turret requires the defined macros in robot_config.h to operate

//setup
void Init_Optics(void);	//reset the tracker to default ready state
 
void Set_Optics_Setting(int rawdata);  //set abstract positon 
int Get_Optics_Setting(void); //Result in servo corrected setting to send
