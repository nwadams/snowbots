/* The sonar control requires the defined macros in robot_config.h and 
furious_control.h to operate */

#define FURIOUS_NO_SONAR_NOOP 0

//Assume robot is facing north for these contants
#define SONAR_LOCPROXNW 1
#define SONAR_LOCPROXN 2
#define SONAR_LOCPROXNE 3
#define SONAR_LOCPROXW 4
#define SONAR_LOCPROXE 5
#define SONAR_LOCPROXSE 6
#define SONAR_LOCPROXS 7
#define SONAR_LOCPROXSW 8


//setup
void Init_Sonar_Prox(void);	//reset the sonar sensors to default state
double Get_Sonar_Distance(int indexid);			//distance in meters
void Set_Sonar_Data(int rawdata, char indexid);	//raw data is passed in
void Print_Sonar_Data(void);


