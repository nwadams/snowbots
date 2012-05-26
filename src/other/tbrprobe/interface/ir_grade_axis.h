//The IR grade system 
//angles in degreees:
//#define IR_distances_top_angle_center_deviation_Theta 30.0
//#define IR_distances_top_angle_center_deviation_Theta_Not 30.0
//#define IR_distances_vertical_angle_difference_Ef 15.0

//setup
void Init_IR_Grade(void);
//poll to calculate new axis
void Update_IR_Grade(int Front_Mid, int Front_Left, int Front_Right, int Rear_Mid ); //IR data is derived for a polled sample
//Zero sensors to flat ground
void Set_IR_Grade_Zero_Setting(int Front_Mid, int Front_Left, int Front_Right, int Rear_Mid );
//angles in degrees
int Get_ROLL(void);
int Get_YAW(void);
int Get_PITCH(void);
int Get_Barrier(void);
//linear distance in cm
int Get_IR_Distance_Mid(void);
int Get_IR_Distance_Left(void);
int Get_IR_Distance_Right(void);
int Get_IR_Distance_Rear(void);
