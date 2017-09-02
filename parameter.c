
int TAIL_ANGLE_STAND_UP = 98; /* 完全停止時の角度[度]     */
int TAIL_ANGLE_RUN      =  3; /* バランス走行時の角度[度] */
int TAIL_ANGLE_DANSA    = 85; /* 完全停止時の角度[度]     */
int TAIL_ANGLE_LUG      = 75; /* 3点移動時の角度[度]      */
int TAIL_ANGLE_GARAGE   = 94; /* 完全停止時の角度[度]     */

float dT_100ms = 0.1;
float dT_4ms   = 0.004;


float PAI         = 3.1472;
float RAD_1_DEG   = 0.0175; //deg@1rad 
float RAD_5_DEG   = 0.0873; //
float MINUS_RAD_5_DEG   = -0.0873; //
float RAD_90_DEG  = 1.5708; //
float RAD_120_DEG = 2.0944; //
float RAD_315_DEG = 5.4978; //
float RAD_345_DEG = 6.0214; //
float RAD_360_DEG = 6.2832; //
float RAD_450_DEG = 7.8540;

float WheelDiameter = 79.95;  //背面から見て左タイヤの直径[mm] 0817 tada
float WHEEL_R       = 39.975; //Wheel radius
int   RoboTread      = 160; //トレッド長さ[mm]

float FINAL_STRAIGHT_LENGTH =  1100;
float DEAD_ZONE_LENGTH      =  600;
float STEP_TO_GARAGE_LENGTH =  900;

//Strategy_Det.o
//x_left, x_right, y_under, y_top

float LineTrace1Area[4]={0, 0, 0, 0};
float MapTraceArea[4]={0, 0, 0, 0};

float StartArea[4]       = {-200,  200, -200,  500};
float First_Straight[4]  = {-200,  200,  500, 2000};
float First_Corner[4]    = {-200, 2000, 2000, 3500};
float Second_Corner[4]   = {-200, 2000, -200, 2000};
float Second_Straight[4] = {2000, 3000, 1000, 2000};
float GoalArea[4]        = {3000, 4000, 1000, 2000};
float Goal_to_Step[4]    = {4000, 6000, 1000, 2000};
float StepArea[4]        = {4000, 6000, 2000, 5000};

float LookUpGateArea[4]={0, 0, 0, 0};
float GarageArea[4]={0, 0, 0, 0};
float StopArea[4]={0, 0, 0, 0};

float Gate12Area[4]={0, 0, 0, 0};
float Gate23Area[4]={0, 0, 0, 0};
float Gate34Area[4]={0, 0, 0, 0};
float Gate45Area[4]={0, 0, 0, 0};
float Gate56Area[4]={0, 0, 0, 0};
float Gate67Area[4]={0, 0, 0, 0};
float Gate78Area[4]={0, 0, 0, 0};
float Gate89Area[4]={0, 0, 0, 0};

