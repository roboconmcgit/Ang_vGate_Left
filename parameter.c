
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


//Strategy_Det.o
//x_left, x_right, y_under, y_top
float MapTraceArea4[4]={936.52, 1592.76, 1344.29, 2523.54};


float Gate45Area[4]={936.52, 1592.76, 1344.29, 2523.54};
