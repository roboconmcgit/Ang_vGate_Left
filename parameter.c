#define OTA_ROBO
//define TADA_ROBO
//#define TOMY_ROBO


//Parameter of Robo
int TAIL_ANGLE_STAND_UP = 98; /* 完全停止時の角度[度]     */
int TAIL_ANGLE_RUN      =  3; /* バランス走行時の角度[度] */
int TAIL_ANGLE_DANSA    = 85; /* 完全停止時の角度[度]     */
int TAIL_ANGLE_LUG      = 75; /* 3点移動時の角度[度]      */
int TAIL_ANGLE_GARAGE   = 94; /* 完全停止時の角度[度]     */

float WheelDiameter = 79.95;  //背面から見て左タイヤの直径[mm] 0817 tada
float WHEEL_R       = 39.975; //Wheel radius
int   RoboTread     = 160; //トレッド長さ[mm]

/* fail rate 20%
float START_ROBO_FORWARD_VAL = 100;
float START_FORWARD_STEP     = 0.1;
*/

/* fail rate under 10%
float START_ROBO_FORWARD_VAL = 50;
float START_FORWARD_STEP     = 0.1;
*/
/* too slow
float START_ROBO_FORWARD_VAL = 25;
float START_FORWARD_STEP     = 0.1;
*/
float START_ROBO_FORWARD_VAL = 40;
float START_FORWARD_STEP     = 0.1;

int   CL_SNSR_GAIN_GRAY = 2;


//Parameter of time length unit
float dT_100ms = 0.1;
float dT_4ms   = 0.004;

float PAI         =  3.1472;
float FIVE_PAI    = 15.708;

float RAD_1_DEG   = 0.0175; //deg@1rad 
float RAD_5_DEG   = 0.0873; //
float RAD_15_DEG  = 0.2618; //
float RAD_30_DEG  = 0.5236; //
float RAD_45_DEG  = 0.7854; //

float MINUS_RAD_5_DEG  = -0.0873; //
float MINUS_RAD_15_DEG = -0.2618; //
float MINUS_RAD_30_DEG = -0.5236; //

float RAD_89_DEG   = 1.5533; //
float RAD_88p5_DEG = 1.5446; //
float RAD_87_DEG   = 1.5184; //
float RAD_90_DEG   = 1.5708; //
float RAD_120_DEG  = 2.0944; //
float RAD_315_DEG  = 5.4978; //
float RAD_345_DEG  = 6.0214; //
float RAD_360_DEG  = 6.2832; //
float RAD_450_DEG  = 7.8540;

//Parameter of Course
float FINAL_STRAIGHT_LENGTH = 1100.0;
//float DEAD_ZONE_LENGTH      =  500.0; //0929 tada
//float DEAD_ZONE_LENGTH      =  700.0; //0929 tada
float DEAD_ZONE_LENGTH      =  600.0; //1008 ota

float DEAD_ZONE_ANGLE       = 0.5; //[rad] 0929 tada Heading Direction while Dead Zone (Less than 1.0)
float LOST_ANGLE_UP         = 2.1; //[rad] 0929 tada Miss Course Desition Angle  Upper
float LOST_ANGLE_LO         = -0.2; //[rad] 0929 tada Miss Course Desition Angle Lower
float LOST_ANGLE_THS        = 0.05; //[rad] 0929 tada 
float LOST_RECOV_LENGTH_1   = 400.0; //[mm] 0929 tada
float LOST_RECOV_LENGTH_2   = 500.0; //[mm] 0929 tada
float RTN_DET_ANGLE         = 1.5; //[rad] 0930 tada Return to Line detect absolute angle

//Parameter of Step
//float STEP_START_LENGTH   = 400;
//float STEP_START_LENGTH   = 550;
//float STEP_START_LENGTH     = 500;
float STEP_START_LENGTH     = 200;

//float FST_DANSA_POS       = 260;
float FST_DANSA_POS         = 270;
float SCD_DANSA_POS         = 200;
//float SCD_DANSA_ON_POS    =  60;
float SCD_DANSA_ON_POS      =  70;

//int   STEP_CLIMB_MAX_SPEED  = 35;
//int   STEP_CLIMB_MAX_SPEED    = 30;
//int   STEP_CLIMB_MAX_SPEED    = 25;
int   STEP_CLIMB_MAX_SPEED    = 20;

//int   STBL_CNT_1st_DANSA    = 750;
int   STBL_CNT_1st_DANSA      = 200;
//int   STBL_CNT_2_DANSA      = 750;
//int   STBL_CNT_2nd_DANSA    =  50;
int   STBL_CNT_2nd_DANSA      = 400;
int   STBL_CNT_2nd_DANSA_ON   = 400;



//Parameter of Garage
//float STEP_TO_GARAGE_LENGTH = 1100;
float STEP_TO_GARAGE_LENGTH = 11100;

//float GARAGE_X_POS          = 1100;
//float GARAGE_X_POS          = 1000;
//float GARAGE_X_POS          = 980;
float GARAGE_X_POS          = 950;

//float GARAGE_LENGTH         =  150;
//float GARAGE_LENGTH         =  50;
float GARAGE_LENGTH         =  70;

//Parameter of Area

//x_left, x_right, y_under, y_top
/*ライン（攻）
float LineTrace1Area[4]={0.0, 936.52, 0.0, 2000.0};
float MapTraceArea1[4]={0.0, 936.52, 2000.0, 2558.54};
float MapTraceArea2[4]={0.0, 2100.0, 2558.54, 3800.0};
float MapTraceArea3[4]={936.52, 2100.0, 2523.54, 2558.54};
float MapTraceArea4[4]={936.52, 1592.76, 1344.29, 2523.54};
float MapTraceArea5[4]={936.52, 1592.76, 0.0, 1344.29};
float MapTraceArea6[4]={1592.76, 2487.29, 0.0, 2523.54};
float MapTraceArea7[4]={2487.29, 3024.44, 0.0, 2523.54};
float MapTraceArea8[4]={3024.44, 4200.0, 0.0, 2523.54};
*/
/* ライン（安）１
float LineTrace1Area[4]={0.0, 936.52, 0.0, 2000.0};
float MapTraceArea1[4]={0.0, 936.52, 2000.0, 2558.54};
float MapTraceArea2[4]={0.0, 2100.0, 2558.54, 3800.0};
float MapTraceArea3[4]={936.52, 2100.0, 2497.44, 2558.54};
float MapTraceArea4[4]={936.52, 1601.26, 1290.24, 2497.44};
float MapTraceArea5[4]={936.52, 1601.26, 0.0, 1290.24};
float MapTraceArea6[4]={1601.26, 2487.29, 0.0, 2497.44};
float MapTraceArea7[4]={2487.29, 3302.03, 0.0, 2497.44};
float MapTraceArea8[4]={3302.03, 4200.0, 0.0, 2497.44};
*/
///*
float LineTrace1Area[4]={  0.0,   936.52,    0.0,  2000.0};

//*/
float StartArea[4]       = {-200.0,  200.0, -200.0,  500.0};
float First_Straight[4]  = {-200.0,  200.0,  500.0, 2000.0};
float First_Corner[4]    = {-200.0, 2000.0, 2000.0, 3500.0};
float Second_Corner[4]   = {-200.0, 2000.0, -200.0, 2000.0};
float Second_Straight[4] = {2000.0, 3000.0, 1000.0, 2000.0};
float GoalArea[4]        = {3000.0, 4000.0, 1000.0, 2000.0};
float Goal_to_Step[4]    = {4000.0, 6000.0, 1000.0, 2000.0};
float StepArea[4]        = {4000.0, 6000.0, 2000.0, 5000.0};

float LookUpGateArea[4]={0.0, 0.0, 0.0, 0.0};
float GarageArea[4]={0.0, 0.0, 0.0, 0.0};
float StopArea[4]={0.0, 0.0, 0.0, 0.0};

/* ライン（攻）
float Gate12Area[4]={0.0, 936.52, 2000.0, 2558.54};
float Gate23Area[4]={0.0, 2100.0, 2558.54, 3800.0};
float Gate34Area[4]={936.52, 2100.0, 2523.54, 2558.54};
float Gate45Area[4]={936.52, 1592.76, 1344.29, 2523.54};
float Gate56Area[4]={936.52, 1592.76, 0.0, 1344.29};
float Gate67Area[4]={1592.76, 2487.29, 0.0, 2523.54};
float Gate78Area[4]={2487.29, 3024.44, 0.0, 2523.54};
float Gate89Area[4]={3024.44, 4200.0, 0.0, 2523.54};
*/
/* ライン（安）１
float Gate12Area[4]={0.0, 936.52, 2000.0, 2558.54};
float Gate23Area[4]={0.0, 2100.0, 2558.54, 3800.0};
float Gate34Area[4]={936.52, 2100.0, 2497.44, 2558.54};
float Gate45Area[4]={936.52, 1601.26, 1290.24, 2497.44};
float Gate56Area[4]={936.52, 1601.26, 0.0, 1290.24};
float Gate67Area[4]={1601.26, 2487.29, 0.0, 2497.44};
float Gate78Area[4]={2487.29, 3302.03, 0.0, 2497.44};
float Gate89Area[4]={3302.03, 4200.0, 0.0, 2497.44};
*/
///*

#ifdef TADA_ROBO
float MapTraceArea1[4]={   0.0,   936.52, 2000.0,  2558.54};
float MapTraceArea2[4]={   0.0,  2100.0,  2558.54, 3800.0};
float MapTraceArea3[4]={ 936.52, 2100.0,  2497.44, 2558.54};
float MapTraceArea4[4]={ 936.52, 1614.4,  1290.24, 2497.44};
float MapTraceArea5[4]={ 936.52, 1614.4,     0.0,  1290.24};
float MapTraceArea6[4]={1614.4,  2255.65,    0.0,  2497.44};
float MapTraceArea7[4]={2255.65, 3202.03,    0.0,  2497.44};
float MapTraceArea8[4]={3202.03, 4200.0,     0.0,  2497.44};

float Gate12Area[4]   ={   0.0,   936.52, 2000.0,  2558.54};
float Gate23Area[4]   ={   0.0,  2100.0,  2558.54, 3800.0};
float Gate34Area[4]   ={ 936.52, 2100.0,  2497.44, 2558.54};
float Gate45Area[4]   ={ 936.52, 1614.4,  1290.24, 2497.44};
float Gate56Area[4]   ={ 936.52, 1614.4,     0.0,  1290.24};
float Gate67Area[4]   ={1614.4,  2255.65,    0.0,  2497.44};
float Gate78Area[4]   ={2255.65, 3202.03,    0.0,  2497.44};
float Gate89Area[4]   ={3202.03, 4200.0,     0.0,  2497.44};
#endif

#ifdef OTA_ROBO
float MapTraceArea1[4]={   0.0,   936.52, 2000.0,  2558.54};
float MapTraceArea2[4]={   0.0,  2100.0,  2558.54, 3800.0};
float MapTraceArea3[4]={ 936.52, 2100.0,  2497.44, 2558.54};

/*
float MapTraceArea4[4]={ 936.52, 1614.4,  1290.24, 2497.44};
float MapTraceArea5[4]={ 936.52, 1614.4,     0.0,  1290.24};
float MapTraceArea6[4]={1614.4,  2255.65,    0.0,  2497.44};
*/
float MapTraceArea4[4]={ 936.52, 1564.4,  1290.24, 2497.44};
float MapTraceArea5[4]={ 936.52, 1564.4,     0.0,  1290.24};
float MapTraceArea6[4]={1564.4,  2255.65,    0.0,  2497.44};


float MapTraceArea7[4]={2255.65, 3202.03,    0.0,  2497.44};
float MapTraceArea8[4]={3202.03, 4200.0,     0.0,  2497.44};

float Gate12Area[4]   ={   0.0,   936.52, 2000.0,  2558.54};
float Gate23Area[4]   ={   0.0,  2100.0,  2558.54, 3800.0};
float Gate34Area[4]   ={ 936.52, 2100.0,  2497.44, 2558.54};
/*
float Gate45Area[4]   ={ 936.52, 1614.4,  1290.24, 2497.44};
float Gate56Area[4]   ={ 936.52, 1614.4,     0.0,  1290.24};
float Gate67Area[4]   ={1614.4,  2255.65,    0.0,  2497.44};
*/

float Gate45Area[4]   ={ 936.52, 1564.4,  1290.24, 2497.44};
float Gate56Area[4]   ={ 936.52, 1564.4,     0.0,  1290.24};
float Gate67Area[4]   ={1564.4,  2255.65,    0.0,  2497.44};

float Gate78Area[4]   ={2255.65, 3202.03,    0.0,  2497.44};
float Gate89Area[4]   ={3202.03, 4200.0,     0.0,  2497.44};
#endif
