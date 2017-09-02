/*
#ifdef __cplusplus
extern "C" {
#endif

#define dT_100ms             0.1 //タスク周期[s]

#ifdef __cplusplus
}
#endif
*/

extern int TAIL_ANGLE_STAND_UP;  /* 完全停止時の角度[度] */
extern int TAIL_ANGLE_RUN;     /* バランス走行時の角度[度] */
extern int TAIL_ANGLE_DANSA; /* 完全停止時の角度[度] */
extern int TAIL_ANGLE_LUG; /* 3点移動時の角度[度] */
extern int TAIL_ANGLE_GARAGE; /* 完全停止時の角度[度] */

extern float dT_100ms;
extern float dT_4ms;

extern float PAI;
extern float RAD_315_DEG;
extern float RAD_1_DEG; //deg@1rad 
extern float RAD_5_DEG; //
extern float MINUS_RAD_5_DEG; //
extern float RAD_90_DEG;
extern float RAD_120_DEG;
extern float RAD_315_DEG;
extern float RAD_345_DEG;
extern float RAD_360_DEG;
extern float RAD_450_DEG;

extern float WheelDiameter; //背面から見てタイヤの直径[mm]
extern float WHEEL_R;       //radius of wheel[mm]
extern int   RoboTread;      //トレッド長さ[mm]

extern float FINAL_STRAIGHT_LENGTH;
extern float DEAD_ZONE_LENGTH;
extern float STEP_TO_GARAGE_LENGTH;

//Strategy_Det.o

extern float LineTrace1Area[4];
extern float MapTraceArea1[4];
extern float MapTraceArea2[4];
extern float MapTraceArea3[4];
extern float MapTraceArea4[4];
extern float MapTraceArea5[4];
extern float MapTraceArea6[4];
extern float MapTraceArea7[4];
extern float MapTraceArea8[4];

extern float StartArea[4];
extern float First_Straight[4];
extern float First_Corner[4];
extern float Second_Straight[4];
extern float Second_Corner[4];

extern float GoalArea[4];
extern float Goal_to_Step[4];
extern float StepArea[4];
extern float LookUpGateArea[4];
extern float GarageArea[4];
extern float StopArea[4];

extern float Gate12Area[4];
extern float Gate23Area[4];
extern float Gate34Area[4];
extern float Gate45Area[4];
extern float Gate56Area[4];
extern float Gate67Area[4];
extern float Gate78Area[4];
extern float Gate89Area[4];

