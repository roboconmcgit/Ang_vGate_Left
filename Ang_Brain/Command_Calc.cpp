#include "Command_Calc.h"
#include "ev3api.h"
#include "Clock.h"
#include "math.h"

using ev3api::Clock;

#define liting_radius 10; // liting spot radius [mm]
//#define STEP_DEBUG

Clock*       gClock;

CommandCalc::CommandCalc(){

}

void CommandCalc::init( ){

  Track_Mode = Start_to_1st_Corner;
#ifdef STEP_DEBUG
  Track_Mode = Return_to_Line;
  //  Track_Mode = Go_Step;
#endif
  Step_Mode  = Step_Start;
  gClock       = new Clock();
}

void CommandCalc::SetCurrentData(int   linevalue,
				 float xvalue,
				 float yvalue,
				 float odo,                     
				 float speed,
				 float yawrate,
				 float yawangle,
				 int   robo_tail_angle,
				 bool  robo_stop,
				 bool  robo_forward,
				 bool  robo_back,
				 bool  robo_turn_left,
				 bool  robo_turn_right,
				 bool  dansa,
				 bool  robo_balance_mode,
				 int   max_forward,
				 float max_yawrate,
				 float min_yawrate

				 ) {

    mLinevalue         = linevalue;
    mXvalue            = xvalue;
    mYvalue            = yvalue;
    mOdo               = odo;
    mSpeed             = speed;
    mYawrate           = yawrate;
    mYawangle          = yawangle;
    mTail_angle        = robo_tail_angle;
    mRobo_stop         = robo_stop;
    mRobo_forward      = robo_forward;
    mRobo_back         = robo_back;
    mRobo_turn_left    = robo_turn_left;
    mRobo_turn_right   = robo_turn_right;
    mDansa             = dansa;
    mRobo_balance_mode = robo_balance_mode;

    mMax_Forward = max_forward;
    mMax_Yawrate = max_yawrate;
    mMin_Yawrate = min_yawrate;

}

void CommandCalc::Track_run( ) {

  static float ref_odo;
  int dammy_line_value;

  switch(Track_Mode){
  case Start_to_1st_Corner:
    forward = mMax_Forward;
    LineTracerYawrate(mLinevalue);
    anglecommand = TAIL_ANGLE_RUN; //0817 tada
    tail_mode_lflag = false;

    if(mYawangle < -2){
      Track_Mode = Snd_Corner;
    }
    break;

  case Snd_Corner:
    forward =  mMax_Forward;
    LineTracerYawrate(mLinevalue);
    anglecommand = TAIL_ANGLE_RUN; //0817 tada
    tail_mode_lflag = false;

    if(mYawangle > 2){
      Track_Mode = Final_Corner;
    }
    break;
    
  case Final_Corner:
    forward =  mMax_Forward;
    LineTracerYawrate(mLinevalue);
    anglecommand = TAIL_ANGLE_RUN; //0817 tada
    tail_mode_lflag = false;

    if((mYawangle < 1) &&(mRobo_forward == 1)){
      Track_Mode = Final_Straight;
      ref_odo = mOdo +  FINAL_STRAIGHT_LENGTH;
    }
    break;

  case Final_Straight:
    forward =  mMax_Forward;
    LineTracerYawrate(mLinevalue);
    anglecommand = TAIL_ANGLE_RUN; //0817 tada
    tail_mode_lflag = false;
    
    if(mOdo > ref_odo){
      Track_Mode = Dead_Zone;
      ref_odo = mOdo + DEAD_ZONE_LENGTH;
    }

    break;

  case Dead_Zone:
    forward =  50;
    dammy_line_value = 50 - 300*mYawangle;
    if(dammy_line_value > 100){
      dammy_line_value = 100;
    }else if(dammy_line_value < 0){
      dammy_line_value = 0;
    }

    LineTracerYawrate(dammy_line_value);

    if(mOdo > ref_odo){
      Track_Mode = Return_to_Line;
    }
    break;

  case Return_to_Line:
    forward =  50;
    LineTracerYawrate((2*mLinevalue));
    anglecommand = TAIL_ANGLE_RUN; //0817 tada
    tail_mode_lflag = false;

    if((mYawangle > 1) &&(mRobo_forward == 1)){
      Track_Mode = Go_Step;
    }

    break;

  case Go_Step:
    StepRunner(mLinevalue, mOdo, mYawangle, mDansa);
    ref_odo = mOdo + STEP_TO_GARAGE_LENGTH;
    break;

  case Approach_to_Garage:
    if(mOdo > ref_odo){
      forward =  0;
      yawratecmd = 0;						//目標yawrate値を更新
    }
    forward =  20;
    dammy_line_value = 50 - 300*(mYawangle-PAI);
    if(dammy_line_value > 100){
      dammy_line_value = 100;
    }else if(dammy_line_value < 0){
      dammy_line_value = 0;
    }

    LineTracerYawrate(dammy_line_value);

    break;


  default:
    forward = 0;
    LineTracerYawrate(mLinevalue);
    anglecommand = TAIL_ANGLE_RUN; //0817 tada
    tail_mode_lflag = true;
    break;
  }
}

void CommandCalc::StrategyCalcRun(int strategy_num, int virtualgate_num) {

  Strategy=static_cast<enumStrategy>(strategy_num);

	switch(Strategy){
	case StartDash:
		StartDashRunner();
	break;

	case LineTrace1:
	  forward = mMax_Forward;
	  LineTracerYawrate(mLinevalue);
	  anglecommand = TAIL_ANGLE_RUN; //0817 tada
	  tail_mode_lflag = false;
		
	break;

	case MapTrace:
		//VirtualGateDet();
		MapTracer(virtualgate_num);
	break;

	case Goal:

	break;

	case Goal2Step:

	break;

	case Step:
	  gForward->init_pid(0.1,0.005,0.05,dT_4ms);
	  StepRunner(mLinevalue, mOdo, mYawangle, mDansa);
	break;

	case LookUpGate:
		LookUpGateRunner();
	break;

	case Garage:
		GarageRunner();
	break;

	case Stop:
		StopRobo();
	break;

	default:

	break;
	}

}

void CommandCalc::StartDashRunner(){

}
//17.07.31 k-tomii add for position estimation
//ライントレースプログラム
void CommandCalc::LineTracer(int line_value,float traceforward) {

	const int LineTraceCommand=80;			//目標ライン値
	const float KP=0.5,KI=0.005,KD=0.05;			//PIDゲインの設定

	static int error_old=0,error_P_old=0;		//過去の偏差
	static float u=0;							//制御入力

	float u_delta=0;							//制御入力の差分
	int error=0,error_P=0,error_I=0,error_D=0;	//偏差
	float u_P_delta=0,u_I_delta=0,u_D_delta=0;	//制御入力の差分

	error=LineTraceCommand-(line_value);			//制御偏差を計算
	error_P=error-error_old;					//P制御用の偏差を計算
	error_I=error;								//I制御用の偏差を計算
	error_D=error_P-error_P_old;				//D制御用の偏差を計算

	u_P_delta=KP*error_P;						//P制御用の入力差分を計算
	u_I_delta=KI*error_I;						//I制御用の入力差分を計算
	u_D_delta=KD*error_D;						//D制御用の入力差分を計算

	u_delta=u_P_delta+u_I_delta+u_D_delta;		//PID制御入力の差分を計算
	u=u+u_delta;								//制御入力を計算

	//入力制限
	if(u>100){
		u=100;
	}else if(u<-100){
		u=-100;
	}

	yawratecmd=-u;										//目標turn値を更新
	forward=traceforward;						//目標forward値を更新

	error_old=error;							//過去の偏差を保存
	error_P_old=error_P;						//過去の偏差を保存

}

//2017/08/06多田さんライントレーサー
void CommandCalc::LineTracerYawrate(int line_value) {

    y_t = -1.0*(((float)line_value-50.0)/50.0) * (float)liting_radius;//add(-1*) for Left Edge Trace
    if(y_t > 10.0) y_t = 10.0;
    if(y_t < -10.0) y_t = -10.0;
	y_t = y_t + 7.0*(y_t/8.0)*(y_t/8.0)*(y_t/8.0);
//    yawratecmd = y_t/4.0;
    yawratecmd = (y_t/4.0)*(pg + df*(y_t-y_t_prev));

    if(yawratecmd > mMax_Yawrate){
      yawratecmd =  mMax_Yawrate;

    }else if (yawratecmd < mMin_Yawrate){
      yawratecmd = mMin_Yawrate;
    }else{
      yawratecmd = yawratecmd;
    }

    y_t_prev = y_t;

}

void CommandCalc::MapTracer(int virtualgate_num) {

	VirtualGate=static_cast<enumVirtualGate>(virtualgate_num);

	switch(VirtualGate){
	case Gate12:

	break;

	case Gate23:

	break;

	case Gate34:

	break;

	case Gate45:

	break;

	case Gate56:

	break;

	case Gate67:

	break;

	case Gate78:

	break;

	case Gate89:

	break;

	default:

	break;
	}

}

void CommandCalc::StepRunner(int line_value, float odo, float angle, bool dansa){
  /*前提条件：ロボットがライン上にあること*/

  float y_t;
  static float angle_change_right_edge_trace;
  static float angle_change_left_edge_trace;
  
  static float target_odo;
  static float target_angle;
  static float target_tail_angle;
  static int32_t clock_start;
  
  switch(Step_Mode){

  case Step_Start:
    forward =  50;
    LineTracerYawrate((2*line_value));
    target_odo = odo + 500;
 

    clock_start = gClock->now();
    anglecommand = TAIL_ANGLE_RUN; //0817 tada
    
    Step_Mode = Approach_to_Step;   
    //debug for stand_up
    //  Step_Mode =  First_Dansa;   
    //    target_odo = odo + 250; //for debug

    break;

  case Approach_to_Step:

    if(odo > target_odo){
      forward =  20;
    }else{
      forward =  50;
    }

      LineTracerYawrate((2*line_value));


    if(dansa){
      Step_Mode = First_Dansa;
      target_odo = odo + 250;
    }


    break;

  case Change_Right_Edge_Trace:
    if(angle < (angle_change_right_edge_trace - (PAI/4.0))){
      Step_Mode = Right_Edge_On;
    }
    else{
      y_t = 0.5*(angle - (angle_change_right_edge_trace - (PAI/4.0)));
      forward = 0;
      yawratecmd = y_t;						//目標yawrate値を更新
      //      yawratecmd = -1.0;						//目標yawrate値を更新
    }
    break;

  case Right_Edge_On:
    if(dansa){
      Step_Mode = First_Dansa;
      target_odo = odo + 250;
    }
    else{
      y_t = (((float)line_value-50.0)/50.0) * (float)liting_radius;//add(-1*) for Left Edge Trace
      if(y_t > 10.0) y_t = 10.0;
      if(y_t < -10.0) y_t = -10.0;
  
      yawratecmd = y_t/6.0;						//目標yawrate値を更新
      forward = 10;
    }
    break;

  case First_Dansa:
    if(odo > target_odo){
      forward = 0;
      yawratecmd = 0;
      target_tail_angle =  TAIL_ANGLE_RUN;
      clock_start = gClock->now();
      Step_Mode = First_Dansa_On;
      gStep->SetInitPIDGain(0.1,0.005,0.05,dT_4ms);
    }else{
      forward    = 20;
      yawratecmd = 0;

    }
    break;

  case First_Dansa_On:
    //    gStep->SetInitPIDGain(0.1,0.005,0.05,dT_4ms);
    forward = gStep->CalcPIDContrInput(target_odo, odo);
    forward = forward * 0.1;
    yawratecmd = 0;
    anglecommand = target_tail_angle;

    if(target_tail_angle < TAIL_ANGLE_DANSA){
      target_tail_angle = target_tail_angle + 0.1;
    }
    if((gClock->now() - clock_start) > 5000){
      Step_Mode = First_Dansa_Tail_On;
      clock_start = gClock->now();
    }

  break;

  case First_Dansa_Tail_On:
    forward = -10;
    yawratecmd = 0;
    if((gClock->now() - clock_start) > 2000){
      forward = 0;
      yawratecmd = 0;
      tail_mode_lflag = true;
      Step_Mode = First_Turn;
      clock_start = gClock->now();
      target_angle = angle + RAD_360_DEG;
    }
    
    break;

  case First_Turn:
    if((gClock->now() - clock_start) > 1000){
      if(angle >= target_angle){
	Step_Mode = First_Dansa_Stand_Up;
	clock_start = gClock->now();
	forward = 0;
	yawratecmd = 0;
	
      }else{
	forward = 0;
	//      y_t = -0.5*(target_angle - angle);
	y_t = -1.5;
	yawratecmd = y_t;						//目標yawrate値を更新
      }						//目標yawrate値を更新
    }
    else{
      forward = 0;
      yawratecmd = 0;
    }
    break;

  case First_Dansa_Stand_Up:
    if((gClock->now() - clock_start) > 1000){

      if(mTail_angle > 98){	
	forward = 0;
	yawratecmd = 0;
	tail_mode_lflag = false;
	anglecommand = TAIL_ANGLE_RUN;
	Step_Mode = Approach_to_2nd_Step;
	clock_start = gClock->now();
      }else if(target_tail_angle < 120){
	target_tail_angle = target_tail_angle + 0.01;
	anglecommand = target_tail_angle;
      }


    }else{
      forward = 0;
      yawratecmd = 0;
    }
  
    break;

  case Approach_to_2nd_Step:
    

    if((gClock->now() - clock_start) < 5000){
      forward = 0;
      yawratecmd = 0.0;
      anglecommand = TAIL_ANGLE_RUN;
    }else{
      forward = 10;
      y_t = (-1.0)*(((float)line_value-50.0)/50.0) * (float)liting_radius;//add(-1*) for Left Edge Trace
      if(y_t > 10.0) y_t = 10.0;
      if(y_t < -10.0) y_t = -10.0;
      yawratecmd = y_t/6.0;						//目標yawrate値を更新

      if(dansa){
	Step_Mode = Second_Dansa;
	target_odo = odo + 250;
      }
    }

    break;
    
  case Second_Dansa:
    if(odo > target_odo){
      forward = 0;
      yawratecmd = 0;
      target_tail_angle =  TAIL_ANGLE_RUN;
      clock_start = gClock->now();
      Step_Mode = Second_Dansa_On;
    }else{
      forward    = 15;
      yawratecmd = 0;
    }
    break;


  case Second_Dansa_On:
    gStep->SetInitPIDGain(0.1,0.005,0.05,dT_4ms);
    forward = gStep->CalcPIDContrInput(target_odo, odo);
    forward = forward * 0.1;
    yawratecmd = 0;
    anglecommand = target_tail_angle;

    if(target_tail_angle < TAIL_ANGLE_DANSA){
      target_tail_angle = target_tail_angle + 0.1;
    }
    if((gClock->now() - clock_start) > 5000){
      Step_Mode = Second_Dansa_Tail_On;
      clock_start = gClock->now();
    }

  break;

  case Second_Dansa_Tail_On:
    forward = -10;
    yawratecmd = 0;
    if((gClock->now() - clock_start) > 2000){
      forward = 0;
      yawratecmd = 0;
      tail_mode_lflag = true;
      Step_Mode = Second_Turn;
      clock_start = gClock->now();
      //      target_angle = angle + RAD_360_DEG;
      target_angle = angle + RAD_450_DEG;
    }
    
    break;

  case Second_Turn:
    if((gClock->now() - clock_start) > 1000){
      if(angle >= target_angle){
	Step_Mode = Second_Dansa_Stand_Up;
	target_odo = odo + 50;
	clock_start = gClock->now();
	forward = 0;
	yawratecmd = 0;
	
      }else{
	forward = 0;
	y_t = -0.5;
	yawratecmd = y_t;						//目標yawrate値を更新
      }						//目標yawrate値を更新
    }
    else{
      forward = 0;
      yawratecmd = 0;
    }
    break;

  case Second_Dansa_Stand_Up:
    if((gClock->now() - clock_start) > 1000){

      if(mTail_angle > 98){	
	forward = 0;
	yawratecmd = 0;
	tail_mode_lflag = false;
	anglecommand = TAIL_ANGLE_RUN;
	Step_Mode = Approach_to_Exit;  
	clock_start = gClock->now();
      }else if(target_tail_angle < 120){
	forward = 0;
	target_tail_angle = target_tail_angle + 0.01;
	anglecommand = target_tail_angle;
      }





    }else{
      forward = 0;
      yawratecmd = 0;
    }


    break;

  case Approach_to_Exit:
       
    /*
    y_t = (((float)line_value-50.0)/50.0) * (float)liting_radius;//add(-1*) for Left Edge Trace
    if(y_t > 10.0) y_t = 10.0;
    if(y_t < -10.0) y_t = -10.0;
    yawratecmd = y_t/6.0;						//目標yawrate値を更新
    */
    if((gClock->now() - clock_start) < 5000){
      forward = 0;
      yawratecmd = 0;						//目標yawrate値を更新
    }else{
      forward = 10;
      if(dansa){
	Track_Mode = Approach_to_Garage;
      }
    }
    break;


  case Change_Left_Edge_Trace:
    if(angle > (angle_change_left_edge_trace + (PAI/4.0))){
      Step_Mode = End;
    }
    else{
      y_t = -0.5*((angle_change_left_edge_trace - (PAI/4.0))-angle);
      forward = 0;
      yawratecmd = y_t;						//目標yawrate値を更新
      //      yawratecmd = -1.0;						//目標yawrate値を更新
    }
    break;

  default:
    y_t = -1.0*(((float)line_value-50.0)/50.0) * (float)liting_radius;//add(-1*) for Left Edge Trace
    if(y_t > 10.0) y_t = 10.0;
    if(y_t < -10.0) y_t = -10.0;
  
    yawratecmd = y_t/6.0;						//目標yawrate値を更新
    forward = 20;
    
    break;
  }

}

void CommandCalc::LookUpGateRunner(){

}

void CommandCalc::GarageRunner(){

}

void CommandCalc::StopRobo(){

}
