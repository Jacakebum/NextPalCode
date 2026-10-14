#include "tim.h"
#include "main.h"
#include "stdio.h"
#include "gpio.h"

#include "Inverse_K.h"
#include "math.h"
#include "Task_StateMachine.h"

#define Trot  1
#define Walk  2
#define Crawl 3

#define FORWARD_RIGHT     htim1
#define FORWARD_LEFT      htim2
#define BACKWARD_RIGHT    htim3
#define BACKWARD_LEFT     htim4

#define HIP     TIM_CHANNEL_2
#define THIGH   TIM_CHANNEL_3
#define SHANK   TIM_CHANNEL_4

#define HEAD_PITCH_TIM     htim3
#define HEAD_ROLL_TIM      htim4
#define HEAD_PITCH_CH      TIM_CHANNEL_1
#define HEAD_ROLL_CH       TIM_CHANNEL_1
#define TAIL_TIM           htim2   /* PA0 = TIM2_CH1 = TAIL-LR (硬件 PWM) */
#define TAIL_CH            TIM_CHANNEL_1

/* Head-YAW (PA15) 和 MOUTH (PA8) 走软件 PWM（TIM6 中断位拆） */

#define FL_HIP_Value    145+10+90/*加号表示前向后逆时针*/
#define FL_THIGH_Value  145+8  /*加号表示往上抬*/
#define FL_SHANK_Value  180+10
#define FR_HIP_Value    150-85/*加号表示前向后逆时针*/
#define FR_THIGH_Value  150-30 /*减号表示左向右看顺时针*/
#define FR_SHANK_Value  180+20 /*加号表示往下压*/
#define BL_HIP_Value    100+0 /*加号表示前向后顺时针*/
#define BL_THIGH_Value  120+5
#define BL_SHANK_Value  150
#define BR_HIP_Value    200
#define BR_THIGH_Value  145-3
#define BR_SHANK_Value  150+12


/*单腿关节转角*/
extern floatTheta Leg_angle[4];

/*陀螺仪解算出的欧拉角*/
extern float angle[3];

/*状态机全局状态（定义在 Task_StateMachine.c）*/
extern MainState_t Mainstate;

/*舵机平滑跟随增益（默认 LEG_PWM_SMOOTH_GAIN；特定动作可临时调大以提高跟踪速度）*/
float servo_smooth_gain = 0.18f;   /* 腿部舵机平滑增益，比原0.28更柔更慢 */

/*==================== 软件 PWM 模块 ====================*/
/* TIM6 每 20us 中断一次，1000 次 = 20ms = 50Hz 舵机帧周期
   驱动 PA8 (MOUTH) 和 PA15 (Head-YAW) 两根不能用硬件 PWM 的舵机
   脉宽范围 50~100 ticks = 1ms~2ms，对应角度 0~180 度 */
#define SW_PWM_PERIOD    1000    /* 20ms / 20us = 1000 ticks */
#define SW_PWM_MIN_TICKS 50      /* 1ms = 50 ticks */
#define SW_PWM_MAX_TICKS 100     /* 2ms = 100 ticks */

volatile uint16_t sw_pwm_counter = 0;   /* 0..999 帧内计数 */
volatile uint16_t sw_pwm_mouth  = 75;  /* MOUTH 当前脉宽 ticks (75=1.5ms=90度居中) */
volatile uint16_t sw_pwm_yaw    = 75;  /* Head-YAW 当前脉宽 ticks */

/* TIM6 中断回调：每 20us 调一次，产生 50Hz 软件 PWM */
void Sw_PWM_Tick(void)
{
    if(sw_pwm_counter == 0)
    {
        /* 帧起始：两根线拉高 */
        GPIOA->BSRR = GPIO_PIN_8 | GPIO_PIN_15;
    }
    /* 到各自脉宽阈值则拉低 */
    if(sw_pwm_counter >= sw_pwm_mouth)
        GPIOA->BRR = GPIO_PIN_8;
    if(sw_pwm_counter >= sw_pwm_yaw)
        GPIOA->BRR = GPIO_PIN_15;

    sw_pwm_counter++;
    if(sw_pwm_counter >= SW_PWM_PERIOD)
        sw_pwm_counter = 0;
}

/* 角度 (0~180) 转软件 PWM ticks (50~100) */
static uint16_t Sw_Angle_To_Ticks(float angle_deg)
{
    if(angle_deg < 0.0f) angle_deg = 0.0f;
    if(angle_deg > 180.0f) angle_deg = 180.0f;
    return (uint16_t)(SW_PWM_MIN_TICKS + (angle_deg / 180.0f) * (SW_PWM_MAX_TICKS - SW_PWM_MIN_TICKS));
}

floatXYZ pos;
floatRPY angle_;	
			
static uint8_t Update_Control(void);
static uint8_t Stand(void);
static uint8_t Pos_Attitude_Control(floatXYZ pos,floatRPY angle);
static uint8_t Servo_Control(void);
static uint8_t IMU_Attitude_Control(void);
static float Trans_Angle(float angle);
static float Clamp_PWM(float pwm);
static uint8_t Head_Servo_Update(void);
static float Clamp_Head_Angle(float angle_deg);
static uint16_t Head_Angle_To_PWM(float angle_deg);

#define HEAD_CENTER_DEG            90.0f
#define PITCH_GEAR_GAIN            3.0f   /* 1:3减速，舵机角度放大3倍补偿输出 */
#define NOD_OUTPUT_AMP_DEG         18.0f  /* 头部输出端振幅（度） */
#define SHAKE_OUTPUT_AMP_DEG       25.0f
#define TILT_OUTPUT_AMP_DEG        20.0f
#define NOD_MOTION_STEP            0.06f  /* 点头更慢更柔 */
#define SHAKE_MOTION_STEP          0.10f  /* 摇头更慢 */
#define TILT_MOTION_STEP           0.10f  /* 歪头更慢 */
#define MOUTH_CLOSE_BIAS_DEG       70.0f  /* 嘴巴机械中位修正：输出朝闭嘴方向偏移70度 */
#define HEAD_SMOOTH_GAIN           0.10f  /* 头/尾/嘴舵机平滑跟随增益（越小越柔，比腿的0.28更慢） */
#define TAIL_WAG_MOTION_STEP       0.12f  /* 摇尾巴摆动速度（rad/10ms），越大甩得越急 */

#define LEG_PWM_MIN                50.0f
#define LEG_PWM_MAX                250.0f
#define LEG_PWM_SMOOTH_GAIN        0.28f

#define COG_Y_SHIFT_MAX   15.0f
#define COG_X_SHIFT_MAX   8.0f
#define COG_Z_SHIFT_MAX   2.0f

/* 0:抬起（负Z，机身降低） 1:压低（正Z，机身升高） */
#define COG_Z_RAISE       1

/*步态相关函数*/
static uint8_t Trot_Gait(MainState_t MainState);
static uint8_t Trot_Jump_Action(void);
static uint8_t Jump_Gait(MainState_t MainState);
static uint8_t Wave_Action(void);
/*步态相关函数*/

void Task_Control_Start(void)
{
    Stand();
    TickType_t xLastWakeUpTime;
    xLastWakeUpTime = xTaskGetTickCount();
    while (1)
    {
			  //Stand();
        Update_Control();
        vTaskDelayUntil(&xLastWakeUpTime, 10);
    }
}

uint8_t Update_Control(void)
{
	Head_Servo_Update();
	
	/*扭动身体（IMU自稳也在此步态下触发，见 case MainState_IMU）*/
	if (GetGaitMode()==GaitMode_Body_Twist)
	{
		static float x=0;
	  static float y=0;
	  static float z=150;
	  static float P=0;
	  static float R=0;
	  static float Y=0;
			pos.x=x;
			pos.y=y;
			pos.z=z;
			angle_.P=P;
			angle_.R=R;
			angle_.Y=Y;				
			switch(GetMainState()) 
			{		
			 case MainState_IMU:
			 {	 
				/*IMU自稳（原触发模式）：先下发"13"切到扭动步态，再下发"07"开启，
				  斜面上自动保持机身水平；下发"0"（全回中站立）退出*/
				IMU_Attitude_Control();
				return 1;
			 } 
			 case MainState_Static:
			 {	 
   			 if(z<150) 
					 z++;
				 
         if(y>0) y--;
				 else if(y<0) y++;
				 
         if(x>0) x--;
				 else if(x<0) x++;
				 
				 if(P>0) P=P-0.5;
				 else if(P<0) P=P+0.5;
				 
         if(R>0) R--;
				 else if(R<0) R++;
				 
         if(Y>0) Y--;
				 else if(Y<0) Y++;
			 } 
			 break;
			 case MainState_Step:
			 {				 
				 z--;	
				 if(z<130) z=130;		
			 }				 
			 break;	
			 case MainState_Forward:
			 {				 
				 P=P-0.5;	
				 if(P<-10) P=-10;		
			 }
			 break;
			 case MainState_Backward:
			 {				 
				 P=P+0.5;	
				 if(P>10) P=10;		
			 }		    
			 break;		 
			 case MainState_Move_to_Left:
			 {				 
				 R--;	
				 if(R<-15) R=-15;		
			 }  
			 break;	
			 case MainState_Move_to_Right:
			 {				 
				 R++;	
				 if(R>15) R=15;		
			 }			 
			 break;	
			 case MainState_Turn_Left:
			 {				 
				 Y++;	
				 if(Y>10) Y=10;		
			 }
			 break;		 
			 case MainState_Turn_Right:
			 {				 
				 Y--;	
				 if(Y<-10) Y=-10;		
			 }   
			 break;	
			 default:
			 ;			 
			}		
//printf("%f \t",z);			
			Pos_Attitude_Control(pos,angle_);	 
	}
	
	/*Trot步态*/
	else if (GetGaitMode()==GaitMode_Trot) 
	{
		Trot_Gait(GetMainState());	
	}
	
	/*跳跃步态*/
	else if (GetGaitMode()==GaitMode_Jump)
	{
		Jump_Gait(GetMainState());
	}
	
	/*Stand模式下的打招呼动作*/
	else if (GetGaitMode()==GaitMode_Stand)
	{
		if (GetMainState()==MainState_Wave)
		{
			Wave_Action();
		}
		else
		{
			Stand();
		}
	}
	
	return 1;
}

uint8_t Head_Servo_Update(void)
{
	float yaw = (float)GetHeadYaw();
	float pitch = (float)GetHeadPitch();
	float roll = (float)GetHeadRoll();
	float tail = (float)GetTailLR();
	float mouth = (float)GetMouthOpen();
	HeadMotionMode_t mode = GetHeadMotionMode();
	TailMotionMode_t tail_mode = GetTailMotionMode();

	/* 头/尾/嘴舵机平滑滤波值（每10ms向目标插值，消除僵硬跳变） */
	static float filt_yaw = 90.0f;
	static float filt_pitch = 90.0f;
	static float filt_roll = 90.0f;
	static float filt_tail = 90.0f;
	static float filt_mouth = 90.0f;
	static uint8_t head_pwm_init = 0;

	static float phase = 0.0f;
	static float tail_phase = 0.0f;
	const float motion_cycle = 6.2831853f;
	const float nod_servo_amp = NOD_OUTPUT_AMP_DEG * PITCH_GEAR_GAIN;

	/* 手动Pitch以90度为中心放大偏移量，补偿1:3减速 */
	pitch = HEAD_CENTER_DEG + (pitch - HEAD_CENTER_DEG) * PITCH_GEAR_GAIN;

	if(mode == HeadMotion_Nod)
	{
		phase += NOD_MOTION_STEP;
		if(phase > motion_cycle) phase -= motion_cycle;
		pitch += nod_servo_amp * sinf(phase);
	}
	else if(mode == HeadMotion_Shake)
	{
		phase += SHAKE_MOTION_STEP;
		if(phase > motion_cycle) phase -= motion_cycle;
		yaw += SHAKE_OUTPUT_AMP_DEG * sinf(phase);
	}
	else if(mode == HeadMotion_Tilt)
	{
		phase += TILT_MOTION_STEP;
		if(phase > motion_cycle) phase -= motion_cycle;
		roll += TILT_OUTPUT_AMP_DEG * sinf(phase);
	}
	else if(mode == HeadMotion_LookLeft)
	{
		yaw = 180.0f;
		phase = 0.0f;
	}
	else if(mode == HeadMotion_LookRight)
	{
		yaw = 0.0f;
		phase = 0.0f;
	}
	else
	{
		phase = 0.0f;
	}

	/* 摇尾巴：以90°为中位，0°~180°大幅正弦来回摆动（指令"43"开启，发"40"回中停止） */
	if(tail_mode == TailMotion_Wag)
	{
		tail_phase += TAIL_WAG_MOTION_STEP;
		if(tail_phase > motion_cycle) tail_phase -= motion_cycle;
		tail = 90.0f + 90.0f * sinf(tail_phase);
	}
	else
	{
		tail_phase = 0.0f;
	}

	/* 平滑跟随目标（HEAD_SMOOTH_GAIN 越小越柔） */
	if(head_pwm_init == 0)
	{
		filt_yaw = yaw; filt_pitch = pitch; filt_roll = roll;
		filt_tail = tail; filt_mouth = mouth;
		head_pwm_init = 1;
	}
	else
	{
		filt_yaw   += HEAD_SMOOTH_GAIN * (yaw   - filt_yaw);
		filt_pitch += HEAD_SMOOTH_GAIN * (pitch - filt_pitch);
		filt_roll  += HEAD_SMOOTH_GAIN * (roll  - filt_roll);
		filt_tail  += HEAD_SMOOTH_GAIN * (tail  - filt_tail);
		filt_mouth += HEAD_SMOOTH_GAIN * (mouth - filt_mouth);
	}

	/* Head-YAW: 软件 PWM 输出到 PA15 */
	sw_pwm_yaw = Sw_Angle_To_Ticks(filt_yaw);
	/* MOUTH: 软件 PWM 输出到 PA8，朝闭嘴方向偏移 70 度（机械中位修正） */
	sw_pwm_mouth = Sw_Angle_To_Ticks(filt_mouth - MOUTH_CLOSE_BIAS_DEG);
	/* Head-PITCH & Head-ROLL: 硬件 PWM */
	__HAL_TIM_SET_COMPARE(&HEAD_PITCH_TIM, HEAD_PITCH_CH, Head_Angle_To_PWM(filt_pitch));
	__HAL_TIM_SET_COMPARE(&HEAD_ROLL_TIM, HEAD_ROLL_CH, Head_Angle_To_PWM(filt_roll));
	/* TAIL: 硬件 PWM (TIM2_CH1 = PA0) */
	__HAL_TIM_SET_COMPARE(&TAIL_TIM, TAIL_CH, Head_Angle_To_PWM(filt_tail));

	return 1;
}

float Clamp_Head_Angle(float angle_deg)
{
	if(angle_deg < 0.0f)
		return 0.0f;
	if(angle_deg > 180.0f)
		return 180.0f;
	return angle_deg;
}

uint16_t Head_Angle_To_PWM(float angle_deg)
{
	float angle = Clamp_Head_Angle(angle_deg);
	return (uint16_t)(50.0f + (angle / 180.0f) * 200.0f);
}

float Clamp_PWM(float pwm)
{
	if(pwm < LEG_PWM_MIN)
		return LEG_PWM_MIN;
	if(pwm > LEG_PWM_MAX)
		return LEG_PWM_MAX;
	return pwm;
}

uint8_t Stand(void)
{

	__HAL_TIM_SET_COMPARE(&FORWARD_RIGHT, HIP,   FR_HIP_Value);   /* 中值1.5ms/20ms=0.075 0.075*2000ms=150 */
	
	__HAL_TIM_SET_COMPARE(&FORWARD_RIGHT, THIGH, FR_THIGH_Value);
	
	__HAL_TIM_SET_COMPARE(&FORWARD_RIGHT, SHANK, FR_SHANK_Value);
	
	__HAL_TIM_SET_COMPARE(&FORWARD_LEFT, HIP,    FL_HIP_Value);   /* 中值1.5ms/20ms=0.075 0.075*2000ms=150 */
	
	__HAL_TIM_SET_COMPARE(&FORWARD_LEFT, THIGH,  FL_THIGH_Value);
	
	__HAL_TIM_SET_COMPARE(&FORWARD_LEFT, SHANK,  FL_SHANK_Value);	
	
	__HAL_TIM_SET_COMPARE(&BACKWARD_RIGHT, HIP,   BR_HIP_Value);   /* 中值1.5ms/20ms=0.075 0.075*2000ms=150 */
	
	__HAL_TIM_SET_COMPARE(&BACKWARD_RIGHT, THIGH, BR_THIGH_Value);
	
	__HAL_TIM_SET_COMPARE(&BACKWARD_RIGHT, SHANK, BR_SHANK_Value);
	
	__HAL_TIM_SET_COMPARE(&BACKWARD_LEFT, HIP,    BL_HIP_Value);   /* 中值1.5ms/20ms=0.075 0.075*2000ms=150 */
	
	__HAL_TIM_SET_COMPARE(&BACKWARD_LEFT, THIGH,  BL_THIGH_Value);
	
	__HAL_TIM_SET_COMPARE(&BACKWARD_LEFT, SHANK,  BL_SHANK_Value);
//		floatXYZ pos;
//		floatRPY angle;
//		
//		pos.x=0;
//		pos.y=0;
//		pos.z=150;
//		angle.P=0;
//		angle.R=0;
//		angle.Y=0;	
//		Pos_Attitude_Control(pos,angle);
	
	return 1;
}

/*位姿控制*/
uint8_t Pos_Attitude_Control(floatXYZ pos,floatRPY angle)
{
	/*每条腿的位置向量*/
  floatXYZ AB[4];
	AB[0]=Leg_Position_Vector(0,pos,angle);
	AB[1]=Leg_Position_Vector(1,pos,angle);
	AB[2]=Leg_Position_Vector(2,pos,angle);
	AB[3]=Leg_Position_Vector(3,pos,angle);
  inverse_kinematics(AB,4);
	Servo_Control();
	return 1;
}

/*角度转化为舵机PWM值*/
uint8_t Servo_Control(void)
{	
	float PWM[4][3];
	float temp_angle[4];
	static float filtered_PWM[4][3];
	static uint8_t pwm_initialized = 0;
	//Leg_angle[1].theta_3=-120*DEG_TO_RAD;
	/*左前腿*/
	PWM[0][0]=FL_HIP_Value   + ( Leg_angle[0].theta_1*RAD_TO_DEG-0 )/180*200;
	PWM[0][1]=FL_THIGH_Value + ( Leg_angle[0].theta_2*RAD_TO_DEG-0 )/180*200;
	/*转化连杆角度*/
	temp_angle[0]=Trans_Angle(-Leg_angle[0].theta_3-90*DEG_TO_RAD);
	PWM[0][2]=FL_SHANK_Value + ( temp_angle[0]*RAD_TO_DEG-0 )/180*200;
	
	/*右前腿*/
	PWM[1][0]=FR_HIP_Value   - ( Leg_angle[1].theta_1*RAD_TO_DEG-0 )/180*200;
	PWM[1][1]=FR_THIGH_Value - ( Leg_angle[1].theta_2*RAD_TO_DEG-0 )/180*200;
	/*转化连杆角度*/
	temp_angle[1]=Trans_Angle(-Leg_angle[1].theta_3-90*DEG_TO_RAD);
	PWM[1][2]=FR_SHANK_Value - ( temp_angle[1]*RAD_TO_DEG-0 )/180*200;
	
	/*左后腿*/
	PWM[2][0]=BL_HIP_Value   - ( Leg_angle[2].theta_1*RAD_TO_DEG-0 )/180*200;
	PWM[2][1]=BL_THIGH_Value + ( Leg_angle[2].theta_2*RAD_TO_DEG-0 )/180*200;
	/*转化连杆角度*/
	temp_angle[2]=Trans_Angle(-Leg_angle[2].theta_3-90*DEG_TO_RAD);
	PWM[2][2]=BL_SHANK_Value + ( temp_angle[2]*RAD_TO_DEG-0 )/180*200;
	
	/*右后腿*/
	PWM[3][0]=BR_HIP_Value   + ( Leg_angle[3].theta_1*RAD_TO_DEG-0 )/180*200;
	PWM[3][1]=BR_THIGH_Value - ( Leg_angle[3].theta_2*RAD_TO_DEG-0 )/180*200;
	/*转化连杆角度*/
	temp_angle[3]=Trans_Angle(-Leg_angle[3].theta_3-90*DEG_TO_RAD);
	PWM[3][2]=BR_SHANK_Value - ( temp_angle[3]*RAD_TO_DEG-0 )/180*200;	

	for(int i=0; i<4; i++)
	{
		for(int j=0; j<3; j++)
		{
			PWM[i][j] = Clamp_PWM(PWM[i][j]);
			if(pwm_initialized == 0)
				filtered_PWM[i][j] = PWM[i][j];
			else
				filtered_PWM[i][j] += servo_smooth_gain * (PWM[i][j] - filtered_PWM[i][j]);
		}
	}
	pwm_initialized = 1;

	__HAL_TIM_SET_COMPARE(&FORWARD_LEFT,   HIP,    (uint16_t)filtered_PWM[0][0]);  
	__HAL_TIM_SET_COMPARE(&FORWARD_LEFT,   THIGH,  (uint16_t)filtered_PWM[0][1]);   
	__HAL_TIM_SET_COMPARE(&FORWARD_LEFT,   SHANK,  (uint16_t)filtered_PWM[0][2]); 	
	__HAL_TIM_SET_COMPARE(&FORWARD_RIGHT,  HIP,    (uint16_t)filtered_PWM[1][0]);  
	__HAL_TIM_SET_COMPARE(&FORWARD_RIGHT,  THIGH,  (uint16_t)filtered_PWM[1][1]);   
	__HAL_TIM_SET_COMPARE(&FORWARD_RIGHT,  SHANK,  (uint16_t)filtered_PWM[1][2]);
	__HAL_TIM_SET_COMPARE(&BACKWARD_LEFT,  HIP,    (uint16_t)filtered_PWM[2][0]);  
	__HAL_TIM_SET_COMPARE(&BACKWARD_LEFT,  THIGH,  (uint16_t)filtered_PWM[2][1]);   
	__HAL_TIM_SET_COMPARE(&BACKWARD_LEFT,  SHANK,  (uint16_t)filtered_PWM[2][2]); 	
	__HAL_TIM_SET_COMPARE(&BACKWARD_RIGHT, HIP,    (uint16_t)filtered_PWM[3][0]);  
	__HAL_TIM_SET_COMPARE(&BACKWARD_RIGHT, THIGH,  (uint16_t)filtered_PWM[3][1]);   
	__HAL_TIM_SET_COMPARE(&BACKWARD_RIGHT, SHANK,  (uint16_t)filtered_PWM[3][2]);	
	
	return 1;
}

/*转化连杆角度*/
float Trans_Angle(float angle)
{
	float l,angle_;
	if(angle>0)
	{
		l=50*sin(0.5*angle);
		//printf("l=%f \n",l);
		angle_=asin( ( pow(l,2)+2*71*l )/ ( 48*( 71+l )) );
	}
	else
	{
		l=50*sin(-0.5*angle);
		//printf("l=%f \n",l);
		angle_=asin( ( pow(l,2)-2*71*l )/ ( 48*( 71-l )) );		
	}
	return angle_;
}

/*Trot步态函数*/
uint8_t Trot_Gait(MainState_t MainState)
{
	/*小跑中的跳跃动作：四腿一起蹲下再快速蹬起（一次性）*/
	if(MainState == MainState_Jump)
	{
		Trot_Jump_Action();
		return 1;
	}

	static float time=0;
	static int GaitTransition_flag=1;
	static MainState_t last_MainState=MainState_Static;

	if(MainState != last_MainState)
	{
		time = 0;
		GaitTransition_flag = 1;
		last_MainState = MainState;
	}

	if(MainState==MainState_Static)
	{
		time = 0;
		GaitTransition_flag = 1;
		Stand();
	}
	else
	{
		if(time>T)
		{
			time=0;
			GaitTransition_flag++;
		}
		floatXYZ AB[4];
		float smooth = Gait_Smooth_Phase(time);
		float body_shift_x = 0;
		float body_shift_y = 0;
		float body_shift_z = 0;
			
		if((GaitTransition_flag%2)==1)
		{
			AB[0]=SwayStatus_Trajectory(MainState,0,time);
			AB[3]=SwayStatus_Trajectory(MainState,3,time);
			AB[1]=SupportingStatus_Trajectory(MainState,1,time);
			AB[2]=SupportingStatus_Trajectory(MainState,2,time);
			
			switch(MainState)
			{
				case MainState_Forward:
					body_shift_y = -COG_Y_SHIFT_MAX * smooth;
					body_shift_x = COG_X_SHIFT_MAX * smooth;
					body_shift_z = (COG_Z_RAISE ? COG_Z_SHIFT_MAX : -COG_Z_SHIFT_MAX) * smooth;
					break;
				case MainState_Backward:
					body_shift_y = -COG_Y_SHIFT_MAX * smooth;
					body_shift_x = -COG_X_SHIFT_MAX * smooth;
					body_shift_z = (COG_Z_RAISE ? COG_Z_SHIFT_MAX : -COG_Z_SHIFT_MAX) * smooth;
					break;
				case MainState_Move_to_Left:
					body_shift_x = -COG_X_SHIFT_MAX * smooth;
					body_shift_z = (COG_Z_RAISE ? COG_Z_SHIFT_MAX : -COG_Z_SHIFT_MAX) * smooth;
					break;
				case MainState_Move_to_Right:
					body_shift_x = COG_X_SHIFT_MAX * smooth;
					body_shift_z = (COG_Z_RAISE ? COG_Z_SHIFT_MAX : -COG_Z_SHIFT_MAX) * smooth;
					break;
				case MainState_Turn_Left:
					body_shift_y = -COG_Y_SHIFT_MAX * smooth;
					body_shift_x = COG_X_SHIFT_MAX * smooth;
					body_shift_z = (COG_Z_RAISE ? COG_Z_SHIFT_MAX : -COG_Z_SHIFT_MAX) * smooth;
					break;
				case MainState_Turn_Right:
					body_shift_y = -COG_Y_SHIFT_MAX * smooth;
					body_shift_x = -COG_X_SHIFT_MAX * smooth;
					body_shift_z = (COG_Z_RAISE ? COG_Z_SHIFT_MAX : -COG_Z_SHIFT_MAX) * smooth;
					break;
				case MainState_Step:
					body_shift_z = -COG_Z_SHIFT_MAX * smooth;
					break;
				default:
					break;
			}
		}
		else
		{
			AB[1]=SwayStatus_Trajectory(MainState,1,time);
			AB[2]=SwayStatus_Trajectory(MainState,2,time);
			AB[0]=SupportingStatus_Trajectory(MainState,0,time);
			AB[3]=SupportingStatus_Trajectory(MainState,3,time);
			
			switch(MainState)
			{
				case MainState_Forward:
					body_shift_y = COG_Y_SHIFT_MAX * smooth;
					body_shift_x = COG_X_SHIFT_MAX * smooth;
					body_shift_z = (COG_Z_RAISE ? COG_Z_SHIFT_MAX : -COG_Z_SHIFT_MAX) * smooth;
					break;
				case MainState_Backward:
					body_shift_y = COG_Y_SHIFT_MAX * smooth;
					body_shift_x = -COG_X_SHIFT_MAX * smooth;
					body_shift_z = (COG_Z_RAISE ? COG_Z_SHIFT_MAX : -COG_Z_SHIFT_MAX) * smooth;
					break;
				case MainState_Move_to_Left:
					body_shift_x = -COG_X_SHIFT_MAX * smooth;
					body_shift_z = (COG_Z_RAISE ? COG_Z_SHIFT_MAX : -COG_Z_SHIFT_MAX) * smooth;
					break;
				case MainState_Move_to_Right:
					body_shift_x = COG_X_SHIFT_MAX * smooth;
					body_shift_z = (COG_Z_RAISE ? COG_Z_SHIFT_MAX : -COG_Z_SHIFT_MAX) * smooth;
					break;
				case MainState_Turn_Left:
					body_shift_y = COG_Y_SHIFT_MAX * smooth;
					body_shift_x = COG_X_SHIFT_MAX * smooth;
					body_shift_z = (COG_Z_RAISE ? COG_Z_SHIFT_MAX : -COG_Z_SHIFT_MAX) * smooth;
					break;
				case MainState_Turn_Right:
					body_shift_y = COG_Y_SHIFT_MAX * smooth;
					body_shift_x = -COG_X_SHIFT_MAX * smooth;
					body_shift_z = (COG_Z_RAISE ? COG_Z_SHIFT_MAX : -COG_Z_SHIFT_MAX) * smooth;
					break;
				case MainState_Step:
					body_shift_z = -COG_Z_SHIFT_MAX * smooth;
					break;
				default:
					break;
			}
		}
		
		for(int i = 0; i < 4; i++)
		{
			AB[i].x += body_shift_x;
			AB[i].y += body_shift_y;
			AB[i].z += body_shift_z;
		}
		
		inverse_kinematics(AB,4);
		Servo_Control();
		time=time+1;
	}		

	return 1;
}

/*跳跃步态：四腿同步弹跳（暂定实现，方向由 MainState 决定） */
uint8_t Jump_Gait(MainState_t MainState)
{
    static float time = 0;
    static float dx_acc = 0;
    static float dy_acc = 0;
    static MainState_t last_MainState = MainState_Static;

    if(MainState != last_MainState)
    {
        last_MainState = MainState;
        dx_acc = 0;
        dy_acc = 0;
    }

    time = time + 1;
    if(time > T) time = 0;

    float p = time / T;
    float jr;
    float launch = 0.0f;

    if(p < 0.35f)
    {
        jr = 25.0f * (p / 0.35f);
    }
    else if(p < 0.70f)
    {
        jr = 25.0f - 70.0f * ((p - 0.35f) / 0.35f);
        launch = (p - 0.35f) / 0.35f;
    }
    else
    {
        jr = -45.0f + 45.0f * ((p - 0.70f) / 0.30f);
    }

    if(MainState == MainState_Forward)        dx_acc += 18.0f * launch;
    else if(MainState == MainState_Backward)  dx_acc -= 18.0f * launch;
    else if(MainState == MainState_Move_to_Left)  dy_acc += 18.0f * launch;
    else if(MainState == MainState_Move_to_Right) dy_acc -= 18.0f * launch;

    floatXYZ pos;
    floatRPY angle;
    pos.x = 0;  pos.y = 0;  pos.z = 150 - jr;
    angle.R = 0; angle.P = 0; angle.Y = 0;
    Pos_Attitude_Control(pos, angle);

    return 1;
}

/*小跑中的跳跃：四腿一起蹲下再快速蹬起（一次性动作，结束自动回站立）
  通过整体机身高度 z 的下蹲/弹起来实现四腿同步蹲蹬 */
uint8_t Trot_Jump_Action(void)
{
    static float jt = 0;
    static uint8_t active = 0;

    if(!active)            /* 首次进入：启动计时，并临时调大平滑增益让舵机真正跟得上 */
    {
        jt = 0;
        active = 1;
        servo_smooth_gain = 0.70f;   /* 跳跃期间跟随更快，蹬起距离更大 */
    }

    const float T_total = 50.0f;   /* 总时长约 0.50s */
    float p = jt / T_total;        /* 归一化进度 0..1 */
    float z;

    if(p < 0.32f)
    {
        /* 蹲下：四腿一起降低机身（储能），先慢后快，蹲得更深 */
        float k = p / 0.32f;
        z = 150.0f - 80.0f * (k * k);        /* 150 -> 70 */
    }
    else if(p < 0.46f)
    {
        /* 蹬起：线性匀速轨迹，配合高平滑增益 -> 既快又真正蹬得高 */
        float k = (p - 0.32f) / 0.14f;
        z = 70.0f + 165.0f * k;              /* 70 -> 235 快速弹起 */
    }
    else if(p < 0.80f)
    {
        /* 滞空回落 */
        float k = (p - 0.46f) / 0.34f;
        z = 235.0f - 85.0f * k;              /* 235 -> 150 */
    }
    else
    {
        /* 落地缓冲：停留站立并结束动作，恢复默认平滑增益 */
        z = 150.0f;
        if(jt >= T_total)
        {
            active = 0;
            servo_smooth_gain = LEG_PWM_SMOOTH_GAIN;  /* 还原，避免影响其它步态 */
            Mainstate = MainState_Static;     /* 结束回站立 */
            return 1;
        }
    }

    floatXYZ pos;
    floatRPY angle;
    pos.x = 0;  pos.y = 0;  pos.z = z;
    angle.R = 0; angle.P = 0; angle.Y = 0;
    Pos_Attitude_Control(pos, angle);

    jt += 1.0f;
    return 1;
}

/*打招呼：右前腿在前述逆解站立基础上向前抬腿并挥手 */
uint8_t Wave_Action(void)
{
    static float phase = 0.0f;

    Stand();   /* 四条腿先进入正确逆解站立 */

    /* hip基准用右前髋站立校准值FR_HIP_Value；右前舵机为减号方向，
       PWM减小=腿外摆（往上举），原代码用160基准导致PWM增大=内收压下，方向反了 */
    float fr_hip_base   = (float)FR_HIP_Value;
    float fr_thigh_base = 66.0f;
    float fr_shank_base = 194.0f;

    float fr_hip   = fr_hip_base   - 25.0f;   /* 髋关节往上举（外摆，PWM减小方向） */
    float fr_thigh = fr_thigh_base + 35.0f;   /* 向前抬（正方向） */
    float fr_shank = fr_shank_base;

    phase += 0.12f;
    if(phase > 6.2831853f) phase -= 6.2831853f;
    float wave = 22.5f * sinf(phase);   /* 摆幅±22.5 */

    /* 在“前平举”基础上再向上抬约90度：中心再 +70（约 +105），叠加±22.5 */
    fr_thigh += wave + 150.0f;   /* 中心约 +105，向前上方抬起 */
    fr_shank += wave;

    __HAL_TIM_SET_COMPARE(&FORWARD_RIGHT, HIP,   (uint16_t)Clamp_PWM(fr_hip));
    __HAL_TIM_SET_COMPARE(&FORWARD_RIGHT, THIGH, (uint16_t)Clamp_PWM(fr_thigh));
    __HAL_TIM_SET_COMPARE(&FORWARD_RIGHT, SHANK, (uint16_t)Clamp_PWM(fr_shank));

    return 1;
}

/*IMU自稳*/
uint8_t IMU_Attitude_Control(void)
{
	floatXYZ pos;
	pos.x=0;
	pos.y=0;
	pos.z=130;		
	floatRPY euler_angle;
	
	/* KP拉大5倍：即使IMU读数偏小也能产生足够补偿；去掉KD避免阻尼拖慢响应 */
	float KP=5.0f;
	float KD=0.0f;	

	static float err_P=0;
	static float err_last_P=0;
	static float err_R=0;
	static float err_last_R=0;	
	/* MPU6050在小狗上安装方向旋转了90°：angle[0]实际是俯仰(前后倾)、angle[1]实际是横滚(左右倾) */
	err_P=0-angle[0];   /* 俯仰误差（前后方向已验证正确） */
	err_R=angle[1];     /* 横滚误差（左右方向已取反修正） */
	
	/* 无死区：任何倾斜都立即补偿 */
	#define IMU_DEADBAND_DEG 0.0f
	/* 限幅放宽到±35°，让补偿幅度更大 */
	#define IMU_MAX_OUTPUT_DEG 35.0f

	if(err_P >  IMU_DEADBAND_DEG) err_P -= IMU_DEADBAND_DEG;
	else if(err_P < -IMU_DEADBAND_DEG) err_P += IMU_DEADBAND_DEG;
	else err_P = 0;

	if(err_R >  IMU_DEADBAND_DEG) err_R -= IMU_DEADBAND_DEG;
	else if(err_R < -IMU_DEADBAND_DEG) err_R += IMU_DEADBAND_DEG;
	else err_R = 0;

	euler_angle.P=KP*err_P+KD*(err_P-err_last_P);
	err_last_P=err_P;
	euler_angle.R=KP*err_R+KD*(err_R-err_last_R);
	err_last_R=err_R;

	if(euler_angle.P >  IMU_MAX_OUTPUT_DEG) euler_angle.P =  IMU_MAX_OUTPUT_DEG;
	if(euler_angle.P < -IMU_MAX_OUTPUT_DEG) euler_angle.P = -IMU_MAX_OUTPUT_DEG;
	if(euler_angle.R >  IMU_MAX_OUTPUT_DEG) euler_angle.R =  IMU_MAX_OUTPUT_DEG;
	if(euler_angle.R < -IMU_MAX_OUTPUT_DEG) euler_angle.R = -IMU_MAX_OUTPUT_DEG;
	
	euler_angle.Y=0;
	/* 自稳时关闭平滑滤波(gain=1.0)，舵机立即跟随目标姿态，零延迟 */
	float saved_gain = servo_smooth_gain;
	servo_smooth_gain = 1.0f;
	Pos_Attitude_Control(pos,euler_angle);
	servo_smooth_gain = saved_gain;
	return 1;
}
