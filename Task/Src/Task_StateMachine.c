#include "Task_StateMachine.h"
#include "stdio.h"
#include "Inverse_K.h"
/*------------------------ state enum --------------------------*/

/*----------------------- state Variables -----------------------*/
GaitMode_t GaitMode;
MainState_t Mainstate;
uint8_t *rdata;
uint8_t *rdata3;

osEvent retval;
osEvent retval3;	
/* ---------------------------- Global Variables ---------------------------- */ 
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern TaskHandle_t TaskHandle_StateMachine;
extern osMailQId myMail01Handle;
extern osMailQId myMail02Handle;
extern uint8_t data[20];
extern uint8_t data2[20];
//extern osMailQId myMail03Handle;

extern uint8_t data3[10];
uint8_t flag=0;
float distance=0;
extern volatile uint8_t bluetooth_connected;
extern volatile uint8_t bluetooth_ack_pending;
	
// 蓝牙连接超时计数器（非static，允许其他文件访问）
uint16_t bluetooth_timeout = 0;

static uint8_t head_yaw = 90;
static uint8_t head_pitch = 90;
static uint8_t head_roll = 90;
static HeadMotionMode_t head_motion_mode = HeadMotion_None;
static uint8_t tail_lr = 90;     /* 尾巴左右: 0=最左, 180=最右, 90=居中 */
static uint8_t mouth_open = 90; /* 嘴巴开合: 0=全闭, 180=全开, 90=半开 */
static TailMotionMode_t tail_motion_mode = TailMotion_None; /* 摇尾巴模式 */

/*----------------------- state functions -----------------------*/
static uint8_t StateMachine_Init(void);
static uint8_t MainState_Update(void);
static uint8_t GaitMode_Update(void);
static uint8_t Avoid_Obstacle(void);
static void ApplyCommand(uint8_t *cmd);

/**************************
  * @brief  状态机任务
  * @param  unused
  * @retval void
  * @note   5ms执行一次
  */
void Task_StateMachine_Start(void *parameters)
{	
	  StateMachine_Init();
		TickType_t xLastWakeUpTime;
	  HAL_UART_Receive_IT(&huart2,data,2);
	  HAL_UART_Receive_IT(&huart3,data2,1);
	  HAL_UART_Receive_IT(&huart4,data3,1);
	  
    xLastWakeUpTime = xTaskGetTickCount();
    
    // 蓝牙心跳包计数器
    static uint16_t heartbeat_count = 0;
    const char* heartbeat_msg = "PING";
    const uint8_t bluetooth_ack_msg[] = "ok";
    
    /* 启动后自动前进计时：2000×10ms=20秒，到点若仍在前进则自动停止 */
    static uint16_t startup_walk_counter = 2000;
    static uint8_t startup_walk_done = 0;
    
    while (1)
    {
			retval=osMailGet(myMail01Handle,0);
			if(retval.status==osEventMail)
			{
			 // printf("Receiving queue!\n");
			  rdata=retval.value.p;
			  
			  ApplyCommand(rdata);
			  // 收到蓝牙数据，重置超时计数器
			  bluetooth_timeout = 0;
			  bluetooth_connected = 1;
			  
			  // 调试：保存接收到的数据
			  extern uint8_t data[20];
			  // 释放邮件队列内存
			  osMailFree(myMail01Handle, rdata);
      }
			retval3=osMailGet(myMail02Handle,0);
			if(retval3.status==osEventMail)
			{
				rdata3=retval3.value.p;
				ApplyCommand(rdata3);
				osMailFree(myMail02Handle, rdata3);
			}
			
			Avoid_Obstacle();
			
			// 蓝牙连接超时检测（降低到2秒）
			bluetooth_timeout++;
			if(bluetooth_timeout >= 200)  // 2秒没有收到数据，认为断开
			{
				bluetooth_connected = 0;
			}

			while(bluetooth_ack_pending > 0)
			{
				if(HAL_UART_Transmit(&huart2, (uint8_t*)bluetooth_ack_msg, 2, 100) != HAL_OK)
				{
					break;
				}
				bluetooth_ack_pending--;
			}
			
			// 每1秒发送一次蓝牙心跳包
			heartbeat_count++;
			if(heartbeat_count >= 100)
			{
				HAL_UART_Transmit(&huart2, (uint8_t*)heartbeat_msg, 4, 100);
				heartbeat_count = 0;
			}

			/* 启动后自动前进20秒后停止；期间用户发指令改变状态则不强制停止 */
			if(!startup_walk_done)
			{
				if(startup_walk_counter > 0) startup_walk_counter--;
				if(startup_walk_counter == 0)
				{
					if(GetMainState() == MainState_Forward && GetGaitMode() == GaitMode_Trot)
						Mainstate = MainState_Static;
					startup_walk_done = 1;
				}
			}

			
			vTaskDelayUntil(&xLastWakeUpTime, 10);      /*10ms更新一次*/
		}
}

static void ApplyCommand(uint8_t *cmd)
{
	rdata = cmd;
	
	/* ASRPro→蓝牙透传：收到"71"/"72"等7x指令，原样通过蓝牙(USART2)转发出去，保持格式一致 */
	if(cmd[0] == '7')
	{
		HAL_UART_Transmit(&huart2, cmd, 2, 100);
	}
	
	MainState_Update();
	GaitMode_Update();
	data[0] = cmd[0];
	data[1] = cmd[1];
}

GaitMode_t GetGaitMode(void)
{
    return GaitMode;
}

MainState_t GetMainState(void)
{
    return Mainstate;
}

uint8_t GetHeadYaw(void)
{
	return head_yaw;
}

uint8_t GetHeadPitch(void)
{
	return head_pitch;
}

uint8_t GetHeadRoll(void)
{
	return head_roll;
}

HeadMotionMode_t GetHeadMotionMode(void)
{
	return head_motion_mode;
}

uint8_t GetTailLR(void)
{
	return tail_lr;
}

TailMotionMode_t GetTailMotionMode(void)
{
	return tail_motion_mode;
}

uint8_t GetMouthOpen(void)
{
	return mouth_open;
}

/*--------------------static Function--------------------*/
/**
 * @brief  初始化状态机
 * @note   默认所有保护模式
 * @retval 
 */
uint8_t StateMachine_Init(void)
{
    GaitMode = GaitMode_Trot;      //初始为小跑
    Mainstate = MainState_Forward; //初始为前进（启动后自动前进20秒后停止）
	head_yaw = 90;
	head_pitch = 90;
	head_roll = 90;
	head_motion_mode = HeadMotion_None;
	tail_lr = 90;
	tail_motion_mode = TailMotion_None;
	mouth_open = 90;
    return 1;
}

uint8_t MainState_Update(void)
{
	if(*rdata==48)     //rdata[0]="0"
	{
	 switch(*(rdata+1)) 
	 {			
		 case 53://rdata[1]="5" - 原踏步，现改为嘴巴测试（蓝牙用，ASRPro不发05）
	       Mainstate = MainState_Static;
	       if(mouth_open > 90) mouth_open = 90;  /* 回半开 */
	       else mouth_open = 180;                /* 全开 */
		 break;	
		 case 50://rdata[1]="2" - 前进（不动）
	       Mainstate = MainState_Forward;
		 break;
		 case 56://rdata[1]="8" - 后退（不动）
	       Mainstate = MainState_Backward;
		 break;		
		 case 52://rdata[1]="4" - 左移（状态对调修正实际方向）
	       Mainstate = MainState_Move_to_Right;
		 break;	
		 case 54://rdata[1]="6" - 右移（状态对调修正实际方向）
	       Mainstate = MainState_Move_to_Left;
		 break;	
		 case 49://rdata[1]="1" - 向左转（恢复原功能）
	       Mainstate = MainState_Turn_Left;
		 break;		
		 case 51://rdata[1]="3" - 右转（ASRPro: 向右转，不动）
			 Mainstate = MainState_Turn_Right;
		 break;	
     case 55://rdata[1]="7" - IMU自稳（仅扭动步态下生效：先下发"13"切到扭动，再下发"07"开启）
		 Mainstate = MainState_IMU;
	 break;
		 case 57://rdata[1]="9" - 原打招呼，现改回向右转
			 Mainstate = MainState_Turn_Right;
		 break;
		 case 65://rdata[1]="A" - 小跑中的跳跃（四腿蹲下再蹬起）
		   GaitMode = GaitMode_Trot;     /* 确保在小跑下执行 */
	       Mainstate = MainState_Jump;
		 break;
		 case 48://rdata[1]="0" - 全部舵机回中并站立
	       Mainstate = MainState_Static;
	       head_motion_mode = HeadMotion_None;
	       head_yaw = 90;
	       head_pitch = 90;
	       head_roll = 90;
	       tail_lr = 90;
	       mouth_open = 90;
		 break;
     default:
       ;
	  }
   }
	 else if(*rdata==50)     //rdata[0]="2" - 舵机测试（ASRPro兼容）
	 {
		 switch(*(rdata+1))
		 {
			 /* 以下 22-27 是 ASRPro 语音指令，用于测试5个舵机 */
			 case 50:  // rdata[1]="2" - ASRPro"原地踏步"→ 尾巴测试（左右切换）
				 Mainstate = MainState_Static;
				 if(tail_lr > 90) tail_lr = 90;  /* 回中 */
				 else tail_lr = 180;            /* 摆到最右 */
			 break;
			 case 51:  // rdata[1]="3" - ASRPro"扭一扭"→ 嘴巴测试（开关切换）
				 Mainstate = MainState_Static;
				 if(mouth_open > 90) mouth_open = 90;  /* 回半开 */
				 else mouth_open = 180;                /* 全开 */
			 break;
		 case 52:  // rdata[1]="4" - ASRPro"看左边"→ 来回左右扭头(Shake)
			 Mainstate = MainState_Static;
			 head_motion_mode = HeadMotion_Shake;
		 break;
		 case 53:  // rdata[1]="5" - ASRPro"看右边"→ 来回左右扭头(Shake)
			 Mainstate = MainState_Static;
			 head_motion_mode = HeadMotion_Shake;
		 break;
		 case 54:  // rdata[1]="6" - ASRPro"点头"→ 来回上下点头(Nod)
			 Mainstate = MainState_Static;
			 head_motion_mode = HeadMotion_Nod;
		 break;
		 case 55:  // rdata[1]="7" - ASRPro"歪头"→ 来回歪头(Tilt)
			 Mainstate = MainState_Static;
			 head_motion_mode = HeadMotion_Tilt;
		 break;
		 case 57:  // rdata[1]="9" - 打招呼（挥手，自动切Stand步态；蓝牙/语音均可发）
			 GaitMode = GaitMode_Stand;    /* 确保在站立步态下执行 */
		     Mainstate = MainState_Wave;
		 break;
			 /* S/T/H 参数控制保留在 6x 段（蓝牙用） */
			 default:
			 break;
		 }
	 }
	 else if(*rdata==54)     //rdata[0]="6" - S/T/H 参数（蓝牙用，避开ASRPro指令码）
	 {
		 switch(*(rdata+1))
		 {
			 case 49:  // rdata[1]="1" - S加5
			 if(S + 5.0f <= 75.0f) S += 5.0f;
		 break;
		 case 50:  // rdata[1]="2" - S减5
			 if(S - 5.0f >= 20.0f) S -= 5.0f;
		 break;
		 case 51:  // rdata[1]="3" - T加1
			 if(T + 1.0f <= 90.0f) T += 1.0f;
		 break;
		 case 52:  // rdata[1]="4" - T减1
			 if(T - 1.0f >= 25.0f) T -= 1.0f;
		 break;
		 case 53:  // rdata[1]="5" - H加2
			 if(H + 2.0f <= 48.0f) H += 2.0f;
		 break;
		 case 54:  // rdata[1]="6" - H减2
			 if(H - 2.0f >= 12.0f) H -= 2.0f;
		 break;
			 default:
			 break;
		 }
	 }
	 else if(*rdata==51)     //rdata[0]="3" - 控制头部三自由度
	 {
		 switch(*(rdata+1))
		 {
			 case 49:  // rdata[1]="1" - ASRPro"叫/看我"→ 嘴巴测试（开关切换）
				 Mainstate = MainState_Static;
				 if(mouth_open > 90) mouth_open = 90;  /* 回半开 */
				 else mouth_open = 180;                /* 全开（张嘴叫） */
			 break;
			 case 50:  // rdata[1]="2" - Yaw减5
				 if(head_yaw >= 5) head_yaw -= 5;
				 head_motion_mode = HeadMotion_None;
			 break;
			 case 51:  // rdata[1]="3" - Pitch加5
				 if(head_pitch + 5 <= 180) head_pitch += 5;
				 head_motion_mode = HeadMotion_None;
			 break;
			 case 52:  // rdata[1]="4" - Pitch减5
				 if(head_pitch >= 5) head_pitch -= 5;
				 head_motion_mode = HeadMotion_None;
			 break;
			 case 53:  // rdata[1]="5" - Roll加5
				 if(head_roll + 5 <= 180) head_roll += 5;
				 head_motion_mode = HeadMotion_None;
			 break;
			 case 54:  // rdata[1]="6" - Roll减5
				 if(head_roll >= 5) head_roll -= 5;
				 head_motion_mode = HeadMotion_None;
			 break;
			 case 55:  // rdata[1]="7" - 点头动作
				 head_motion_mode = HeadMotion_Nod;
			 break;
			 case 56:  // rdata[1]="8" - 摇头动作
				 head_motion_mode = HeadMotion_Shake;
			 break;
			 case 57:  // rdata[1]="9" - 歪头动作
				 head_motion_mode = HeadMotion_Tilt;
			 break;
			 case 48:  // rdata[1]="0" - ASRPro"好狗"→ 全部回中并站立
			 Mainstate = MainState_Static;
			 head_motion_mode = HeadMotion_None;
			 head_yaw = 90;
			 head_pitch = 90;
			 head_roll = 90;
			 tail_lr = 90;
			 mouth_open = 90;
		 break;
			 default:
			 break;
		 }
	 }
	 /* rdata[0]="4" - 控制尾巴左右摆动 */
	 else if(*rdata==52)
	 {
		 switch(*(rdata+1))
		 {
			 case 49:  // rdata[1]="1" - 尾巴左摆 +5
				 if(tail_lr + 5 <= 180) tail_lr += 5;
			 break;
			 case 50:  // rdata[1]="2" - 尾巴右摆 -5
				 if(tail_lr >= 5) tail_lr -= 5;
			 break;
			 case 51:  // rdata[1]="3" - 摇尾巴（以90°为中位，0°~180°大幅来回摆动）
				 tail_motion_mode = TailMotion_Wag;
			 break;
			 case 48:  // rdata[1]="0" - 尾巴回中（同时停止摇尾巴）
				 tail_lr = 90;
				 tail_motion_mode = TailMotion_None;
			 break;
			 default:
			 break;
		 }
	 }
	 /* rdata[0]="5" - 控制嘴巴开合 */
	 else if(*rdata==53)
	 {
		 switch(*(rdata+1))
		 {
			 case 49:  // rdata[1]="1" - 嘴巴张大 +5
				 if(mouth_open + 5 <= 180) mouth_open += 5;
			 break;
			 case 50:  // rdata[1]="2" - 嘴巴闭合 -5
				 if(mouth_open >= 5) mouth_open -= 5;
			 break;
			 case 48:  // rdata[1]="0" - 嘴巴半开(回中)
				 mouth_open = 90;
			 break;
			 default:
			 break;
		 }
	 }
	 else if(*rdata==49)     //rdata[0]="1"
	 {
		 flag=0;
		 Mainstate=MainState_Static;
	 }
		 return 1;
}

uint8_t GaitMode_Update(void)
{
	if(*rdata==49)     //rdata[0]="1"
	{
		switch(*(rdata+1))
		 {				 
			 case 48:	//rdata[1]="10"
         GaitMode = GaitMode_Walk;
			 break;	
			 case 49://rdata[1]="11"
         GaitMode = GaitMode_Trot;
			 break;
			 case 50://rdata[1]="12"
         GaitMode = GaitMode_Stand;
			 break;		
			 case 51://rdata[1]="13"
         GaitMode = GaitMode_Body_Twist;
			 break;
			 case 52://rdata[1]="14"
         GaitMode = GaitMode_Jump;
			 break;			 

			 default:
				 GaitMode = GaitMode_Stand;
		 }
	}
  return 1;	
}

uint8_t Avoid_Obstacle(void)
{	
	return 1;		
}
