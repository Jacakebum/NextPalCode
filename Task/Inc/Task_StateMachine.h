#ifndef _TASK_STATEMACHINE_H__
#define _TASK_STATEMACHINE_H__
#include "sysconfig.h"

/******��̬
       վ��
       Walk��̬
			 Trot��̬
			 Crawl��̬
***************/
typedef enum
{
    GaitMode_Stand,	
    GaitMode_Walk,
    GaitMode_Trot,
	  GaitMode_Crawl,
	  GaitMode_Body_Twist,
	  GaitMode_Jump
}GaitMode_t;

/******��״̬
       ��ֹ
       ǰ��
			 ����
			 ����ƽ��
			 ����ƽ��
			 ������ת
			 ������ת
***************/
typedef enum
{
	  MainState_Static,
		MainState_Step,
    MainState_Forward,
    MainState_Backward,
    MainState_Move_to_Left,
    MainState_Move_to_Right,
    MainState_Turn_Left,
	  MainState_Turn_Right,
	  MainState_IMU,
	  MainState_Avoid_Obstacle,
	  MainState_Wave,
	  MainState_Jump
}MainState_t;

typedef enum
{
	HeadMotion_None = 0,
	HeadMotion_Nod,
	HeadMotion_Shake,
	HeadMotion_Tilt,
	HeadMotion_LookLeft,   /* 单向向左扭头（看左边） */
	HeadMotion_LookRight   /* 单向向右扭头（看右边） */
}HeadMotionMode_t;

typedef enum
{
	TailMotion_None = 0,
	TailMotion_Wag   /* 摇尾巴：以90°为中位，在0°~180°之间大幅来回摆动 */
}TailMotionMode_t;

void Task_StateMachine_Start(void *parameters);
GaitMode_t GetGaitMode(void);
MainState_t GetMainState(void);
uint8_t GetHeadYaw(void);
uint8_t GetHeadPitch(void);
uint8_t GetHeadRoll(void);
HeadMotionMode_t GetHeadMotionMode(void);
uint8_t GetTailLR(void);
uint8_t GetMouthOpen(void);
TailMotionMode_t GetTailMotionMode(void);
#endif
