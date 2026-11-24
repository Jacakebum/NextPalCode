/*ͷ�ļ�Ԥ����*/
#define __TASK_INIT_GLOBALS
/*ͷ�ļ�Ԥ��������*/

/*����ͷ�ļ�*/
#include "Task_init.h"
#include "tim.h"
#include "MPU6050.h"
#include "oled.h"
#include "stdio.h"

/* -------------------------------------------------------------------------- */
static void Servo_Init(void);

/* ---------------------------- Static Variables ---------------------------- */


/* ---------------------------- global functions ---------------------------- */

/**
  * @brief  Create all task
  * @note   
  * @param  parameters: none
  * @retval None
  */
void Task_Init_Start(void *parameters)
{
	taskENTER_CRITICAL();
	
	IMU_Init();
	Servo_Init();
	OLED_Init();
	OLED_ShowString(2, 1, "NextPal");
  printf("** Peripheral inited successfully. ** \r\n");
	
	HAL_Delay(2000);
	
	OLED_Clear();
	OLED_ShowString(3, 1, "G:");
	OLED_ShowString(3, 8, "A:");
	OLED_ShowString(4, 1, "BT:");
	
	HAL_Delay(100);
	
	vTaskDelete(NULL);
	taskEXIT_CRITICAL();
}

/* ----------------------- Static Function Definitions ---------------------- */


void Servo_Init(void)
{
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);	
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);	

	/* 启动 TIM6 软PWM中断（驱动 PA8 MOUTH, PA15 Head-YAW） */
	HAL_TIM_Base_Start_IT(&htim6);
}
