#include "Task_Init.h"

#include "bsp_dwt.h"
#include "usart.h"
#include "remote.h"
#include "robot_def.h"
#include "ins_task.h"
#include "tim.h"



TaskHandle_t Task_Robot_Handle;
TaskHandle_t Task_Protect_Handle;

#ifdef BMI088_INS
TaskHandle_t Task_INS_Handle;
#endif

#ifdef GIMBAL_BOARD
TaskHandle_t Task_Shoot_Handle;
TaskHandle_t Task_Gimbal_Handle;
TaskHandle_t Task_Vision_Handle;
#endif
#ifdef CHASSIS_BOARD
TaskHandle_t Task_Chassis_Handle;
TaskHandle_t Task_PowerController_Handle;
#endif

void Task_Init()
{
    taskENTER_CRITICAL(); // 进入临界区
	 /* 外设初始化 */
	DWT_Init(168);
/* 创建任务 */
#ifdef GIMBAL_BOARD
	/* 串口1初始化 DT7遥控器 */
	Remote_Init();
    
	xTaskCreate((TaskFunction_t)Task_Robot,             "Task_Robot",             128*4, NULL, 7, &Task_Robot_Handle);
	xTaskCreate((TaskFunction_t)Task_Protect,           "Task_Protect",           128*2, NULL, 6, &Task_Protect_Handle);
	xTaskCreate((TaskFunction_t)Task_Gimbal,            "Task_Gimbal",            128*8, NULL, 5, &Task_Gimbal_Handle);
	xTaskCreate((TaskFunction_t)Task_Shoot,             "Task_Shoot",             128*8, NULL, 5, &Task_Shoot_Handle);
	xTaskCreate((TaskFunction_t)Task_Vision,            "Task_Vision",            128*2, NULL, 6, &Task_Vision_Handle);
#endif
#ifdef CHASSIS_BOARD
	xTaskCreate((TaskFunction_t)Task_Chassis,           "Task_Chassis",          128*8, NULL, 5, &Task_Chassis_Handle);
	xTaskCreate((TaskFunction_t)Task_Robot,             "Task_Robot",            128*4, NULL, 6, &Task_Robot_Handle);
	xTaskCreate((TaskFunction_t)Task_Protect,           "Task_Protect",          128*2, NULL, 7, &Task_Protect_Handle);
#endif
#ifdef WT931_IMU
    /* 串口2初始化（WT931陀螺仪） */
	IMU_Init();
#elif defined(BMI088_INS)
	INS_Init();
	xTaskCreate((TaskFunction_t)Task_INS,            "Task_INS",            128*8,   NULL, 7, &Task_INS_Handle);
#endif
     HAL_TIM_Base_Start_IT(&htim5);
    taskEXIT_CRITICAL(); // 退出临界区
    vTaskDelete(NULL);   // 删除开始空闲任务
}




