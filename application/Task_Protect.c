#include "Task_Protect.h"
#include "Task_Init.h"
#include "WatchDog.h"
#include "bsp_dwt.h"


#define LOOK_STACK 0
#if LOOK_STACK
UBaseType_t uxHighWaterMark[6];//观察任务堆栈使用情况
char taskListBuffer[30*6];//保存任务运行时间信息 分别是：任务名 任务状态 优先级 剩余栈 任务序号
char taskStateBuffer[30*6];//保存任务运行时间信息 分别是：任务名 运行计数  使用率
#endif

float dwt_tim, dwt_delt;

/* 断控保护任务 */
void Task_Protect(void *pvParameters)
{
    static portTickType currentTime;
	for(;;)
	{
	    currentTime = xTaskGetTickCount();  // 	获取当前系统时间
		WatchDog_Polling(); // 软件看门狗轮询
		
//		HAL_IWDG_Refresh(&hiwdg);  // 硬件看门狗
		
#if LOOK_STACK
        //获得任务名 任务状态 优先级 剩余栈 任务序号
        memset(taskListBuffer, 0, 30*6);
        vTaskList((char *)&taskListBuffer); 
        //获取剩余Stack大小,堆栈不够会进入硬件错误
        uxHighWaterMark[0] = uxTaskGetStackHighWaterMark( Task_Robot_Handle );
        uxHighWaterMark[1] = uxTaskGetStackHighWaterMark( Task_DJLMotor_Handle );
        uxHighWaterMark[2] = uxTaskGetStackHighWaterMark( Task_DMMotor_Handle );
        uxHighWaterMark[3] = uxTaskGetStackHighWaterMark( Task_Protect_Handle );
        uxHighWaterMark[4] = uxTaskGetStackHighWaterMark( Task_Shoot_Handle );
        uxHighWaterMark[5] = uxTaskGetStackHighWaterMark( Task_Gimbal_Handle );
        //获取任务名 运行计数  使用率
        memset(taskStateBuffer, 0, 30*6);
        vTaskGetRunTimeStats((char *)&taskStateBuffer);
#endif
		
       vTaskDelayUntil(&currentTime, 3);
	}
}
