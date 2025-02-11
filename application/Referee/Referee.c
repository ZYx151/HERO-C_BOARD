#include "Referee.h"
#include "usart.h"
#include "robot_def.h"
#include "DMA_double_buffer.h"
#include "pub_sub.h"

#include "Referee_unpack.h"
#include "WatchDog.h"

#include "Chassis.h"

/** 裁判系统  **/
uint8_t Usart6_Referee_buffer[2][REFEREE_BUFFER_LEN];  // 用于裁判系统接收
Publisher_t *referee_feedback_pub;           // 裁判系统反馈消息发布者
RMQueue_Handle Referee_queue;                // 裁判系统解算数据包
Referee_data_t referee_send_data;
WatchDog_TypeDef *Referee_Dog;

void Referee_Init()
{
    /* 串口6初始化(裁判系统通信)*/
    RMQueueInit(&Referee_queue, REFEREE_BUFFER_LEN, RM_QUEUE_LEN);
    HAL_NVIC_DisableIRQ(USART6_IRQn);       // DMA发送需在对应DMA中断中清除状态
    HAL_UART_Receive_DMA_double(&huart6, Usart6_Referee_buffer[0] , Usart6_Referee_buffer[1], REFEREE_BUFFER_LEN);
    
    referee_feedback_pub = PubRegister("referee", sizeof(Referee_data_t) );
	
	WatchDog_Init_config Referee_Config = {
	//	.watch_callback = ,
	//	.feed_callback = ,
		.dog_name = "RefereeDog",
		.Max_num = 15,
	};
	Referee_Dog = WatchDog_Init(Referee_Config);
}

void Referee_Update()
{
    unpack_referee_system_data(&Referee_queue);//裁判系统解包
        if(!Damage_status.HP_deduction_reason) {
            Damage_status.HP_deduction_reason = 1;
        }
	/* 等级（底盘增益，小陀螺增益），枪口热量(弹频限制)，弹丸速度（自调节或手动调），颜色（视觉识别），冷却增益（单频） */
	if (Referee_Dog->state == Dog_Online){
		referee_send_data.refree_status = Device_Online;
		if(Robot_state.robot_id < 100)
			referee_send_data.robot_color = 1;//红1
		else
			referee_send_data.robot_color = 0;//蓝0
		referee_send_data.shooter_barrel_heat_limit   = Robot_state.shooter_barrel_heat_limit; // 热量上限
		referee_send_data.heat_limit_remain           = Real_time_chassis_power_and_shoot_heat.shooter_42mm_barrel_heat; // 实时热量
		referee_send_data.buffer_energy               = Real_time_chassis_power_and_shoot_heat.buffer_energy;   // 缓冲能量
		referee_send_data.bullet_speed_now            = Real_time_shooting.initial_speed;   // 实时弹速
		referee_send_data.chassis_power               = Real_time_chassis_power_and_shoot_heat.chassis_power; // 底盘功率
		referee_send_data.chassis_power_limit         = Robot_state.chassis_power_limit;    // 功率上限
        referee_send_data.robot_HP                    = Robot_state.current_HP;             // 机器人当前血量
        referee_send_data.chassis_output              = Robot_state.power_management.chassis_output; //chassis 口输出：0 为无输出，1 为 24V 输出
	} else referee_send_data.refree_status = Device_Offline;
	 //  消息发布
    PubPushMessage( referee_feedback_pub, (void *)&referee_send_data);
}

/* DMA接收完成回调函数 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART6)  {
       void *endPtr = RMQueueGetEndPtr(&Referee_queue);
        if (endPtr) {
            (huart->hdmarx->Instance->CR & DMA_SxCR_CT) == RESET ?  
                memcpy(endPtr, Usart6_Referee_buffer[1], REFEREE_BUFFER_LEN)
			      : memcpy(endPtr, Usart6_Referee_buffer[0], REFEREE_BUFFER_LEN);
           RMQueuePushEndPtr(&Referee_queue);
        } else  {
            unpack_referee_system_data(&Referee_queue);//队列满了没处理
          RMQueuePushEndPtr(&Referee_queue);//队列被锁定
        }
        Feed_Dog(Referee_Dog);
	}
}


