#include "Gimbal_board_cmd.h"
#include "Task_Init.h"
 // module
#include "DJIMotor.h"
#include "IMU.h"
#include "remote.h"
#include "pub_sub.h"
#include "controller.h"
 // bsp
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "bsp_usb.h"
#include "can_comm.h"
#include "CANDrive.h"
#include "ins_task.h"

static  CANCommInstance *can_cmd_comm;

// cmd的private函数
void soft_rest(void);
void stop_mode_update(void);                //机器人停止模式更新函数
void remote_mode_update(void);              //机器人遥控器模式更新函数
void mouse_key_mode_update(void);           //机器人键鼠模式更新函数
void send_cmd_and_data(void);               //发布指令和板间通信
void mousekey_GimbalChassis_default(void);  //底盘和云台的默认状态

// 其他功能函数
float   get_offset_angle(short init_forward, short now_encoder);  // 获取云台朝向与底盘正前的夹角
void    Gimbal_Refreeget(CANInstance *instance);
static	Robot_Status_e robot_state; // 机器人整体工作状态
static	Robot_Status_e last_robot_state; // 机器人整体工作状态
	/* 外设  todo: 添加蜂鸣器 PC虚拟串口 */
static RC_Ctl_t *rc_data;   // 遥控器数据指针
static attitude_t *ins;

   /* 云台部分 */
static	Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static	Subscriber_t *gimbal_upload_sub;        // 云台反馈信息订阅者
static	Gimbal_ctrl_cmd_t gimbal_control;       // 传递给云台的控制信息
static	Gimbal_upload_t gimbal_upload_data;     // 从云台获取的反馈信息
    
   /* 发射部分 */
static	Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static	Subscriber_t *shoot_upload_sub;         // 发射反馈信息订阅者
static	Shoot_ctrl_cmd_t shoot_control;       // 传递给发射的控制信息
static	Shoot_upload_t   shoot_upload_data;     // 从发射获取的反馈信息
    
static	uint8_t Soft_Reset_Flag; //软重启状态量
static	int16_t soft_reset_cnt;
    
	/* 双板通信 */
static	Gimbal_board_send_t   send_data;
static	Gimbal_board_send_UI_t  send_UI;
static	Chassis_board_send_t  receive_data;
	/* 自瞄模式 */
Aim_Action_e AimAction = AIM_STOP;   // 手动自瞄开关

void Gimbal_Cmd_Init()
{
	// 定义发布者订阅者
	gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_ctrl_cmd_t) );
	gimbal_upload_sub = SubRegister("gimbal_upload", sizeof(Gimbal_upload_t) );
	shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_ctrl_cmd_t) );
	shoot_upload_sub = SubRegister("shoot_upload", sizeof(Shoot_upload_t));
	
	/* 上下板通信 */
	CANComm_Init_Config_s comm_conf = {
		.can_config = {
			.can_handle = &hcan1,
			.tx_id = 0x110,
			.rx_id = 0x101,
			.can_module_callback = Gimbal_Refreeget,
		},
		.recv_data_len = sizeof(Chassis_board_send_t),
		.send_data_len = sizeof(Gimbal_board_send_t),
		.dog_config = {
			.Max_num = 10,
			.dog_name = "UPBOARD",
		},
	};
	can_cmd_comm = CANCommInit(comm_conf);
    
	robot_state = ROBOT_READY; //     //初始化为ready
    last_robot_state = ROBOT_STOP;
	Soft_Reset_Flag = 0;
		
    rc_data = get_remote_control_point();
	ins     = INS_Init();
}

void Gimbal_board_CMD_Update()
{
    static uint16_t tim; // 用于云台归中
	// 板间通信 -收
	receive_data = *(Chassis_board_send_t *)CANCommGet(can_cmd_comm);
    if (can_cmd_comm->Dog->state != Dog_Online) {
		gimbal_control.rotate_feedforward = 0;  // 底盘小陀螺前馈
		shoot_control.bullet_speed_fdb = 0;     // 实时弹速
	} else {
		gimbal_control.rotate_feedforward = receive_data.chassis_gyro;
		shoot_control.bullet_speed_fdb  = receive_data.shoot_referee_data.bullet_speed_now;
		shoot_control.heat_limit_remain = receive_data.shoot_referee_data.heat_limit_remain;
	}
	// 判断云台模块  todo : 与PC通信
	SubGetMessage(gimbal_upload_sub, (void *)&gimbal_upload_data);
    if (gimbal_upload_data.gimbal_status != Device_Online)
	  {
		gimbal_control.mode = GIMBAL_ZERO_FORCE;
	  }
	  // 软件重启
	  if (Soft_Reset_Flag > 0)
	       robot_state = ROBOT_STOP;
	  if (Soft_Reset_Flag == 1) {
		   static int16_t robot_stop_cnt = 0;
           robot_stop_cnt ++;
		  // 等待20ms使得电机全部下线
		   if(robot_stop_cnt >= 20) {
               Soft_Reset_Flag = 2;
               send_data.Close_flag = 1;
		   }
	  } else if (Soft_Reset_Flag == 2) {
        //等待10ms使得can通信完成
        static int16_t can_wait_cnt = 0;
        can_wait_cnt++;
        if (can_wait_cnt == 10) 
             soft_rest();
	  }
	  
	  /* 发射部分 */
	  // 获取实际弹速  todo 用于视觉
	SubGetMessage(shoot_upload_sub, (void* )&shoot_upload_data);
    if (shoot_upload_data.shoot_status != Device_Online)
		shoot_control.mode = SHOOT_STOP;
	// 遥控器判断
	if ( rc_data->Remote_dog->state != Dog_Online)
	    robot_state = ROBOT_STOP;
	if ( rc_data->Remote_dog->state == Dog_Online && robot_state == ROBOT_STOP && Soft_Reset_Flag == 0){  // 恢复任务
		    robot_state = ROBOT_READY;
	}
    
	// 遥控器离线模式
	if (robot_state == ROBOT_STOP)
		stop_mode_update();
	  
     else if(robot_state == ROBOT_READY) {
		 if(gimbal_upload_data.gimbal_status == Device_Online) 
		  {
			tim++;
			if (tim < 30)
			{
	#if   Yaw_Mid_Right < Yaw_Mid_Left
			if ( (gimbal_upload_data.yaw_encorder <= Yaw_Mid_Left) && (gimbal_upload_data.yaw_encorder >= Yaw_Mid_Right) )
	#elif Yaw_Mid_Right > Yaw_Mid_Left
			if ( (gimbal_upload_data.yaw_encorder <= Yaw_Mid_Left) || (gimbal_upload_data.yaw_encorder >= Yaw_Mid_Right) )
	#endif
				gimbal_control.Mid_mode = FRONT;
			else
				gimbal_control.Mid_mode = BACK;
			gimbal_control.mode = GIMBAL_ZERO_FORCE;
			gimbal_control.Mech_Ref.Pitch = Pitch_Mid;
			} else if (tim < 1000 ) {
				gimbal_control.mode = GIMBAL_MIDDLE;  // 云台归中 归中时先使用机械模式  后续考虑使用陀螺仪
			} else {
				  tim = 0;
				  robot_state = Robot_RUNNING;
				  gimbal_control.mode = GIMBAL_GYRO_MODE; // 跟随模式
				  gimbal_control.Gyro_Ref.Yaw   = ins->ContinuousYaw;  /* 陀螺仪模式 */ 
				  gimbal_control.Gyro_Ref.Pitch = ins->Roll;
			   }
	     }   
	 } else if(robot_state == Robot_RUNNING) {		 
		 /* 向底盘发送角度差 */
		switch (rc_data->RemoteMode) {
			case REMOTE_INPUT:        	// 遥控器控制模式
				remote_mode_update();send_data.Close_flag = 0; break;
			case KEY_MOUSE_INPUT:
				mouse_key_mode_update();send_data.Close_flag = 0; break;
			case STOP :    // 急停  todo : 修改为期望速度为零
				gimbal_control.mode  = GIMBAL_ZERO_FORCE;
				shoot_control.mode   = SHOOT_ZERO_FORCE;
				send_UI.chassis_mode = CHASSIS_ZERO_FORCE;
			    gimbal_control.Gyro_Ref.Yaw   = ins->ContinuousYaw;  /* 陀螺仪模式 */
			    gimbal_control.Gyro_Ref.Pitch = ins->Roll;
			    send_data.Close_flag = 1;
			 break;
		}
	 }
	 /* 裁判系统UI数据 */
//	 send_data.gimbal_mode = gimbal_upload_data.mode;
//	 send_data.shoot_mode  = shoot_upload_data.mode;
//	 send_data.bullet_mode = shoot_upload_data.bullet_mode;
//	 send_data.now_robot_mode = robot_state;
	 
	 // 根据底盘模式进行计算底盘的旋转速度 获取云台补偿角度
	 gimbal_control.chassis_mode = send_UI.chassis_mode;
	 gimbal_control.shoot_mode = shoot_control.mode;
	// 发布数据
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_control); 
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_control);
	gimbal_control.last_mode = gimbal_control.mode;
	 /**  底盘信息发射  **/
#if CHASSIS_MOVE == 0
send_data.Close_flag = 1;
send_data.vx = 0;
send_data.vy = 0;
send_data.rotate = 0;
#endif
//  	 CANCommSend(can_cmd_comm, (void *)&send_data);
	 CAN_Send_StdDataFrame(&hcan1, 0x110, (uint8_t *)&send_data);
}

/**  软件软重启  **/ 
void soft_rest()
{
    // 关闭所有中断
    __set_FAULTMASK(1); 
    // 复位
    NVIC_SystemReset();
}

/* 机器人停止模式 */
void stop_mode_update()
{
	// 关闭自身模块
	gimbal_control.mode = GIMBAL_ZERO_FORCE;
	shoot_control.mode  = SHOOT_ZERO_FORCE;
	// 关闭底盘模块
	send_data.Close_flag = 1;
//    send_data.chassis_mode = CHASSIS_ZERO_FORCE;
}

// 机器人等级 用于适应不同功率
  static  float Level_Gain;
  static  float chassis_offset; // 底盘补偿角度
   
void remote_mode_update()
{
	int16_t left_right_ref = 0,forward_back_ref = 0;
	
	/* 云台前馈值 */
    gimbal_control.Feedback_Speed.Yaw = 0;
	gimbal_control.Feedback_Speed.Pitch = 0;

	/* 遥控器控制模式 */
	switch (rc_data->rc.s1) {
		case 1:
			if(ins->ins_dog->state == Dog_Online) {
				gimbal_control.mode  = GIMBAL_MIDDLE;
			    send_UI.chassis_mode = CHASSIS_NORMAL;
			} else {
				gimbal_control.mode  = 0;
			    send_UI.chassis_mode = 0;
			}
		   shoot_control.mode = SHOOT_NOMAL;
		   shoot_control.fire_rate = 1; // 设置弹频
		   AimAction = AIM_STOP;
		break;
	    case 3:
			if(ins->ins_dog->state == Dog_Online) {
				gimbal_control.mode  = GIMBAL_MIDDLE;
			    send_UI.chassis_mode = CHASSIS_NORMAL;
			} else {
				gimbal_control.mode  = 0;
			    send_UI.chassis_mode = 0;
			}
		   
	       shoot_control.mode = SHOOT_READY;
		   AimAction = AIM_STOP;
			break;
	    case 2:
			if(ins->ins_dog->state == Dog_Online) {
				gimbal_control.mode  = GIMBAL_GYRO_MODE;
			    send_UI.chassis_mode = CHASSIS_FOLLOW;
			} else {
				gimbal_control.mode  = 0;
			    send_UI.chassis_mode = 0;
			}
		   shoot_control.mode = SHOOT_STOP;
		   AimAction = AIM_STOP;
		 break;
	}
	
	/* 陀螺仪模式 */
	if(gimbal_control.mode == GIMBAL_GYRO_MODE && AimAction == AIM_STOP) {
        gimbal_control.Gyro_Ref.Yaw -= rc_data->Key_CH[2] * DR16_Rocker_Yaw_Resolution * 1.2f;
        gimbal_control.Feedback_Speed.Yaw -= rc_data->Key_CH[2] * DR16_Rocker_Yaw_Resolution * 1000.0f;
        gimbal_control.Gyro_Ref.Pitch += rc_data->Key_CH[3] * DR16_Rocker_Pitch_Resolution;
        gimbal_control.Feedback_Speed.Pitch += rc_data->Key_CH[3] * DR16_Rocker_Pitch_Resolution * 1000.0f;
	    limit(gimbal_control.Gyro_Ref.Pitch, IMU_UP_limit, IMU_DOWN_limit);
		 /* 如果底盘跟随很慢 及时切换零点 */
#if     Yaw_Mid_Right < Yaw_Mid_Left
        if ( (gimbal_upload_data.yaw_encorder <= Yaw_Mid_Left) && (gimbal_upload_data.yaw_encorder >= Yaw_Mid_Right)){
#elif   Yaw_Mid_Right > Yaw_Mid_Left
        if ( (gimbal_upload_data.yaw_encorder <= Yaw_Mid_Left) || (gimbal_upload_data.yaw_encorder >= Yaw_Mid_Right)){
#endif
			gimbal_control.Mid_mode = FRONT;
		} else
		   	gimbal_control.Mid_mode = BACK;
	} else {
		gimbal_control.Gyro_Ref.Yaw   = ins->ContinuousYaw;
		gimbal_control.Gyro_Ref.Pitch = ins->Roll;
	}
	
	 /** 归中模式控制 **/
	if(gimbal_control.mode == GIMBAL_MIDDLE) {
		gimbal_control.Mech_Ref.Pitch += rc_data->Key_CH[3] * 0.36f * 5.0f;;
		limit(gimbal_control.Mech_Ref.Pitch, 6000, 4800);
	} else {
		gimbal_control.Mech_Ref.Pitch = gimbal_upload_data.pitch_encorder;
	}
	
	/* 底盘参数控制 遥控器底盘功能调试*/
	send_UI.chassis_dispatch_mode = (uint8_t )chassis_dispatch_without_acc_limit; // 修改功率模式 调试用
	
    /* 速度上限与加减速规划控制 */
	if (send_UI.chassis_mode != CHASSIS_ZERO_FORCE) {
	    /* 获取底盘旋转速度 */
		send_data.rotate = gimbal_upload_data.rotate_speed * 2.0f;
		// 小陀螺时平移速度削减
		 Level_Gain = 5.5f + receive_data.shoot_referee_data.robot_level * 0.5f;
		 if (send_data.rotate <= 8000 * PI) {
		     left_right_ref = (float )rc_data->Key_CH[0] * Level_Gain * 3000;
		     forward_back_ref = (float )rc_data->Key_CH[1] * Level_Gain * 3000;
		} else {
		     left_right_ref = (float )rc_data->Key_CH[0] * Level_Gain * (1 - send_data.rotate / 10000.0f) * 0.5f * 4000;
		     forward_back_ref = (float )rc_data->Key_CH[1] * Level_Gain * (1 - send_data.rotate / 10000.0f) * 0.5f * 4000;
		}
	}
	
	 /* 底盘补偿(小陀螺模式也能正常移动) */
	static float sin_theta, cos_theta; // 利用YAW电机回传角度 求解底盘相对于云台的角度
	chassis_offset = (gimbal_upload_data.yaw_encorder - Yaw_Mid_Front) / 1303.80f;
	sin_theta = sinf(chassis_offset);
	cos_theta = cosf(chassis_offset);
	
    send_data.vy  =  left_right_ref * cos_theta - forward_back_ref * sin_theta;  //  left_right_ref 
	send_data.vx  =  left_right_ref * sin_theta + forward_back_ref * cos_theta;  // forward_back_ref
	
	/* 底盘平移前馈补偿 */
//	float sin_yaw_feedforward, cos_yaw_feedforward;
//	cos_yaw_feedforward = cosf(chassis_offset - receive_data.chassis_gyro * 0.10f);
//	sin_yaw_feedforward = sinf(chassis_offset - receive_data.chassis_gyro * 0.10f);
//	send_data.vy =  left_right_ref * cos_yaw_feedforward - forward_back_ref * sin_yaw_feedforward;
//	send_data.vx =  left_right_ref * sin_yaw_feedforward + forward_back_ref * cos_yaw_feedforward;

    /* 裁判系统返回热量 */
    shoot_control.bullet_speed = receive_data.shoot_referee_data.bullet_speed_now;    // 裁判系统返回弹速
    shoot_control.bullet_mode = BULLET_SINGLE;
    shoot_control.heat_limit_remain = receive_data.shoot_referee_data.heat_limit_remain;  // 剩余热量
}


/**  键鼠控制  **/
void mouse_key_mode_update()
{
	/* 云台前馈值 */
    gimbal_control.Feedback_Speed.Yaw = 0;
	gimbal_control.Feedback_Speed.Pitch = 0;

	if (gimbal_control.last_mode != gimbal_control.mode && gimbal_control.mode != GIMBAL_GYRO_MODE) {
	      gimbal_control.Gyro_Ref.Yaw   = ins->ContinuousYaw;  /* 陀螺仪模式 */
		  gimbal_control.Gyro_Ref.Pitch = ins->Roll;  
	}
	static uint8_t mouse_r_flag, normal_action;
	Gimbal_board_send_t *chassis = &send_data;
	Shoot_ctrl_cmd_t *shoot = &shoot_control;

    if (send_UI.chassis_mode != CHASSIS_ZERO_FORCE) {
		 Level_Gain = 4.5 + receive_data.shoot_referee_data.robot_level;
		 if (send_data.rotate <= 8000*PI) {
		chassis->vy = rc_data->key[KEY_PRESS].W  - rc_data->key[KEY_PRESS].S ;
		chassis->vx = rc_data->key[KEY_PRESS].A  - rc_data->key[KEY_PRESS].D ;
		} else {
		chassis->vy = rc_data->key[KEY_PRESS].W - rc_data->key[KEY_PRESS].S * (1 - send_data.rotate / 10000.0f);
		chassis->vx = rc_data->key[KEY_PRESS].A - rc_data->key[KEY_PRESS].D * (1 - send_data.rotate / 10000.0f);
		}
	}
     
	/* 底盘补偿(小陀螺模式也能正常移动)*/
	chassis_offset = -(gimbal_upload_data.yaw_encorder - Yaw_Mid_Front) / 1303.80f;
	send_data.vy =  send_data.vy * cosf(chassis_offset) + send_data.vx * sinf(chassis_offset);
	send_data.vx = -send_data.vy * sinf( chassis_offset) + send_data.vx * cosf(chassis_offset);
    // R:底盘模式
	if (send_UI.chassis_mode != CHASSIS_ZERO_FORCE )
	  switch (rc_data->key_count[KEY_PRESS][Key_R] % 3) {
		case 0:
             mousekey_GimbalChassis_default();
		     break;
		case 1:
		   	send_UI.chassis_mode = CHASSIS_SPIN;  // 小陀螺模式
            gimbal_control.mode = GIMBAL_GYRO_MODE;
            break;
		default :
		   	send_UI.chassis_mode = CHASSIS_NORMAL; //云台底盘分离
	   	    gimbal_control.mode = GIMBAL_GYRO_MODE;
		    break;
	  }
      // x:修改底盘功率模式
	   	switch (rc_data->key_count[KEY_PRESS][Key_X] % 3) {
			case 1:   //飞坡模式 暂时去掉云台跟随底盘 强制修改为底盘跟随云台模式 方便飞坡时控制方向
			    send_UI.chassis_dispatch_mode = chassis_dispatch_fly;
			    send_UI.chassis_mode = CHASSIS_FOLLOW;
                gimbal_control.mode = GIMBAL_GYRO_MODE;
			    break;
			case 2 :
                mousekey_GimbalChassis_default();                              
			    send_UI.chassis_dispatch_mode = chassis_dispatch_climb; // 爬坡模式
			    break;
			default:
                mousekey_GimbalChassis_default();
			break;
		}
		
		// 鼠标右键开启摩擦轮
		if (rc_data->mouse.press_r && mouse_r_flag == 0) {
			if (shoot->mode ==  SHOOT_STOP)
		          shoot->mode = SHOOT_READY;
		     else
				 shoot->mode = SHOOT_STOP;
		   mouse_r_flag = 1;
		}
		
		if (rc_data->mouse.press_r == 0)
			 mouse_r_flag = 0;
		 if (rc_data->mouse.press_l == 1) {
			  shoot->mode = SHOOT_NOMAL;
 		  	  normal_action++ ;
		 }
		if (normal_action >= 250)
			 shoot->bullet_mode = BULLET_CONTINUE;
		
		// T: 修改发射模式
		if (shoot->bullet_mode != BULLET_CONTINUE)
		  switch (rc_data->key_count[KEY_PRESS][Key_C] % 4) {
			  case 0 :
				  shoot->bullet_mode = BULLET_SINGLE; // 单发
				  break;
			  case 1:
				  shoot->bullet_mode = BULLET_DOUBLE;
			      break;
			  default:
				  shoot->bullet_mode = BULLET_TRIBLE;
				  break;
		   }
		   
		// F: 自瞄模式 
//		if (rc_data->key_count[KEY_PRESS][Key_F] % 2) {
//			if (autoaim_mode != auto_aim_normal )
//				 autoaim_mode = auto_aim_normal;
//	        else autoaim_mode = auto_aim_off;
//		}
		
		// shift加速 允许超功率 长按
		if (rc_data->key[KEY_PRESS].Shift) {
			chassis->vx *= 1.5f;
			chassis->vy *= 1.5f;
		}
	   // Q/E 底盘快速转向
		if (rc_data->key[KEY_PRESS].Q) {
			if (send_UI.chassis_mode == CHASSIS_FOLLOW || send_UI.chassis_mode == CHASSIS_SPIN)
			gimbal_control.Gyro_Ref.Yaw += 0.307f;
			else chassis->rotate += 2000 ;  // 后续改为功率控制
		}
		if (rc_data->key[KEY_PRESS].E) {
			if (send_UI.chassis_mode == CHASSIS_FOLLOW || send_UI.chassis_mode == CHASSIS_SPIN)
			gimbal_control.Gyro_Ref.Yaw -= 0.307f;
			else chassis->rotate -= 2000 ;  // 后续改为功率控制
		}
		
		// V: 快速转头 点按
		if (rc_data->key[KEY_PRESS].V != rc_data->Lastkey[KEY_PRESS].V ) {
			if (gimbal_control.Mid_mode == FRONT )
				 gimbal_control.Mid_mode = BACK;
	        else gimbal_control.Mid_mode = FRONT;
			gimbal_control.Gyro_Ref.Yaw += 180;
		}
	
		   /* 陀螺仪模式 */
	if( gimbal_control.mode == GIMBAL_GYRO_MODE ) {
        gimbal_control.Gyro_Ref.Yaw -= rc_data->Mouse_Ch[0]  * DR16_Mouse_Yaw_Angle_Resolution;
		gimbal_control.Feedback_Speed.Yaw -= rc_data->Mouse_Ch[0] * DR16_Mouse_Yaw_Angle_Resolution * 1000.0f;
		gimbal_control.Gyro_Ref.Pitch += rc_data->Mouse_Ch[1] * DR16_Mouse_Pitch_Angle_Resolution;
		gimbal_control.Feedback_Speed.Pitch -= rc_data->Mouse_Ch[1] * DR16_Mouse_Pitch_Angle_Resolution * 1000.0f;
	    limit(gimbal_control.Gyro_Ref.Pitch, IMU_UP_limit, IMU_DOWN_limit);
		 /* 如果底盘跟随很慢 及时切换零点 */
#if   Yaw_Mid_Right < Yaw_Mid_Left
        if ( (gimbal_upload_data.yaw_encorder <= Yaw_Mid_Left) && (gimbal_upload_data.yaw_encorder >= Yaw_Mid_Right) ){
#elif Yaw_Mid_Right > Yaw_Mid_Left
        if ( (gimbal_upload_data.yaw_encorder <= Yaw_Mid_Left) || (gimbal_upload_data.yaw_encorder >= Yaw_Mid_Right) ){
#endif
			gimbal_control.Mid_mode = FRONT;
		} else {
			gimbal_control.Mid_mode = BACK;
		}
	} else {
	    gimbal_control.Gyro_Ref.Yaw   = ins->ContinuousYaw;
		gimbal_control.Gyro_Ref.Pitch = ins->Roll;
	}
	   /** 归中模式控制 **/
	if ( gimbal_control.mode == GIMBAL_MIDDLE) {
		gimbal_control.Mech_Ref.Pitch += rc_data->Mouse_Ch[1] * 0.36f * 10.5f;;
        limit(gimbal_control.Mech_Ref.Pitch, 1750, 0);
	} else {
		gimbal_control.Mech_Ref.Pitch  = gimbal_upload_data.pitch_encorder;
	}
}

// 云台底盘模式重置
void mousekey_GimbalChassis_default() {
    gimbal_control.mode =  GIMBAL_ZERO_FORCE;
    send_UI.chassis_mode = CHASSIS_ZERO_FORCE;
    send_UI.chassis_dispatch_mode = chassis_dispatch_mild;
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 */
float get_offset_angle(short init_forward, short now_encoder) {
    short tmp = 0;
    if (init_forward < 4096) {
        if (now_encoder > init_forward && now_encoder <= 4096 + init_forward) {
            tmp = now_encoder - init_forward;
        } else if (now_encoder > 4096 + init_forward) {
            tmp = -8192 + now_encoder - init_forward;
        } else {
            tmp = now_encoder - init_forward;
        }
    } else {
        if (now_encoder > init_forward) {
            tmp = now_encoder - init_forward;
        } else if (now_encoder <= init_forward && now_encoder >= init_forward - 4096) {
            tmp = now_encoder - init_forward;
        } else {
            tmp = now_encoder + 8192 - init_forward;
        }
    }
    return tmp * 360.0 / 8192.0;
}

void Gimbal_Refreeget(CANInstance *_instance)
{
	CANCommInstance *comm = (CANCommInstance *)_instance->id;
	
	memcpy(&receive_data, _instance->rx_buff, sizeof(Chassis_board_send_t));
	comm->recv_state = 0;
	comm->cur_recv_len = 0;
	comm->update_flag = 1;
	Feed_Dog(comm->Dog);
}
