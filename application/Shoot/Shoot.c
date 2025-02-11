#include "Shoot.h"
#include "RMQueue.h"
#include "motor.h"
#include "PID.h"
#include "user_lib.h"

/*  发射部分 */
static float fire_speed_nofil[3] = {0};

static DJIMotor_Instance *friction_r, *friction_l, *friction_u, *loader;
static Publisher_t      *shoot_pub;          // 云台控制消息发布者
static Subscriber_t     *shoot_sub;          // 云台反馈信息订阅者
static Shoot_ctrl_cmd_t *shoot_ctrl_cmd;     // 云台接收CMD的控制信息
static Shoot_upload_t    shoot_feedback_data;// 云台发布的反馈信息

/* 用于发射 */
static float Angle_Target = 0;      //!< @brief 拨弹盘期望角度
static float RAMP_Angle_Target = 0; //!< @brief 拨弹盘期望斜坡角度
static uint32_t cooldown_start; // 冷却起始时间点
static uint32_t cooldown_tim;   // 冷却时间
// 补偿控制
//RMQueue_Handle *bullet_speed_queue;
static uint32_t shoot_time;  // 发射时间计数
static uint16_t buttle_cnt;
static uint16_t last_bullet_cnt;
float shoot_speed_reduce;
/**  其他功能函数  **/
void Shoot_load_Update(void);     // 拨弹盘控制
void Shoot_friction_Update(void); // 摩擦轮控制
void Shoot_Calc(void);
void Shoot_STOP(void);

static Smc shoot_smc[3];
static PID Shoot_Speed_PID[3] = {{.Kp = 15, .Ki = 0.0f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 16000, .DeadBand = 0.50f, .inter_threLow = 500, .inter_threUp = 1000},            //摩擦轮左
                          {.Kp = 15, .Ki = 0.0f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 16000, .DeadBand = 0.50f, .inter_threLow = 500, .inter_threUp = 1000},                  //摩擦轮右
						  {.Kp = 15, .Ki = 0.0f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 16000, .DeadBand = 0.50f, .inter_threLow = 500, .inter_threUp = 1000}};                 // 摩擦轮上

static PID_Smis Pluck_Place_PIDS = {.Kp = 8.0f, .Ki = 0, .Kd = 0.5f, .interlimit = 3000, .outlimit = 16000, .DeadBand = 0.0f, .inter_threLow = 500, .inter_threUp = 1000};         //拨弹盘单发位置环
static PID Pluck_Speed_PID = {.Kp = 15.0f, .Ki = 0.80f, .Kd = 0.0f, .interlimit = 4000, .outlimit = 15000, .DeadBand = 0.0f, .inter_threLow = 20, .inter_threUp = 5000};                   //拨弹盘单发速度环
static PID Pluck_Continue_PID = {.Kp = 15, .Ki = 0, .Kd = 0, .interlimit = 3000, .outlimit = 16000, .DeadBand = 5.0f, .inter_threLow = 500, .inter_threUp = 1000};               //拨弹盘连发模式

void Shoot_Init()
{
    SMC_SetConfig(&shoot_smc[0].config, 3.0f, 20, 125, ReachingLaw_sqrt, 16000);
    SMC_SetConfig(&shoot_smc[1].config, 3.0f, 20, 125, ReachingLaw_sqrt, 16000);
    SMC_SetConfig(&shoot_smc[2].config, 3.0f, 20, 125, ReachingLaw_sqrt, 16000);
    
	shoot_pub = PubRegister("shoot_upload", sizeof(Shoot_upload_t));
	shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_ctrl_cmd_t));

Motor_Init_config_s loader_config = {
	.motor_type = RM3508,
	.dog_init_config = {
		.dog_name = "loader",
		.GPIO_Pin = GPIO_PIN_1,
	},
	.can_init_config = {
		.can_handle = &hcan1,
		.tx_id = 1,
		.rx_id = 201,
	},
	.contruller_config = {
		.MotorCenter = 0,
		.setting_config = {
			.feedback_reverse_flag = FEEDBACK_NORMAL,
			.motor_reverse_flag =  MOTOR_NORMAL,    // 电机不反转   
			.angle_feedback_source = MOTOR_FEED,    // 云台为MID归中模式时使用机械角度
			.speed_feedback_source = MOTOR_FEED,    // 云台初始化后使用 DJIMotorChangeFeed函数修改反馈类型使用IMU控制
			.close_loop_type = ANGLE_LOOP,
			.feedfoward_type = NONE_FEEDFORWARD, // 不使用前馈类型
			.control_mode = PID_MODEL,
		}
	},
	.other_angle_feedback_ptr = 0,
	.other_speed_feedback_ptr = 0,
};
	loader = DJI_Motor_create(&loader_config);

Motor_Init_config_s friction_u_config = {
	.motor_type = RM3508,
	.dog_init_config = {
		.dog_name = "fire_u",
	    .GPIO_Pin = GPIO_PIN_1,
	},
	.can_init_config = {
		.can_handle = &hcan1,
		.tx_id = 4,
		.rx_id = 204,
	},
	.contruller_config = {
		.setting_config = {
			.motor_reverse_flag = MOTOR_NORMAL,   // 电机不反转
			.feedback_reverse_flag = FEEDBACK_NORMAL,
			.angle_feedback_source = MOTOR_FEED,
			.speed_feedback_source = MOTOR_FEED,    // 云台初始化后使用 DJIMotorChangeFeed函数修改反馈类型使用IMU控制
		.close_loop_type = SPEED_LOOP,          // 使用闭环类型 默认位置环位最外环
		.feedfoward_type = NONE_FEEDFORWARD, // 不使用前馈类型 
		.control_mode = PID_MODEL,
		},
	},
	.other_angle_feedback_ptr = 0,
	.other_speed_feedback_ptr = 0,
};
	friction_u = DJI_Motor_create(&friction_u_config);

Motor_Init_config_s friction_r_config = {
	.motor_type = RM3508,
	.dog_init_config = { 
		.dog_name = "fire_r",
		.GPIO_Pin = GPIO_PIN_1,
	},
	.can_init_config = {
		.can_handle = &hcan1,
		.tx_id = 3,
		.rx_id = 203,
	},
	.contruller_config = {
		.setting_config = {
			.motor_reverse_flag = MOTOR_NORMAL,   // 电机不反转
			.feedback_reverse_flag = FEEDBACK_NORMAL,
			.angle_feedback_source = MOTOR_FEED,
			.speed_feedback_source = MOTOR_FEED,    // 云台初始化后使用 DJIMotorChangeFeed函数修改反馈类型使用IMU控制
		.close_loop_type = SPEED_LOOP,          // 使用闭环类型 默认位置环位最外环
		.feedfoward_type = NONE_FEEDFORWARD, // 不使用前馈类型 
		.control_mode = PID_MODEL,
		},
	},
	.other_angle_feedback_ptr = 0,
	.other_speed_feedback_ptr = 0,
};
  friction_r = DJI_Motor_create(&friction_r_config);
  
   Motor_Init_config_s friction_l_config = {
	.motor_type = RM3508,
	.dog_init_config = {
		.dog_name = "fire_l",
	    .GPIO_Pin = GPIO_PIN_1,
	},
	.can_init_config = {
		.can_handle = &hcan1,
		.tx_id = 2,
		.rx_id = 202,
	},
    .contruller_config = {
		.setting_config = {
		.motor_reverse_flag = MOTOR_NORMAL,   // 电机不反转
		.feedback_reverse_flag = FEEDBACK_NORMAL,
		.angle_feedback_source = MOTOR_FEED,
		.speed_feedback_source = MOTOR_FEED,    // 云台初始化后使用 DJIMotorChangeFeed函数修改反馈类型使用IMU控制
		.close_loop_type = SPEED_LOOP,          // 使用闭环类型 默认位置环位最外环
		.feedfoward_type = NONE_FEEDFORWARD, // 不使用前馈类型 
		.control_mode = PID_MODEL,
	  },
	},
	.other_angle_feedback_ptr = 0,
	.other_speed_feedback_ptr = 0,
};
  friction_l = DJI_Motor_create(&friction_l_config);
  
  // 订阅者初始化
  shoot_ctrl_cmd = (Shoot_ctrl_cmd_t *)malloc(sizeof(Shoot_ctrl_cmd_t));
  memset(shoot_ctrl_cmd, 0, sizeof(Shoot_ctrl_cmd_t));
    cooldown_start = cooldown_tim = 0;
}
/*
	PubPushMessage(gimbal_pub, (void *)&gimbal_feed);
	SubGetMessage(gimbal_sub, gimbal_get_control);
*/
float abs_deltePlack, abs_deltaSpeed, loader_angle_deg;
int32_t loader_angle, loader_target, loader_round;
void Shoot_load_Update()
{
    static uint16_t  Stuck_time   = 0;                   //!< @brief 检测卡弹的时间
	static uint8_t Add_Angle_Flag = 0;               	 //!< @brief 拨弹盘角度+1标志位
    static float Stuck_Angle_Target = 0;                 //!< @brief 拨弹盘卡弹退弹期望角度
    static const uint16_t Stuck_thre = 500;              //!< @brief 卡弹阈值，卡弹超过此时间便认为卡弹
    static float Last_Target_Pluck = 0, Off_Angle = 0, continur_tims;  //!< @brief 发弹补偿
	static float Target_Continue_Speed = -360.0f * REDUCTION_RATIO_WHEEL / SHOOT_NUM_PER_CIRCLE * 0.99f;
    
	shoot_ctrl_cmd->heat_limit_remain = 1000;  // 测试用
	
	// 设定拨弹盘转动角度
//	static float load_delta_pos = -360.0f * SHOOT_MOTOR_DECELE_RATIO / SHOOT_NUM_PER_CIRCLE * 1.02f;
	static int16_t load_delta_pos = -1385;
    /* 暂停模式 */	
	if (shoot_ctrl_cmd->mode == SHOOT_STOP) {
		shoot_ctrl_cmd->bullet_mode = BULLET_HOLDON;
		/** 卡弹时间清零  **/
		Stuck_time = 0; 
        ABS(Last_Target_Pluck) <= 100 ? Off_Angle = 0 
		    : ( Off_Angle = (int16_t )(loader->measure.Angle_DEG - Last_Target_Pluck) % (int16_t)load_delta_pos);
		DJIMotorOuterLoop(loader, ANGLE_LOOP);   // 修改为角度控制
		DJIMotorSetRef(loader, Angle_Target);
	} else {
		/**  连发设置  **/
		if(shoot_ctrl_cmd->mode == SHOOT_NOMAL) {
			continur_tims ++;
			if(continur_tims >= 800) {
			   shoot_ctrl_cmd->bullet_mode = BULLET_CONTINUE;
			   Stuck_time = 0;
			}
		} else
			continur_tims  = 0; 
		/** 判断卡弹时间 **/ 
		if(Stuck_time >= Stuck_thre)
			shoot_ctrl_cmd->mode = SHOOT_STUCKING;
		/** 卡弹检测 **/
	   if(shoot_ctrl_cmd->bullet_mode != BULLET_CONTINUE) {
		   abs_deltaSpeed = fabs(loader->measure.SpeedFilter);
		   abs_deltePlack = fabs(Angle_Target - loader->measure.Angle_DEG);
		   if ( abs_deltePlack > abs(load_delta_pos * 0.5f) && abs_deltaSpeed <= 50.0f )
			   Stuck_time ++;
		   else
			   Stuck_time = 0;
			/* 卡弹时可能卡在摩擦轮中 */
		  if(shoot_ctrl_cmd->mode == SHOOT_READY) {
			 if(abs(friction_l->measure.Speed) < 20 && abs(friction_r->measure.Speed) < 20 && abs(friction_u->measure.Speed) < 20)
			     Stuck_time ++;
		     else
				 Stuck_time = 0;
		   }
	   } else if(shoot_ctrl_cmd->bullet_mode == BULLET_CONTINUE) {
		   if (abs_deltaSpeed <= 50.0f)  // 连发时速度很小
				Stuck_time ++;
			else
				Stuck_time = 0;
	   }
	   
	   /** 卡弹反转  **/
	  if(shoot_ctrl_cmd->mode == SHOOT_STUCKING) 
	  {
	    if(Stuck_time == Stuck_thre || Stuck_time == Stuck_thre+1 ) { // 快速退半发的位置
		  Stuck_Angle_Target = loader->measure.Angle_DEG - 0.6f * load_delta_pos;
	  	  DJIMotorOuterLoop(loader, ANGLE_LOOP); // 修改为角度控制
		  DJIMotorSetRef(loader, Stuck_Angle_Target);
	    }
	    Stuck_time ++;
	    if(Stuck_time >= Stuck_thre * 3.0f)
		    Stuck_time = 0;
	  }
	  else 
	  {
		/* 增加打弹标志位 */ 
		if (shoot_ctrl_cmd->mode == SHOOT_READY)
			  Add_Angle_Flag = 1;
		/* 开始打弹 */
		if (Add_Angle_Flag == 1 && shoot_ctrl_cmd->mode == SHOOT_NOMAL && shoot_ctrl_cmd->bullet_mode != BULLET_CONTINUE) {
			switch (shoot_ctrl_cmd->bullet_mode) {
				case BULLET_HOLDON :
					DJIMotorOuterLoop(loader, ANGLE_LOOP); // 修改为角度控制
					Angle_Target = loader->measure.Angle_DEG;
					DJIMotorSetRef(loader, Angle_Target);
				break;
				
				case BULLET_SINGLE : // 单发
					DJIMotorOuterLoop(loader, ANGLE_LOOP);
					Angle_Target = loader->measure.Angle_DEG + load_delta_pos;
					cooldown_tim   = 100; // 时间间隔
				break;
				
				case BULLET_DOUBLE: // 双发
					DJIMotorOuterLoop(loader, ANGLE_LOOP);
					Angle_Target = loader->measure.Angle_DEG + load_delta_pos * 2;
					cooldown_tim = 200;
				break;
				
				case BULLET_TRIBLE:  // 三连发
					DJIMotorOuterLoop(loader, ANGLE_LOOP);
					Angle_Target = loader->measure.Angle_DEG + load_delta_pos * 3;
					cooldown_tim = 300;
				break;
				
				default:
				  DJIMotorOuterLoop(loader, ANGLE_LOOP);  // 修改为角度控制
				  Angle_Target = loader->measure.Angle_DEG;
				break;
			}
			Last_Target_Pluck = Angle_Target;
			Add_Angle_Flag = 0;
			Off_Angle = 0;
		}
		/** 设置拨弹盘期望 **/
		if (shoot_ctrl_cmd->bullet_mode == BULLET_CONTINUE) {
			DJIMotorOuterLoop(loader, SPEED_LOOP);
			DJIMotorSetRef(loader, Target_Continue_Speed);
			Angle_Target      = loader->measure.Angle_DEG;
			RAMP_Angle_Target = loader->measure.Angle_DEG;
		} else if ( shoot_ctrl_cmd->bullet_mode != BULLET_CONTINUE) {
			DJIMotorOuterLoop(loader, ANGLE_LOOP);
			RAMP_Angle_Target = RAMP_float(Angle_Target, RAMP_Angle_Target, 12);
			DJIMotorSetRef(loader, RAMP_Angle_Target);
		}
	 }
  }
}

 void Shoot_friction_Update()
{
	shoot_ctrl_cmd->bullet_speed = 15;        // 测试
	if (shoot_ctrl_cmd->mode == SHOOT_STOP) {
		DJIMotorSetRef(friction_l, 0);
		DJIMotorSetRef(friction_r, 0);
		DJIMotorSetRef(friction_u, 0);
	} else if (shoot_ctrl_cmd->mode == SHOOT_STUCKING) {  //SHOOT_SPEED
            /*发射机构电机全退弹*/
		DJIMotorSetRef(friction_l, SHOOT_SPEED);
		DJIMotorSetRef(friction_r, SHOOT_SPEED);
		DJIMotorSetRef(friction_u, SHOOT_SPEED);
	} else {
		switch (shoot_ctrl_cmd->bullet_speed) {
			case 20:
			    DJIMotorSetRef(friction_l,  SHOOT_SPEED+500);
			    DJIMotorSetRef(friction_r, -SHOOT_SPEED-500);
			    DJIMotorSetRef(friction_u, -SHOOT_SPEED-500);
                shoot_feedback_data.real_bullet_speed = 18.2f;  // 此处填写该case下调得实际弹速的典型值
			    break;
			case 15:
			    DJIMotorSetRef(friction_l,  SHOOT_SPEED);
			    DJIMotorSetRef(friction_r, -SHOOT_SPEED);
			    DJIMotorSetRef(friction_u, -SHOOT_SPEED);
                shoot_feedback_data.real_bullet_speed = 15.2f;  // 此处填写该case下调得实际弹速的典型值
			    break;
			case 0:
			    DJIMotorSetRef(friction_l, 0);
			    DJIMotorSetRef(friction_r, 0);
			    DJIMotorSetRef(friction_u, 0);
                shoot_feedback_data.real_bullet_speed = 0.0f;
			    break;
			default :  // 使用最低弹速保持一致
			    DJIMotorSetRef(friction_l, -SHOOT_SPEED);
			    DJIMotorSetRef(friction_r, SHOOT_SPEED);
			    DJIMotorSetRef(friction_u, -SHOOT_SPEED);
                shoot_feedback_data.real_bullet_speed = 14.2f;  // 此处填写该case下调得实际弹速的典型值
				break;
		}
	}
}

 void Shoot_Upeadt() {
	// 发弹计数及弹速显示
	static float last_bullet_speed_fdb;

	/* 接收消息 */
	if (SubGetMessage( shoot_sub, (void *)shoot_ctrl_cmd) != 1)
	{
   	    shoot_feedback_data.shoot_status = Device_Offline;
  	    shoot_ctrl_cmd->mode = SHOOT_ZERO_FORCE;
    }
	/* 电机在线检测 */
	if (loader->watchdog->state != Dog_Online)
	{
   	    shoot_feedback_data.shoot_status = Device_Offline;
	    shoot_ctrl_cmd->mode = SHOOT_ZERO_FORCE;
	}
	else
	{
	   shoot_feedback_data.shoot_status = Device_Online;
	}
    /* 弹频控制 */
	if (shoot_ctrl_cmd->bullet_speed_fdb != last_bullet_speed_fdb) {
		buttle_cnt ++;
		shoot_time = HAL_GetTick();
//		RMQueuePush(bullet_speed_queue, &shoot_ctrl_cmd->bullet_speed_fdb);
		last_bullet_speed_fdb = shoot_ctrl_cmd->bullet_speed_fdb;
	} 
	// 电机控制
	if (shoot_ctrl_cmd->mode == SHOOT_ZERO_FORCE) {
		Shoot_STOP();
		Angle_Target      = loader->measure.Angle_DEG;
		RAMP_Angle_Target = loader->measure.Angle_DEG;
    }
	else 
	{
		Shoot_load_Update();
		Shoot_friction_Update();
		Shoot_Calc();
	}
	loader_angle  = loader->measure.MchanicalAngle;
	loader_angle_deg= loader->measure.Angle_DEG;
	loader_target = loader->motor_controller.ref_position;
	loader_round = loader->measure.round;

	fire_speed_nofil[0] =  friction_l->measure.SpeedFilter;
	fire_speed_nofil[1] = -friction_r->measure.SpeedFilter;
	fire_speed_nofil[2] = -friction_u->measure.SpeedFilter;
	// 推送消息
   	shoot_feedback_data.mode = shoot_ctrl_cmd->mode;
   	shoot_feedback_data.bullet_mode = shoot_ctrl_cmd->bullet_mode;
   	PubPushMessage( shoot_pub, (void *)&shoot_feedback_data);
}

 void Shoot_Calc()
{
    static int16_t can1_send[4];
	/**  摩擦轮控制  **/
	if (friction_r->motor_controller.ref_speed != 0 ) {			
		SMC_Calc(&shoot_smc[0], friction_l->motor_controller.ref_speed, friction_l->measure.SpeedFilter);
		SMC_Calc(&shoot_smc[1], friction_r->motor_controller.ref_speed, friction_r->measure.SpeedFilter);
		SMC_Calc(&shoot_smc[2], friction_u->motor_controller.ref_speed, friction_u->measure.SpeedFilter);
		can1_send[1] = (int16_t )shoot_smc[0].output;
		can1_send[2] = (int16_t )shoot_smc[1].output;
		can1_send[3] = (int16_t )shoot_smc[2].output;
	} else {
		PID_Control( friction_l->measure.SpeedFilter, friction_l->motor_controller.ref_speed, &Shoot_Speed_PID[0] );
		PID_Control( friction_r->measure.SpeedFilter, friction_r->motor_controller.ref_speed, &Shoot_Speed_PID[1] );
		PID_Control( friction_u->measure.SpeedFilter, friction_u->motor_controller.ref_speed, &Shoot_Speed_PID[2] );
		can1_send[1] = (int16_t )Shoot_Speed_PID[0].pid_out;
		can1_send[2] = (int16_t )Shoot_Speed_PID[1].pid_out;
		can1_send[3] = (int16_t )Shoot_Speed_PID[2].pid_out;
	}
	limit(can1_send[1] , RM3508_LIMIT, -RM3508_LIMIT);
    limit(can1_send[2] , RM3508_LIMIT, -RM3508_LIMIT);
    limit(can1_send[3] , RM3508_LIMIT, -RM3508_LIMIT);
    
	/**  拨弹盘控制  **/
	if( loader->motor_controller.motor_setting.close_loop_type >= ANGLE_LOOP) {
		PID_Control_Smis(loader->measure.Angle_DEG, loader->motor_controller.ref_position, &Pluck_Place_PIDS, loader->measure.SpeedFilter);
		PID_Control( loader->measure.SpeedFilter, Pluck_Place_PIDS.pid_out, &Pluck_Speed_PID);
  	    can1_send[0] = (int16_t )Pluck_Speed_PID.pid_out;
        limit(can1_send[0], RM3508_LIMIT, -RM3508_LIMIT);
	} else {
	    PID_Control(loader->measure.SpeedFilter, loader->motor_controller.ref_speed, &Pluck_Continue_PID);
   	   	can1_send[0] = (int16_t )Pluck_Continue_PID.pid_out;
        limit(can1_send[0], RM3508_LIMIT, -RM3508_LIMIT);
    }
	
#if SHOOT_MOVE
	DJIMotor_Transmit( &hcan1, 0x200, can1_send);
#endif
}

void Shoot_STOP()
{
	static int16_t can1_send[4] = {0};
	DJIMotor_Transmit( &hcan1, 0x200, can1_send);
}


