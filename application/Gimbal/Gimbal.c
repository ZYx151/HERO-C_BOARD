#include "Gimbal.h"

#include "DJIMotor.h"
#include "DMmotor.h"
#include "pub_sub.h"
#include "ins_task.h"
#include "remote.h"
#include "PID.h"
#include "bsp_usart.h"  
#include "usb_typdef.h"                                 

#include "controller.h"

extern Aim_Data Aim_Ref; /** 自瞄发射 **/
/* 云台部分 */
static Publisher_t *gimbal_pub;            // 云台控制消息发布者
static  Subscriber_t *gimbal_sub;          // 云台反馈信息订阅者

static  Gimbal_ctrl_cmd_t *gimbal_get_ctrl ;// 云台接收CMD的控制信息
static  DMMotor_Instance_1_4  *pitch_motor ;
static  DJIMotor_Instance *yaw_motor ;                                                                                        
static  Gimbal_upload_t   gimbal_feedback_data;  // 云台发布的反馈信息

//  功能函数
static attitude_t *ins;
static RC_Ctl_t *rc_data;
void Gimbal_GYRO_Calc(void);   /** 云台陀螺仪模式  **/ 
void Gimbal_MIDDLE_Calc(void); /** 云台机械模式  **/ 
void Gimbal_STOP(void);        /** 云台停止模式 **/ 
void Gimbal_SystemCalc(float Gimbal_pitch, float Gimbal_yaw); /** 云台电机系统辨识模式 **/
float auto_rotate_param(void);
float yaw_ff = 0.0f, pitch_deg, pitch_imu, pitch_target;
uint8_t test_gimbal = 0;
PTZAngle_Ref_t gimbal_test;

static PID_Smis Gimbal_Pos_PID[2][3] = {{{.Kp = 4.5f, .Ki = 0, .Kd = -2.2f, .interlimit = 3000, .outlimit = 30000, .DeadBand = 10.0f, .inter_threLow = 50, .inter_threUp = 500  },   // YAW轴归中
                              {.Kp = 35.0f, .Ki = 0.0f, .Kd = -10.0f, .interlimit = 2000, .outlimit = 30000, .DeadBand = 0.001f, .inter_threLow = 5, .inter_threUp = 10  },        // YAW轴陀螺仪
							  {.Kp = 25.0f, .Ki = 0.0f, .Kd = 5.0f,  .interlimit = 2000, .outlimit = 30000, .DeadBand = 0.001f, .inter_threLow = 5, .inter_threUp = 10  }},       // YAW轴发射

                            {{.Kp = 0.5f , .Ki = 0, .Kd = 0.15f,.interlimit = 3000, .outlimit = 16000, .DeadBand = 1.0f, .inter_threLow = 5, .inter_threUp = 500},   // PITCH轴归中
                             {.Kp = 3.50f,  .Ki = 0, .Kd = 0.35f, .interlimit = 2000, .outlimit = 1000, .DeadBand = 0.01f, .inter_threLow = 5, .inter_threUp = 10},   // PITCH轴陀螺仪
                             {.Kp = 2.8f,  .Ki = 0, .Kd = 0.35f,.interlimit = 2000, .outlimit = 1000, .DeadBand = 1.0f, .inter_threLow = 5, .inter_threUp = 10}}};   // PITCH轴发射
						     
static PID Gimbal_Speed_PID[2][3] = {{{.Kp = 4.30f, .Ki = 0, .Kd = 0.0f, .interlimit = 3000, .outlimit = 28000, .DeadBand = 0, .inter_threLow = 500, .inter_threUp = 1000},
							  {.Kp = 350.0f, .Ki = 6.0f, .Kd = 0.0f, .interlimit = 10000, .outlimit = 30000, .DeadBand = 0, .inter_threLow = 5, .inter_threUp = 10000},
							  {.Kp = 20.0f, .Ki = 6.0f, .Kd = 0.0f, .interlimit = 10000, .outlimit = 28000, .DeadBand = 0, .inter_threLow = 5, .inter_threUp = 5000}},
								  
                            {{.Kp = 0.12f, .Ki = 0.0f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 10000, .DeadBand = 0, .inter_threLow = 500, .inter_threUp = 1000},
                             {.Kp = 3.0f,  .Ki = 1.0f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 10000, .DeadBand = 0, .inter_threLow = 2, .inter_threUp = 1000},
                             {.Kp = 2.50f,  .Ki = 0.5f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 10000, .DeadBand = 0, .inter_threLow = 2, .inter_threUp = 1000}}};

/* 位置环PID为底盘跟随解算旋转速度 */
static PID_Smis Chassis_Rotate_PIDS = {.Kp = 1.50f, .Ki = 0, .Kd = -2.5f, .interlimit = 1500, .outlimit = 4000*PI, .DeadBand = 0.05f, .inter_threLow = 500, .inter_threUp = 2000};     // 底盘跟随   位置环
static PID Chassis_Rotate_PID = { .Kp =  15.0f, .Ki = 6.0f, .Kd = 0, .interlimit = 3000, .outlimit = 8000*PI, .DeadBand = 0, .inter_threLow = 100*PI, .inter_threUp = 2000*PI};             // 速度环
// 前反馈
static FeedForward_Typedef FF_rotete = { .K1 = 0.0f, .K2 = 2.8f, .K3 = 0.0f, .OutMax = 3000 * PI, .DWT_CNT = 0};
static FeedForward_Typedef GimbalYaw_FF = { .K1 = 0.0f, .K2 = 2.0f, .K3 = 0.0f , .OutMax = 3000, .DWT_CNT = 0};

void Gimbal_Init(void)
{
	memset(&gimbal_feedback_data, 0, sizeof(Gimbal_upload_t));
 	gimbal_pub = PubRegister("gimbal_upload", sizeof(Gimbal_upload_t) );
	gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_ctrl_cmd_t) );
	
	gimbal_get_ctrl = (Gimbal_ctrl_cmd_t* )RMLIB_MALLOC(sizeof(Gimbal_ctrl_cmd_t));
	memset(gimbal_get_ctrl, 0, sizeof(Gimbal_ctrl_cmd_t));
	ins = INS_Init();
	rc_data = get_remote_control_point();
	
Motor_Init_config_s pitch_config = {
	.motor_type = DM4310,
	.dog_init_config = {
		.dog_name = "Pitch",
		.GPIO_Pin = GPIO_PIN_13,
	},
	.can_init_config = {
		.can_handle = &hcan2,
		.tx_id = 1,
		.rx_id = 0x301,
	},
	.contruller_config = {
		.MotorCenter = Pitch_Mid,
		.setting_config = {
			.feedback_reverse_flag = FEEDBACK_NORMAL,
			.motor_reverse_flag =   MOTOR_NORMAL,   // 电机反转
			.angle_feedback_source = OTHER_FEED,    //云台为MID归中模式时使用机械角度
			.speed_feedback_source = OTHER_FEED,    // 云台初始化后使用 DJIMotorChangeFeed函数修改反馈类型使用IMU控制
			.close_loop_type = ANGLE_LOOP,
			.feedfoward_type = NONE_FEEDFORWARD,    // 不使用前馈类型 
			.control_mode = PID_MODEL,
		},
	},
	.other_angle_feedback_ptr = &ins->Pitch,
	.other_speed_feedback_ptr = &ins->gyro[0],
};

Motor_Init_config_s yaw_config = {
	.motor_type = GM6020,
	.dog_init_config = {
		.dog_name = "Yaw",
	    .GPIO_Pin = GPIO_PIN_13,
	},
	.can_init_config = {
		.can_handle = &hcan2,
		.tx_id = 1,
		.rx_id = 205,
	},
	.contruller_config = {
		.MotorCenter = Yaw_Mid_Front,
		.setting_config = {
			.motor_reverse_flag = MOTOR_NORMAL,   // 电机不反转
			.feedback_reverse_flag = FEEDBACK_NORMAL,
			.angle_feedback_source = MOTOR_FEED,
			.speed_feedback_source = MOTOR_FEED,    // 云台初始化后使用 DJIMotorChangeFeed函数修改反馈类型使用IMU控制
			.close_loop_type = ANGLE_LOOP,          // 使用闭环类型 默认位置环位最外环
			.feedfoward_type = NONE_FEEDFORWARD, // 不使用前馈类型 
			.control_mode = PID_MODEL,
		},
	},
	.other_angle_feedback_ptr = &ins->ContinuousYaw,
	.other_speed_feedback_ptr = &ins->gyro[2],
};
    
/* 跟随PID初始化 */
	pitch_motor =  DMMotorInit_1_4(&pitch_config);
    yaw_motor = DJI_Motor_create(&yaw_config);
}

/*
	PubPushMessage(gimbal_pub, (void *)&gimbal_feed);
	SubGetMessage(gimbal_sub, gimbal_get_control);
*/
void Gimbal_Task(void)
{
	// 通过订阅者获得控制参数
	if (SubGetMessage( gimbal_sub,(void *) gimbal_get_ctrl) != 1) {
		yaw_motor->stop_flag  = MOTOR_STOP;
	    pitch_motor->stop_flag = MOTOR_STOP;
		return;
	}
	// 检测是否有外设掉线
	if ( pitch_motor->watchdog->state == Dog_Online ) {
		gimbal_feedback_data.gimbal_status = Device_Online;
		gimbal_feedback_data.yaw_encorder = yaw_motor->measure.MchanicalAngle;
		gimbal_feedback_data.pitch_encorder = pitch_motor->measure.MchanicalAngle;
	} else {
		gimbal_feedback_data.gimbal_status = Device_Offline;
		gimbal_get_ctrl->mode = GIMBAL_ZERO_FORCE ;
	}
  gimbal_get_ctrl->rotate_feedforward = 0;

   switch (gimbal_get_ctrl->mode) {
		// 启动电机
			case GIMBAL_ZERO_FORCE :
		   	    yaw_motor->stop_flag   = MOTOR_STOP;
		        pitch_motor->stop_flag = MOTOR_STOP;
			    gimbal_feedback_data.rotate_speed = 0;
			    Gimbal_STOP();
			break;
      // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
			case GIMBAL_GYRO_MODE :
				DJIMotorEnable(yaw_motor);
			    DMMotorEnable(pitch_motor);
			   if(gimbal_get_ctrl->last_mode != GIMBAL_GYRO_MODE) {
					DJIMotorChangeFeed(yaw_motor,  OTHER_FEED);
					DMMotorChangeFeed(pitch_motor, OTHER_FEED);
    			}
			    DJIMotorSetRef(yaw_motor,  gimbal_get_ctrl->Gyro_Ref.Yaw);
			    DMMotorSetRef(pitch_motor, gimbal_get_ctrl->Gyro_Ref.Pitch);
			    /* 底盘跟随速度解算 */
				Gimbal_GYRO_Calc();
 			    auto_rotate_param();
				pitch_target = gimbal_get_ctrl->Gyro_Ref.Pitch;
			break;
			
			case GIMBAL_MIDDLE :
				DJIMotorEnable(yaw_motor);
			    DMMotorEnable(pitch_motor);
			    if(gimbal_get_ctrl->last_mode != GIMBAL_MIDDLE) {
					DJIMotorChangeFeed(yaw_motor, MOTOR_FEED);
					DMMotorChangeFeed(pitch_motor, MOTOR_FEED);
			    }
				Gimbal_MIDDLE_Calc();
 			    auto_rotate_param();
			break;
				
			case GIMBAL_SYSTEM_MODE :
				Gimbal_SystemCalc(0, 15000);
			break;
				
			default :
		   	    yaw_motor->stop_flag  = MOTOR_STOP;
		        pitch_motor->stop_flag = MOTOR_STOP;
			    Gimbal_STOP();
			break;
	}
	Aim_Ref.Auto_Mode = auto_aim_off;
	pitch_deg = pitch_motor->measure.Angle_DEG;
	pitch_imu = ins->Roll;

	gimbal_feedback_data.mode = gimbal_get_ctrl->mode;
     // 推送消息
	PubPushMessage( gimbal_pub, (void *)&gimbal_feedback_data);
}

void Gimbal_GYRO_Calc()
{
     /* 电机发送 */
     static int16_t can2_dji_send[4], can2_dm_send[4];
	/** 测试 **/
	gimbal_get_ctrl->shoot_mode = 0;
	if(test_gimbal == 0) {
		/**  YAW轴解算  遥控器模式  **/
		PID_Control_Smis( ins->ContinuousYaw, gimbal_get_ctrl->Gyro_Ref.Yaw, &Gimbal_Pos_PID[0][1], ins->gyro[2]);
		yaw_ff = FeedForward_Calc(&GimbalYaw_FF, gimbal_get_ctrl->Gyro_Ref.Yaw); //  + yaw_ff * 0.8f
		PID_Control( ins->gyro[2], Gimbal_Pos_PID[0][1].pid_out, &Gimbal_Speed_PID[0][1]);  //  + gimbal_get_ctrl->Feedback_Speed.Yaw * 0.5f 
		can2_dji_send[0] = (int16_t )Gimbal_Speed_PID[0][1].pid_out;
		DJIMotor_Transmit(&hcan2, 0x1FF, can2_dji_send);
 
		/**  PITCH轴解算 **/
		PID_Control_Smis( pitch_motor->measure.Angle_DEG, gimbal_get_ctrl->Gyro_Ref.Pitch, &Gimbal_Pos_PID[1][1], pitch_motor->measure.SpeedFilter*PI/30);
		PID_Control( pitch_motor->measure.SpeedFilter*PI/30, Gimbal_Pos_PID[1][1].pid_out, &Gimbal_Speed_PID[1][1]);
//		PID_Control_Smis( ins->Roll, gimbal_get_ctrl->Gyro_Ref.Pitch, &Gimbal_Pos_PID[1][2], pitch_motor->measure.SpeedFilter*PI/30);
//		PID_Control( pitch_motor->measure.SpeedFilter*PI/30, Gimbal_Pos_PID[1][2].pid_out, &Gimbal_Speed_PID[1][2]);
		can2_dm_send[0] = (int16_t )Gimbal_Speed_PID[1][1].pid_out;
		DJIMotor_Transmit(&hcan2, 0x3FE, can2_dm_send);
	} else {
		/**  YAW轴解算  自瞄模式  **/
		PID_Control_Smis( ins->ContinuousYaw, gimbal_get_ctrl->Gyro_Ref.Yaw, &Gimbal_Pos_PID[0][1], ins->gyro[2]);
		yaw_ff = FeedForward_Calc(&GimbalYaw_FF, gimbal_get_ctrl->Gyro_Ref.Yaw); //  + yaw_ff * 0.8f
		PID_Control( ins->gyro[2], Gimbal_Pos_PID[0][1].pid_out + gimbal_get_ctrl->Feedback_Speed.Yaw * 0.5f , &Gimbal_Speed_PID[0][1]);
		can2_dji_send[0] = (int16_t )Gimbal_Speed_PID[0][1].pid_out;
		DJIMotor_Transmit(&hcan2, 0x1FF, can2_dji_send);

		/**  PITCH轴解算 **/
		PID_Control_Smis( pitch_motor->measure.Angle_DEG, gimbal_test.Pitch, &Gimbal_Pos_PID[1][1], pitch_motor->measure.SpeedFilter*PI/30);
		PID_Control( pitch_motor->measure.SpeedFilter*PI/30, Gimbal_Pos_PID[1][1].pid_out, &Gimbal_Speed_PID[1][1]);
		can2_dm_send[0] = (int16_t )Gimbal_Speed_PID[1][1].pid_out;
//		PID_Control_Smis( ins->Roll, gimbal_get_ctrl->Gyro_Ref.Pitch, &Gimbal_Pos_PID[1][2], pitch_motor->measure.SpeedFilter*PI/30);
//		PID_Control( pitch_motor->measure.SpeedFilter*PI/30, Gimbal_Pos_PID[1][2].pid_out, &Gimbal_Speed_PID[1][2]);
		DJIMotor_Transmit(&hcan2, 0x3FE, can2_dm_send);
	}
	PID_IoutReset(&Gimbal_Speed_PID[1][0]);
	PID_IoutReset(&Gimbal_Speed_PID[0][0]);
	PID_IoutReset(&Gimbal_Speed_PID[1][2]);
	PID_IoutReset(&Gimbal_Speed_PID[0][2]);
}

 void Gimbal_MIDDLE_Calc()
{
	/* 电机发送 */
     static int16_t can2_dji_send[4], can2_dm_send[4];
	 static float Expect_Yaw, Expect_Pitch;  // 获取期望机械角度
	if(gimbal_get_ctrl->Mid_mode == FRONT ) {
		Expect_Yaw = QuickCentering(yaw_motor->measure.MchanicalAngle, Yaw_Mid_Front);
	} else {
		Expect_Yaw = QuickCentering(yaw_motor->measure.MchanicalAngle, Yaw_Mid_Back);
	}
	Expect_Pitch = QuickCentering(pitch_motor->measure.MchanicalAngle, gimbal_get_ctrl->Mech_Ref.Pitch);
	
    /**  YAW轴解算  **/ 	/**  PITCH轴解算 **/
	if (gimbal_get_ctrl->shoot_mode == SHOOT_NOMAL ||  gimbal_get_ctrl->shoot_mode == SHOOT_READY) {	
//		PID_Control_Smis( yaw_motor->measure.Angle_DEG, gimbal_get_ctrl->Mid_mode == FRONT ? 0 : (Yaw_Mid_Back - Yaw_Mid_Front)*0.043945f, &Gimbal_Pos_PID[0][2], yaw_motor->measure.SpeedFilter*PI/30.0f);
		PID_Control_Smis( yaw_motor->measure.MchanicalAngle, Expect_Yaw, &Gimbal_Pos_PID[0][2], yaw_motor->measure.SpeedFilter);
		PID_Control( yaw_motor->measure.SpeedFilter, Gimbal_Pos_PID[0][2].pid_out, &Gimbal_Speed_PID[0][2]);
		can2_dji_send[0] = (int16_t )Gimbal_Speed_PID[0][2].pid_out;

		pitch_target = (Expect_Pitch - pitch_motor->measure.MotorCenter)* 0.0356f;
		PID_Control_Smis( pitch_motor->measure.Angle_DEG, pitch_target, &Gimbal_Pos_PID[1][1], pitch_motor->measure.SpeedFilter*PI/30);
		PID_Control( pitch_motor->measure.SpeedFilter*PI/30, Gimbal_Pos_PID[1][1].pid_out, &Gimbal_Speed_PID[1][1]);
		can2_dm_send[0] = (int16_t )Gimbal_Speed_PID[1][1].pid_out;
	} else {
		PID_Control_Smis( yaw_motor->measure.MchanicalAngle, Expect_Yaw, &Gimbal_Pos_PID[0][0], yaw_motor->measure.SpeedFilter);
		PID_Control( yaw_motor->measure.SpeedFilter, Gimbal_Pos_PID[0][0].pid_out, &Gimbal_Speed_PID[0][0]);
		can2_dji_send[0] = (int16_t )Gimbal_Speed_PID[0][0].pid_out;

		PID_Control_Smis(pitch_motor->measure.MchanicalAngle, Expect_Pitch, &Gimbal_Pos_PID[1][0], pitch_motor->measure.SpeedFilter);  // 角度环
		PID_Control(pitch_motor->measure.SpeedFilter, Gimbal_Pos_PID[1][0].pid_out, &Gimbal_Speed_PID[1][0]);
		can2_dm_send[0] = (int16_t )Gimbal_Speed_PID[1][0].pid_out;
	}
	
#if GIMBAL_MOVE
	DJIMotor_Transmit(&hcan2, 0x1FF, can2_dji_send);
	DJIMotor_Transmit(&hcan2, 0x3FE, can2_dm_send);
#endif
	PID_IoutReset(&Gimbal_Speed_PID[1][1]);
	PID_IoutReset(&Gimbal_Speed_PID[0][1]);
//	PID_IoutReset(&Gimbal_Speed_PID[1][2]);
//	PID_IoutReset(&Gimbal_Speed_PID[0][2]);
}

 void Gimbal_STOP()
{      /* 电机发送 */
    static int16_t can2_dji_send[4], can2_dm_send[4];
	
	can2_dji_send[0] = 0;
	can2_dm_send[0] = 0;
#if GIMBAL_MOVE
	DJIMotor_Transmit(&hcan2, 0x1FF, can2_dji_send);
	DJIMotor_Transmit(&hcan2, 0x3FE, can2_dm_send);
#endif
}


/**
 * @brief  云台系统辨识模式
 * 
 * @param 根据不同的频率获得对应不同周期下sin信号,F低于50重复10周期，否则20周期
*/
__WEAK void get_F();
float Gimbal_direct = 0.0f, Gimbal_Speed = 0.0f;
uint8_t F_Change_flag = 0, T_cnt = 0;
void Gimbal_SystemCalc(float Gimbal_pitch, float Gimbal_yaw)
{
    /* 电机发送 */
    static int16_t can2_dji_send[4], can2_dm_send[4], idx = 0, gimbalyaw_init;
	if(idx++  == 0 ) {
		 gimbalyaw_init = ins->ContinuousYaw;
	}
																											      
     if( F_Change_flag == 1 && fabs(ins->ContinuousYaw - gimbalyaw_init) <= 1.0f) {
		get_F();
		T_cnt = 0;
		F_Change_flag = 0;
	 }
	 
	/* 用于开环系统辨识 */
	Gimbal_Speed = yaw_motor->measure.SpeedFilter;
	yaw_ff = ins->gyro[2];
//    can2_dm_send[0] = Gimbal_direct *Gimbal_pitch;
    can2_dji_send[0] = Gimbal_direct * Gimbal_yaw;
    MotorSend(&hcan2, 0x1FF, can2_dji_send);
//    MotorSend(&hcan2, 0x3FE, can2_dm_send);
}


/**
 * @brief 底盘不同模式旋转速度解算 
 *
**/
 float auto_rotate_param()
{
	uint16_t roate_speed_MAX = 8000 * PI;   // 设置最大旋转速度
	float rotate, angle, offset_angle;
	int16_t yaw_mid;
	offset_angle = yaw_motor->measure.MchanicalAngle;
	
	 gimbal_get_ctrl->Mid_mode == FRONT ? ( yaw_mid = QuickCentering(offset_angle, Yaw_Mid_Front)) : ( yaw_mid = QuickCentering(offset_angle, Yaw_Mid_Back));
    if (gimbal_get_ctrl->chassis_mode == CHASSIS_FOLLOW)
	{
		PID_Control_Smis( offset_angle, yaw_mid,&Chassis_Rotate_PIDS, yaw_motor->measure.SpeedFilter + gimbal_get_ctrl->rotate_feedforward );
		PID_Control(yaw_motor->measure.SpeedFilter , Chassis_Rotate_PIDS.pid_out, &Chassis_Rotate_PID);
		rotate = Chassis_Rotate_PID.pid_out + FeedForward_Calc(&FF_rotete, gimbal_get_ctrl->Feedback_Speed.Yaw);
	}
	else if (gimbal_get_ctrl->chassis_mode == CHASSIS_SPIN)
	{
		//按角度变速（头为前方，装甲板向前转的快，对角转的慢）（angle为比例）
		if (offset_angle <= 7880 && offset_angle >= 5830)
			angle = ABS(6855.0f - offset_angle) / 2050.57f;
		if (offset_angle <= 5830 && offset_angle >= 3780)
			angle = ABS(4805.0f - offset_angle) / 2050.57f;
		if (offset_angle <= 3780 && offset_angle >= 1730)
			angle = ABS(2755.0f - offset_angle) / 2050.57f;
		if (offset_angle <= 1730 && offset_angle > 0)
			angle = ABS(705.0f - offset_angle) / 2050.57f;
		if (offset_angle <= 8191 && offset_angle >= 7880)
			angle = ABS(705.0f + 8191.0f - offset_angle) / 2050.57f;
		rotate = 5000.0f * PI;
	}
	else if (gimbal_get_ctrl->chassis_mode == CHASSIS_NORMAL) {
	        if(rc_data->RemoteMode == REMOTE_INPUT)
            	 rotate  = (float )rc_data->Key_CH[2] * 2000 * PI + FeedForward_Calc(&FF_rotete, rc_data->Key_CH[2] * 3000 );  
	        else if(rc_data->RemoteMode == KEY_MOUSE_INPUT)
            	 rotate  = (float )rc_data->Mouse_Ch[1] * 4000 * PI + FeedForward_Calc(&FF_rotete, rc_data->Key_CH[2] * 4000 );
	} 
	else 
		rotate = 0;
    
    limit(rotate, +roate_speed_MAX, -roate_speed_MAX);
	gimbal_feedback_data.rotate_speed = rotate;
	return rotate;
}

 /* 时间式变速 等待测试*/
////   static uint8_t spin_speed_change = 1;// 0：定速 1：加速 2：减速 （初始从低往高加）
////     // 基准转速 = 最低转速 + 高功率下加速旋转
////   float rotate_benchmark = 8000 + Referee_data_Rx.robot_level * 200;
////   if (spin_speed_change == 0) {
////     // 定速
////     rotate = rotate_benchmark;
////   } else {
////     // 变速范围
////     float rotate_max = rotate_benchmark + 50;
////     float rotate_min = rotate_benchmark - 50;
////     if (rotate < rotate_min) rotate = rotate_min;
////     if (rotate > rotate_max) rotate = rotate_max;
////     if (spin_speed_change == 1) {
////         // 加速（目前为线性）
////         rotate += 0.0005;
////         if (rotate > rotate_max){
////             spin_speed_change = 2;
////         }
////     }else {
////         // 减速 建议不要使用功率减速 以使同功率下整体平均转速较高
////         rotate -= 0.0001;
////         if (rotate < rotate_min){
////             spin_speed_change = 1;
////            }
////         }
////    }

