#include "Task_Vision.h"

#include "bsp_usb.h"
#include "WatchDog.h"
#include "Referee_unpack.h"

#include "robot_def.h"
#include "ins_task.h"
#include "VCOMCOMM.h"

/** @brief 重力 */
#define GRAVITY 9.78f

extern USBD_HandleTypeDef hUsbDeviceFS;
static Aim_Rx_t Aim_Rx;                 //!< @brief  储存自瞄信息
static Aim_Rx_info Aim_Rx_infopack;     //!< @brief  接收处理结构体
static Aim_Tx_t Aim_Tx;                 //!< @brief  时间发送结构体
Aim_Data Aim_Ref;
extern Aim_Action_e AimAction ;
static AutoAim_mode_e Auto_Aim_mode = auto_aim_off;
static attitude_t *ins;
static WatchDog_TypeDef *PC_Dog;
/**  功能函数  **/
void Aim_Init();        // 自瞄通信初始化
void Send_to_Vision();  // 通过虚拟串口给PC发送消息
void Aim_Control();

/**  坐标转换(N->B)  **/
//  void Coordinate_Transformation (RotationMatrix_t R, const Pose_t* PoseN, Pose_t* PoseB); 
/**  坐标点到原点的水平距离  **/
float DistanceHorizontal(Pose_t pose);
/**  坐标点到原点的距离  **/
float DistanceToOrigin(Pose_t pose);
/**  空气阻力模型  **/
float monoAirResistance_Model(float horizontal, float bullet_speed, float angle_pitch, float k);
/**  获得飞行时间  **/
float Get_bullet_fly_time( float horizontal, float bullet_speed, float angle_pitch);
/**  完全空气阻力模型 **/
double ballisticSolver(float horizontal, float vertical, double bullet_speed, double k);
/**  Pitch轴弹道补偿  **/
float Get_Pitch_Angle_Compensation(float horizontal, float vertical, float bullet_speed, float k);
/** 根据最优决策得出被击打装甲板 自动解算弹道 */
void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z);


void Task_INS(void *pvParameters)
{
	 static portTickType currentTime;
     for(;;)
	 {
		 currentTime = xTaskGetTickCount();
		 INS_Task();
		 vTaskDelayUntil(&currentTime, 1);
	}
}


void Task_Vision()
{
	static portTickType currentTime;
	Aim_Init();
	for (;;)
	{
		currentTime = xTaskGetTickCount();
		  // 四元数发送
		Send_to_Vision();
		  // 自瞄数据解算
		Aim_Control();
		vTaskDelayUntil(&currentTime, 1);
	}
}

void Vision_TimestampReceive(uint8_t *data)
{
	uint64_t StandardTimeStamp;
	memcpy(&StandardTimeStamp, data, sizeof(StandardTimeStamp));      // 时间戳同步
	Aim_Rx.TimeStamp_setoff = StandardTimeStamp - xTaskGetTickCount();
	Aim_Rx.Rx_State         = COLOUR;
	Feed_Dog(PC_Dog);
}


void Vision_ColorReceive(uint8_t *data)
{
	Aim_Rx.Rx_State = UPDATE;
	Feed_Dog(PC_Dog);
}

void Vision_DataReceive(uint8_t *data)
{
	memcpy(&Aim_Rx_infopack, data, sizeof(Aim_Rx_info));        //自瞄数据
	if(Aim_Rx.Rx_State != TIMESTAMP || Aim_Rx.Rx_State != COLOUR)
	    Aim_Rx.Rx_State  = UPDATING;
	Aim_Control();   
	Feed_Dog(PC_Dog);
}

void Aim_Init()
{
	Aim_Rx.Rx_State          = TIMESTAMP; 
	Aim_Rx.Fixed_Center_time = 200;               //静态预测时间(拨弹盘转动，通信延迟等)
	Aim_Rx.Fixed_Armor_time  = 100;
	Aim_Rx.K                 = 0.000556;          //空气阻力系数
	
	WatchDog_Init_config PC = {
		.watch_callback = 0,
		.dog_name = "USBDOG",
		.owner_id = 0,
		.Max_num = 10,
	};
	PC_Dog = WatchDog_Init(PC);
	ins = INS_Init();
}

void Send_to_Vision()
{
    static uint8_t Count = 0, Count_Max = 10; //控制发送频率，根据IMU频率更改，给下板发送的频率 = IMU频率(1k) / Count_Max
    if(Count ++ == Count_Max) {
	    Aim_Tx.Quaternions.p0 = ins->q[0];
        Aim_Tx.Quaternions.p1 = ins->q[1];
        Aim_Tx.Quaternions.p2 = ins->q[2];
        Aim_Tx.Quaternions.p3 = ins->q[3];
        UsbTransmit(1, 10, (uint8_t *)&Aim_Tx, sizeof(Aim_Tx));
	    Count = 0;
    }
}


/* 自瞄控制主程序 */
void Aim_Control()
{
    /* 自瞄系统正在测试中 */
    static uint16_t Predicted_Gap = 0, Bullet_fly_time = 400 , Bullet_Speed = 15;  // 动态预测时间
    static float Tar_horizontal, Tar_vertical, Tar_angle_pitch;//弹道模型
    static float distance_min = 0;       // 获得最近装甲板
    static  uint8_t idx = 0;             // 索引
    
      /* 自瞄模式 */
    if(AimAction != AIM_STOP && PC_Dog->state == Dog_Online ) {
	  Aim_Rx.Rx_State == UPDATING ? Predicted_Gap = 0 : Predicted_Gap ++; //量测更新,在视觉空窗期电控去预测

	  Aim_Rx.Predicted_Center_time = (Aim_Rx.Fixed_Center_time + Aim_Rx_infopack.delay + Bullet_fly_time + Predicted_Gap) / 1000.0f ;  //预测时间ms
	  Aim_Rx.Predicted_Armor_time  = (Aim_Rx.Fixed_Armor_time  + Aim_Rx_infopack.delay + Bullet_fly_time + Predicted_Gap) / 1000.0f ;

	  /* Center_Pose更新(预测) */
	  Aim_Rx.Predicted_Center_Pose.x = Aim_Rx_infopack.pose.pos_X + Aim_Rx_infopack.pose.Vx * Aim_Rx.Predicted_Center_time;
	  Aim_Rx.Predicted_Center_Pose.y = Aim_Rx_infopack.pose.pos_Y + Aim_Rx_infopack.pose.Vy * Aim_Rx.Predicted_Center_time;
	  Aim_Rx.Predicted_Center_Pose.z = Aim_Rx_infopack.pose.pos_Z + Aim_Rx_infopack.pose.Vz * Aim_Rx.Predicted_Center_time;
  	  Aim_Rx.Predicted_Center_Pose.HorizontalDistance = DistanceHorizontal(Aim_Rx.Predicted_Center_Pose);

	  /* 获得目标Amror_Pose并得到Tar_Armor */
	  distance_min = Aim_Rx.Predicted_Center_Pose.HorizontalDistance;
   	  for(uint8_t i = 0; i < Aim_Rx_infopack.pose.armor_number; i++) 
	   {   //  根据对方装甲板数目觉得  i的最大值
		  if(Aim_Rx_infopack.pose.armor_id == 6 && Aim_Rx_infopack.pose.armor_number == 3) {  //  判断为对方前哨站 进入前哨站模式 手动设置装甲板转速和装甲板之间距离
			Aim_Rx_infopack.pose.r1 = 0.2;
			Aim_Rx_infopack.pose.r2 = Aim_Rx_infopack.pose.r1;
			Aim_Rx_infopack.pose.omega = 2.512; // 前哨站转速一定
		  } 
																															// 此处为预测对方各个装甲板的朝向角       
		  float Predicted_theta = Aim_Rx_infopack.pose.theta + Aim_Rx_infopack.pose.omega * Aim_Rx.Predicted_Armor_time + i * (2 * PI / Aim_Rx_infopack.pose.armor_number);

		  if(Aim_Rx_infopack.pose.armor_number<=3)  // 可能为对方平衡步兵 或者为对方前哨站
				Aim_Rx_infopack.pose.r2 = Aim_Rx_infopack.pose.r1;

		  float r = Aim_Rx_infopack.pose.r1 * ((i+1) % 2) + Aim_Rx_infopack.pose.r2 * (i % 2);    //r1是当前装甲板
		  float dz = Aim_Rx_infopack.pose.dz * (i % 2) ; //  另一个装甲板的高度
		  Aim_Rx.Predicted_Armor_Pose[i].x = Aim_Rx.Predicted_Center_Pose.x - r * cos(Predicted_theta);
		  Aim_Rx.Predicted_Armor_Pose[i].y = Aim_Rx.Predicted_Center_Pose.y - r * sin(Predicted_theta);
		  Aim_Rx.Predicted_Armor_Pose[i].z = Aim_Rx.Predicted_Center_Pose.z + dz;
		  Aim_Rx.Predicted_Armor_Pose[i].HorizontalDistance = DistanceHorizontal(Aim_Rx.Predicted_Armor_Pose[i]);
            
/*     两种常见决策方案：
		  1.计算枪管到目标装甲板theta最接近的那个装甲板
		  2.计算水平距离最近的装甲板，当前是这种
*/
		  if(Aim_Rx.Predicted_Armor_Pose[i].HorizontalDistance < distance_min) {
              distance_min = Aim_Rx.Predicted_Armor_Pose[i].HorizontalDistance;    // 选出水平距离最近的装甲板
              idx = i;
           }
        }
         
        /* 发送预测位置给上位机可视化 */
        Aim_Tx.Tar_Pose.pos_X = Aim_Rx.Predicted_Armor_Pose[idx].x;
        Aim_Tx.Tar_Pose.pos_Y = Aim_Rx.Predicted_Armor_Pose[idx].y;
        Aim_Tx.Tar_Pose.pos_Z = Aim_Rx.Predicted_Armor_Pose[idx].z;
        
        /* 弹道模型更新 */
        if(Aim_Rx.Rx_State == UPDATING) {
            Tar_horizontal       = Aim_Rx.Predicted_Armor_Pose[idx].HorizontalDistance - 0.5f; // 减去对方装甲板坐标系到枪口高度
            Tar_vertical        = Aim_Rx.Predicted_Armor_Pose[idx].z;
            Aim_Ref.Auto_Mode >= auto_aim_normal ? (Aim_Rx.PitchOffset = Get_Pitch_Angle_Compensation(Tar_horizontal, Tar_vertical, Bullet_Speed, Aim_Rx.K) )  //单方向空气阻力
												 : (Aim_Rx.PitchOffset = ballisticSolver(Tar_horizontal, Tar_vertical, Bullet_Speed, Aim_Rx.K) );      //完美空气阻力
            Tar_angle_pitch      = atan2(Tar_vertical, Tar_horizontal);
            Bullet_fly_time      = Get_bullet_fly_time(Tar_horizontal, Bullet_Speed, Tar_angle_pitch + Aim_Rx.PitchOffset);
            Aim_Rx.Rx_State      = UPDATE;
        }
        /* 坐标变换成角度 */
        Aim_Rx.Predicted_Armor_Yaw      = atan2( Aim_Rx.Predicted_Armor_Pose[idx].y, Aim_Rx.Predicted_Armor_Pose[idx].x)                     * 180 / PI + ins->YawRoundCount * 360.0f;
        Aim_Rx.Predicted_Armor_Pitch    = atan2( Aim_Rx.Predicted_Armor_Pose[idx].z, Aim_Rx.Predicted_Armor_Pose[idx].HorizontalDistance)    * 180 / PI;
        Aim_Rx.Predicted_Center_Yaw     = atan2( Aim_Rx.Predicted_Center_Pose.y,     Aim_Rx.Predicted_Center_Pose.x )                        * 180 / PI + ins->YawRoundCount * 360.0f;
        Aim_Rx.Predicted_Center_Pitch   = atan2( Aim_Rx.Predicted_Center_Pose.z,     Aim_Rx.Predicted_Center_Pose.HorizontalDistance)        * 180 / PI; 
        
        /* 得到期望角度（角度 + 弹道补偿）*/
        if( ABS(ins->ContinuousYaw -  Aim_Rx.Predicted_Armor_Yaw) < 30 && ABS(ins->Pitch + Aim_Rx.Predicted_Armor_Pitch) < 30) {
            if( ABS(Aim_Rx_infopack.pose.omega) <= 0.3f )   // Yaw轴根据目标转速选择跟装甲板还是跟车辆中心
                Aim_Ref.Yaw = Aim_Rx.Predicted_Armor_Yaw;
            else
                Aim_Ref.Yaw = Aim_Rx.Predicted_Center_Yaw;
            Aim_Ref.Pitch =  - (Aim_Rx.Predicted_Armor_Pitch + Aim_Rx.PitchOffset);
        }

        /* 能否开启自瞄(有距离判断  在0.7m - 7.5m自瞄) */
        if(Aim_Rx_infopack.tracker_status && Aim_Rx.Predicted_Center_Pose.HorizontalDistance >= 0.7f && Aim_Rx.Predicted_Center_Pose.HorizontalDistance <= 7.5f) {
            Aim_Ref.Auto_Mode = auto_aim_normal;	
        } else
            Aim_Ref.Auto_Mode = auto_aim_off;
    } else 
	   Aim_Ref.Auto_Mode = auto_aim_off;
	
	Aim_Ref.Aim_Action = AimAction;
	Aim_Ref.PC_State   = PC_Dog->state;
}

/* 坐标点到原点水平距离 */
float DistanceHorizontal(Pose_t pose){
    return sqrt(  pose.x * pose.x + pose.y * pose.y);
}

/* 坐标点到原点的距离 */
float DistanceToOrigin(Pose_t pose){
    return sqrt(  pose.x * pose.x + pose.y * pose.y + pose.z * pose.z);
}

/* 获得子弹飞行时间 */
float Get_bullet_fly_time( float horizontal, float bullet_speed, float angle_pitch) {
    float t;
    t = (float)((exp(Aim_Rx.K * horizontal) - 1) / (Aim_Rx.K * bullet_speed * cosf(angle_pitch)));//水平方向求得飞行时间 t
    if(t < 0){
        //由于严重超出最大射程，计算过程中浮点数溢出，导致t变成负数
        //重置t，防止下次调用会出现nan
        t = 0;
        return 0;
    }
    return t;
}

/* 求得Pitch轴角度补偿(单方向空气阻力，适用于远距离，小角度) */
float Get_Pitch_Angle_Compensation(float horizontal, float vertical, float bullet_speed, float k) {
    float temp_vertical, actual_vertical, error_vertical;
    float pitch, pitch_new;
    
    pitch = atan2(vertical, horizontal);
    temp_vertical = vertical;
    //迭代重力法
    for (uint8_t i = 0; i < 20; i++){
        pitch_new = atan2(temp_vertical, horizontal);
        actual_vertical = monoAirResistance_Model(horizontal, bullet_speed, pitch_new, k);
        error_vertical = 0.3f * (vertical - actual_vertical);
        temp_vertical = temp_vertical + error_vertical;
        if (fabsf(error_vertical) < 0.0001f)
            break;
    }
    return (pitch_new - pitch) * 180 / PI;
}

/* 单方向空气阻力弹道模型 利用飞行速度求Pitch轴竖直高度  */
float monoAirResistance_Model(float horizontal, float bullet_speed, float angle_pitch, float k){
    float actual_vertical, t;
    
    t = Get_bullet_fly_time(horizontal, bullet_speed, angle_pitch);
    actual_vertical = bullet_speed * sin(angle_pitch) * t - GRAVITY * t * t / 2; //得出子弹会打到的竖直高度
//    actual_vertical = (1 / k) * log(k * bullet_speed * sin(angle_pitch) * t + 1) - (GRAVITY * t*t)/( ( 1/k ) * (2 * t * bullet_speed * sin(angle_pitch) + 2) ); // 弹丸上升阻力模型
    return actual_vertical;
}

/* 完美弹道模型 */
double ballisticSolver(float horizontal, float vertical, double bullet_speed, double k)
{
// FIX ME:  NaN Failed 是否因为 vertical 有时候为负
// TODO: 世界坐标系下进行迭代
    double pitch = atan2(vertical, horizontal);
    double temp_vertical = vertical;
    double pitch_new = pitch;

    // 迭代求解
    for (int i = 0; i < 20; ++i) {
        double x = 0.0;
        double y = 0.0;
        double p = tan(pitch_new);
        double v = bullet_speed;
        double u = v / sqrt(1 + p * p);
        double delta_x = horizontal / 10;
        
        // 使用四阶龙格-库塔法求解微分方程，步长决定精度
        for (int j = 0; j < 20; ++j) {
            double k1_u = -k * u * sqrt(1 + (p*p) );
            double k1_p = -GRAVITY / (u*u);
            double k1_u_sum = u + k1_u * (delta_x / 2);
            double k1_p_sum = p + k1_p * (delta_x / 2);

            double k2_u = -k * k1_u_sum * sqrt(1 + (k1_p_sum*k1_p_sum) );
            double k2_p = -GRAVITY / (k1_u_sum*k1_u_sum);
            double k2_u_sum = u + k2_u * (delta_x / 2);
            double k2_p_sum = p + k2_p * (delta_x / 2);

            double k3_u = -k * k2_u_sum * sqrt(1 + (k2_p_sum*k2_p_sum) );
            double k3_p = -GRAVITY / (k2_u_sum*k2_u_sum);
            double k3_u_sum = u + k3_u * (delta_x / 2);
            double k3_p_sum = p + k3_p * (delta_x / 2);

            double k4_u = -k * k3_u_sum * sqrt(1 + (k3_p_sum*k3_p_sum));
            double k4_p = -GRAVITY / (k3_u_sum*k3_u_sum);

            u += (delta_x / 6) * (k1_u + 2 * k2_u + 2 * k3_u + k4_u);
            p += (delta_x / 6) * (k1_p + 2 * k2_p + 2 * k3_p + k4_p);
           
            x += delta_x;
            y += p * delta_x;
        }

        double error = vertical - y;

        // 如果误差满足停止条件，则跳出迭代
        if (fabs(error) <= 0.00001) {
            break;
        } else {
            temp_vertical += error;
            pitch_new = atan2(temp_vertical, horizontal);
        }
    }
    // 返回仰角修正值
    return (pitch_new - pitch) * 180 / PI;
}
/*旋转矩阵*/
/* void applyRotation(const RotationMatrix_t *rotationMatrix, Aim_Rx_info *info) {
    // Define matrices A and B
    arm_matrix_instance_f32 matA, matB, matResult;
    float32_t A_data[3][3], B_data[3][1], result_data[3][1];

    // Copy rotation matrix data to matrix A
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            A_data[i][j] = rotationMatrix->r[i][j];
        }
    }
    
    // Copy pose data to matrix B
    B_data[0][0] = info->pose.pos_X;
    B_data[1][0] = info->pose.pos_Y;
    B_data[2][0] = info->pose.pos_Z;
    
    // Initialize matrix structures
    arm_mat_init_f32(&matA, 3, 3, (float32_t *)A_data);
    arm_mat_init_f32(&matB, 3, 1, (float32_t *)B_data);
    arm_mat_init_f32(&matResult, 3, 1, (float32_t *)result_data);
    
    // Perform matrix multiplication: Result = A * B
    arm_mat_mult_f32(&matA, &matB, &matResult);

    // Copy result back to pose
    info->pose.pos_X = result_data[0][0];
    info->pose.pos_Y = result_data[1][0];
    info->pose.pos_Z = result_data[2][0];
} */
