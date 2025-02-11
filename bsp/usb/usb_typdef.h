#ifndef USB_TYPDEF_H
#define USB_TYPDEF_H

#include <stdbool.h>
#include <string.h>
#include "cmsis_os.h"
#include "usbd_def.h"

/* 串口通信校验 */
#define SEND_SOF    ((uint8_t)0x5A)  // 发送帧头
#define RECEIVE_SOF ((uint8_t)0x5A)  // 接收帧头

#define DETECT_RX_VISION_ID        ((uint8_t)0x00) // 控制组接收视觉ID
#define DETECT_DATA_SEND_ID        ((uint8_t)0x01) // 控制组虚拟串口 发送ID

/* 功能ID */
#define VISION_TOMESTAMP_RECEIVE    ((uint8_t)0x00) // 视觉对齐时间戳
#define VISION_COLOR_RECEIVE        ((uint8_t)0x02) // 颜色识别
#define VISION_DATA_RECEIVE         ((uint8_t)0x01) // 自瞄数据接收

#define TOMESTAMP_DATA_SEND      ((uint8_t)0x00) //  对齐时间戳
#define ENEMY_COLOR_DATA_SEND      ((uint8_t)0x01) // 发送敌方颜色
#define QUATERNION_DATA_SEND      ((uint8_t)0x01) // 发送四元数


#define AM02_ARMOR_X  14
#define AM02_ARMOR_Y  12.5f
#define AM12_ARMOR_X  23.5f
#define AM12_ARMOR_Y  12.7f

#pragma pack(1)

/* 发送给视觉的结构体 */
typedef struct{
    uint64_t TimeStamp;      //!< @brief 对齐后的时间戳
    struct{
        float p0;       
        float p1;
        float p2;
        float p3;
    }Quaternions;            //!< @brief IMU四元数(云台位姿）
    uint16_t Time_Gap;
    struct{
    float pos_X;
    float pos_Y;
    float pos_Z;
    }Tar_Pose;              //!< @brief 预测击打点位姿
} Aim_Tx_t;
#pragma pack()

/* 接收视觉数据的结构体（内存对齐） */
#pragma pack(1)
typedef struct{
    struct{
        float pos_X;                        //!< @brief 全局坐标系下车辆中心点的坐标
        float pos_Y;
        float pos_Z;
        float Vx;                       //!< @brief 全局坐标系下车辆中心点的线速度
        float Vy;
        float Vz;
        float theta;                    //!< @brief 目标装甲板朝向角
        float omega;                    //!< @brief 目标装甲板朝向角的角速度
        float r1;                       //!< @brief 目标中心到前后装甲板的距离
        float r2;                       //!< @brief 目标中心到左右装甲板的距离
        float dz;                       //!< @brief 另一对装甲板的相对于被跟踪装甲板的高度差
        uint8_t armor_number;            //!< @brief 装甲板的数量
        uint8_t armor_id;                //!< @brief 目标车辆ID
    }pose;                               //!< @brief 目标车辆位姿
    uint8_t delay;                       //!< @brief 视觉程序延迟
    uint8_t tracker_status;              //!< @brief 相机追踪状态
} Aim_Rx_info;

#pragma pack()
#pragma pack(1)
/* 目标位姿 */
typedef struct{
        float x;
        float y;
        float z;
        float HorizontalDistance;
}Pose_t;

/* 储存自瞄信息 */
typedef struct{
    enum {
        TIMESTAMP = 0,  // 时间戳同步
        COLOUR = 1,
        UPDATE = 2,
        UPDATING = 3,
    }Rx_State;                        //!< @brief 接收状态

    Pose_t Predicted_Center_Pose;     //!< @brief 惯性系下的目标中心点位姿
    float Predicted_Center_time;      //!< @brief 预测时间
    uint16_t  Fixed_Center_time;      //!< @brief 静态预测时间(拨弹盘转动，通信延迟等)
    float Predicted_Center_Yaw;       //!< @brief 预测中心的Y轴角度
    float Predicted_Center_Pitch;     //!< @brief 预测中心的P轴角度
    
    Pose_t Predicted_Armor_Pose[4];   //!< @brief 目标装甲板位姿（最多四块）
    float Predicted_Armor_time;
    uint16_t  Fixed_Armor_time;
    float Predicted_Armor_Yaw;
    float Predicted_Armor_Pitch; 
    
    float P_thre;                      //!< @brief 自动打弹角度阈值
    float Y_thre; 
    float K;                           //!< @brief 空气阻力系数
    float PitchOffset;                 //!< @brief 弹道补偿
    int64_t TimeStamp_setoff;          //!< @brief 补偿单片机时间戳
}Aim_Rx_t;

typedef struct{
	float Yaw;
	float Pitch;
	
	uint8_t PC_State;
    uint8_t Aim_Action;
	uint8_t Auto_Mode;
}Aim_Data;


#pragma pack()



#endif
