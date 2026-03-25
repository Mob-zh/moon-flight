#ifndef _IMU_H_
#define _IMU_H_

#include <rtthread.h>

#define M_PI 3.1415926f

// 数据类型
typedef struct
{
    float rol; // 横滚角
    float pit; // 俯仰角
    float yaw; // 偏航角
} FLOAT_ANGLE;

typedef struct
{
    float X;
    float Y;
    float Z;
} FLOAT_XYZ;

// 全局对象
extern FLOAT_ANGLE Att_Angle; // 姿态角输出
extern FLOAT_XYZ   Gyr_filt;  // 滤波后角速度
extern FLOAT_XYZ   Acc_filt;  // 滤波后加速度计
extern float       imu_Kp;
extern float       imu_Ki;

// 初始化惯导系统
void IMU_init(void);

#endif
