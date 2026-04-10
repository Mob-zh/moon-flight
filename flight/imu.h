#ifndef _IMU_H_
#define _IMU_H_

#include <rtthread.h>

#define M_PI 3.1415926f

// ==================== 加速度计初始方向配置 ====================
// 初始偏置宏：设定上电时的加速度计"基准"（单位：g）
// 飞机平放时，加速度计应输出 (0, 0, 1)（Z轴向上为正）
// 如果要设定不同基准，可修改此宏
#define IMU_ACC_INIT_BIAS_X  0.0f   // X轴初始偏置 (g)
#define IMU_ACC_INIT_BIAS_Y  0.0f   // Y轴初始偏置 (g)
#define IMU_ACC_INIT_BIAS_Z  1.0f   // Z轴初始偏置 (g) - 默认水平朝上

// 启用初始偏置功能（默认关闭，保持原有行为）
// 设为1时：首次上电记录加速度计值，之后减去初始偏置，实现姿态归零到预设基准
// 设为0时：保持原有行为，显示当前实际姿态
#define IMU_USE_INIT_BIAS    0

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
