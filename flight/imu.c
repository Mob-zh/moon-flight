#include "imu.h"
#include "ano_data.h"
#include "icm42688.h"
#include <math.h>
#include <rtthread.h>
#include <stdint.h>

// ==================== 数据类型定义 ====================
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

// ==================== 算法参数 ====================
#define Kp       0.3f   // 比例增益控制加速度计的收敛速率
#define Ki       0.001f // 积分增益控制陀螺偏差的收敛速度
#define halfT    0.005f // 采样周期的一半 (对应100Hz)
#define RadtoDeg 57.3f  // 弧度转角度系数

// ==================== 全局变量 ====================
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;  // 四元数元素
float exInt = 0, eyInt = 0, ezInt = 0; // 积分误差

FLOAT_ANGLE Att_Angle = {0};                // 姿态角输出
FLOAT_XYZ   Acc_filt = {0}, Gyr_filt = {0}; // 滤波后的传感器数据

// ==================== 快速平方根倒数 ====================
static float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

// ==================== 数据准备函数 ====================
void IMU_Prepare_Data(void)
{
    // 读取传感器数据
    g_icm_accgyro.readGyro(&g_icm_accgyro);
    g_icm_accgyro.readAcc(&g_icm_accgyro);

    // 减去零偏
    g_icm_accgyro.accData[0] -= g_icm_accgyro.accData_offset[0];
    g_icm_accgyro.accData[1] -= g_icm_accgyro.accData_offset[1];
    g_icm_accgyro.accData[2] -= g_icm_accgyro.accData_offset[2];

    g_icm_accgyro.gyroData[0] -= g_icm_accgyro.gyroData_offset[0];
    g_icm_accgyro.gyroData[1] -= g_icm_accgyro.gyroData_offset[1];
    g_icm_accgyro.gyroData[2] -= g_icm_accgyro.gyroData_offset[2];

    // 陀螺仪数据处理：LSB → 弧度/秒
    Gyr_filt.X = (float)(g_icm_accgyro.gyroData[0] * g_icm_accgyro.gyroScale * M_PI / 180.0f);
    Gyr_filt.Y = (float)(g_icm_accgyro.gyroData[1] * g_icm_accgyro.gyroScale * M_PI / 180.0f);
    Gyr_filt.Z = (float)(g_icm_accgyro.gyroData[2] * g_icm_accgyro.gyroScale * M_PI / 180.0f);

    // 加速度计数据处理：LSB → g
    Acc_filt.X = (float)(g_icm_accgyro.accData[0] * g_icm_accgyro.accScale);
    Acc_filt.Y = (float)(g_icm_accgyro.accData[1] * g_icm_accgyro.accScale);
    Acc_filt.Z = (float)(g_icm_accgyro.accData[2] * g_icm_accgyro.accScale);
}

// ==================== 核心姿态解算算法 ====================
void IMUupdate(FLOAT_XYZ *Gyr_filt, FLOAT_XYZ *Acc_filt, FLOAT_ANGLE *Att_Angle)
{
    float ax = Acc_filt->X, ay = Acc_filt->Y, az = Acc_filt->Z;
    float gx = Gyr_filt->X, gy = Gyr_filt->Y, gz = Gyr_filt->Z;
    float vx, vy, vz;
    float ex, ey, ez;
    float norm;

    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    // 检查加速度计数据有效性
    if (ax * ay * az == 0)
        return;

    // 归一化加速度计数据
    norm = invSqrt(ax * ax + ay * ay + az * az);
    ax   = ax * norm;
    ay   = ay * norm;
    az   = az * norm;

    // 陀螺仪积分估计重力向量(机体坐标系)
    vx = 2 * (q1 * q3 - q0 * q2);   // 矩阵(3,1)项
    vy = 2 * (q0 * q1 + q2 * q3);   // 矩阵(3,2)项
    vz = q0q0 - q1q1 - q2q2 + q3q3; // 矩阵(3,3)项

    // 向量叉乘得到误差
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    // 误差积分
    exInt = exInt + ex * Ki;
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;

    // 将误差PI补偿到陀螺仪
    gx = gx + Kp * ex + exInt;
    gy = gy + Kp * ey + eyInt;
    gz = gz + Kp * ez + ezInt;

    // 四元数微分方程
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

    // 归一化四元数
    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0   = q0 * norm;
    q1   = q1 * norm;
    q2   = q2 * norm;
    q3   = q3 * norm;

    // 发送四元数数据给上位机
    // ANO_DT_Send_Att_RawData(q0, q1, q2, q3);

    // 四元数转换成欧拉角(Z->Y->X)
    // 偏航角YAW - 使用阈值过滤减少漂移
    if ((Gyr_filt->Z * RadtoDeg > 1.0f) || (Gyr_filt->Z * RadtoDeg < -1.0f))
    {
        Att_Angle->yaw += Gyr_filt->Z * RadtoDeg * halfT * 2.0f;
    }

    // 横滚角ROLL
    Att_Angle->rol = -asin(2.0f * (q1 * q3 - q0 * q2)) * RadtoDeg;

    // 俯仰角PITCH
    Att_Angle->pit = -atan2(2.0f * q2 * q3 + 2.0f * q0 * q1, q0q0 - q1q1 - q2q2 + q3q3) * RadtoDeg;
}

// ==================== 线程入口函数 ====================
static void IMU_update_thread_entry(void *parameter)
{
    // char status[64];
    while (1)
    {
        // 准备传感器数据
        IMU_Prepare_Data();

        // 执行姿态解算
        IMUupdate(&Gyr_filt, &Acc_filt, &Att_Angle);

        // 发送原始数据给上位机（可选）
        // ANO_DT_Send_IMU_RawData(g_icm_accgyro.accData[0], g_icm_accgyro.accData[1], g_icm_accgyro.accData[2],
        //                         g_icm_accgyro.gyroData[0], g_icm_accgyro.gyroData[1], g_icm_accgyro.gyroData[2], 0);

        // 发送姿态角数据给上位机
        // 参数顺序: A(俯仰pitch), B(横滚roll), C(偏航yaw)
        // ANO_DT_Send_Euler_Angles(Att_Angle.pit, Att_Angle.rol, Att_Angle.yaw);

        // sprintf(status, "rol: %f, pit: %f, yaw: %f\n", Att_Angle.rol, Att_Angle.pit, Att_Angle.yaw);
        // rt_kprintf("%s", status);

        // 延时10ms，对应100Hz采样率
        rt_thread_mdelay(10);
    }
}

// ==================== IMU初始化函数 ====================
void IMU_init(void)
{
    accgyro_init(&g_icm_accgyro);

    // 创建IMU更新线程
    rt_thread_t imu_thread = rt_thread_create("imu_update", IMU_update_thread_entry, RT_NULL, 1024, 20, 10);
    if (imu_thread != RT_NULL)
        rt_thread_startup(imu_thread);
}