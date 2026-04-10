#include "imu.h"
#include "ano_data.h"
#include "icm42688.h"
#include <math.h>
#include <rtthread.h>
#include <stdint.h>
#include <string.h>

// ==================== 算法参数 ====================
#define halfT    0.0000625f // 采样周期的一半 (对应8kHz)
#define RadtoDeg 57.3f      // 弧度转角度系数

// ==================== 全局变量 ====================
rt_sem_t    imu_sem = RT_NULL;               // IMU信号量（定时器中断→IMU线程）
float       imu_Kp  = 2.0f;                  // 比例增益 0.5 - 2.0 (动态调整)
float       imu_Ki  = 0.0001f;               // 积分增益 0.0001
float       q0 = 1, q1 = 0, q2 = 0, q3 = 0;  // 四元数元素
float       exInt = 0, eyInt = 0, ezInt = 0; // 积分误差
rt_thread_t imu_thread = RT_NULL;

// 初始偏置变量（首次IMU数据有效时自动记录）
static bool init_bias_recorded = false;
static float init_acc_bias[3] = {0};

// 加速度计零偏（用于手动校准）
float acc_offset[3] = {0, 0, 0};

FLOAT_ANGLE Att_Angle = {0};                // 姿态角输出
FLOAT_XYZ   Acc_filt = {0}, Gyr_filt = {0}; // 滤波后的传感器数据

char float_str[64] = {0};

extern rt_sem_t control_sem; // 控制信号量（IMU线程→控制线程）

// ==================== 快速平方根倒数（卡马克算法） ====================
static float invSqrt(float x)
{
    // Carmack快速平方根倒数
    float halfx = 0.5f * x;
    union
    {
        float    f;
        uint32_t i;
    } val;
    val.f = x;
    val.i = 0x5f375a86 - (val.i >> 1);
    val.f = val.f * (1.5f - halfx * val.f * val.f);
    return val.f;
}

// ==================== 动态Kp计算 ====================
static float base_Kp = 0.5f; // 基础增益

static float IMU_CalcDynamicKp(float gx, float gy, float gz)
{
    // 计算角速度模值
    float gyro_mag = sqrtf(gx * gx + gy * gy + gz * gz);

    // 根据角速度动态调整Kp：0.5 ~ 2.0
    float dynamic_Kp = base_Kp + (gyro_mag * 0.1f);
    if (dynamic_Kp > 2.0f)
        dynamic_Kp = 2.0f;

    return dynamic_Kp;
}

// ==================== 数据准备函数 ====================
void IMU_Prepare_Data(void)
{
    // 读取传感器数据
    g_icm_accgyro.readGyro(&g_icm_accgyro);
    g_icm_accgyro.readAcc(&g_icm_accgyro);

    // 减去零偏（陀螺仪）
    g_icm_accgyro.gyroData[0] -= g_icm_accgyro.gyroData_offset[0];
    g_icm_accgyro.gyroData[1] -= g_icm_accgyro.gyroData_offset[1];
    g_icm_accgyro.gyroData[2] -= g_icm_accgyro.gyroData_offset[2];

    // 减去加速度计零偏
    g_icm_accgyro.accData[0] -= (g_icm_accgyro.accData_offset[0] + (int16_t)(acc_offset[0] / g_icm_accgyro.accScale));
    g_icm_accgyro.accData[1] -= (g_icm_accgyro.accData_offset[1] + (int16_t)(acc_offset[1] / g_icm_accgyro.accScale));
    g_icm_accgyro.accData[2] -= (g_icm_accgyro.accData_offset[2] + (int16_t)(acc_offset[2] / g_icm_accgyro.accScale));

    // 陀螺仪数据处理：LSB → 弧度/秒
    Gyr_filt.X = (float)(g_icm_accgyro.gyroData[0] * g_icm_accgyro.gyroScale * M_PI / 180.0f);
    Gyr_filt.Y = (float)(g_icm_accgyro.gyroData[1] * g_icm_accgyro.gyroScale * M_PI / 180.0f);
    Gyr_filt.Z = (float)(g_icm_accgyro.gyroData[2] * g_icm_accgyro.gyroScale * M_PI / 180.0f);

    // 加速度计数据处理：LSB → g
    Acc_filt.X = (float)(g_icm_accgyro.accData[0] * g_icm_accgyro.accScale);
    Acc_filt.Y = (float)(g_icm_accgyro.accData[1] * g_icm_accgyro.accScale);
    Acc_filt.Z = (float)(g_icm_accgyro.accData[2] * g_icm_accgyro.accScale);

    // 首次记录初始偏置（用于姿态归零）
    if (!init_bias_recorded && IMU_USE_INIT_BIAS)
    {
        init_acc_bias[0] = Acc_filt.X;
        init_acc_bias[1] = Acc_filt.Y;
        init_acc_bias[2] = Acc_filt.Z;
        init_bias_recorded = true;
        rt_kprintf("IMU init bias recorded: %.3f, %.3f, %.3f\n",
                   init_acc_bias[0], init_acc_bias[1], init_acc_bias[2]);
    }
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

    // 应用初始偏置修正（使姿态归零到设定基准）
    if (IMU_USE_INIT_BIAS && init_bias_recorded)
    {
        ax -= (init_acc_bias[0] - IMU_ACC_INIT_BIAS_X);
        ay -= (init_acc_bias[1] - IMU_ACC_INIT_BIAS_Y);
        az -= (init_acc_bias[2] - IMU_ACC_INIT_BIAS_Z);
    }

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
    exInt = exInt + ex * imu_Ki;
    eyInt = eyInt + ey * imu_Ki;
    ezInt = ezInt + ez * imu_Ki;

    // 积分误差归一化（防止饱和）
    exInt *= 0.99f;
    eyInt *= 0.99f;
    ezInt *= 0.99f;

    // 动态计算Kp（根据角速度调整）
    float current_Kp = IMU_CalcDynamicKp(gx, gy, gz);

    // 将误差PI补偿到陀螺仪
    gx = gx + current_Kp * ex + exInt;
    gy = gy + current_Kp * ey + eyInt;
    gz = gz + current_Kp * ez + ezInt;

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

    // 四元数符号修正：防止 q 变成 -q 时显示跳变
    static float last_q0 = 1;
    if (q0 * last_q0 < 0)
    {
        q0 = -q0;
        q1 = -q1;
        q2 = -q2;
        q3 = -q3;
    }
    last_q0 = q0;

    // 发送四元数数据给上位机
    // ANO_DT_Send_Att_RawData(q0, q1, q2, q3);

    // 四元数转换成欧拉角(Z->Y->X)
    // 偏航角YAW - 使用阈值过滤减少漂移
    if ((Gyr_filt->Z * RadtoDeg > 0.4f) || (Gyr_filt->Z * RadtoDeg < -0.4f))
    {
        Att_Angle->yaw += Gyr_filt->Z * RadtoDeg * halfT * 2.0f; // 0 - 2
    }

    // 横滚角ROLL
    Att_Angle->rol = -asin(2.0f * (q1 * q3 - q0 * q2)) * RadtoDeg;

    // 俯仰角PITCH
    Att_Angle->pit = -atan2(2.0f * q2 * q3 + 2.0f * q0 * q1, q0q0 - q1q1 - q2q2 + q3q3) * RadtoDeg;
}

// ==================== IMU初始化函数 ====================
void IMU_init(void)
{
    // 初始化信号量
    imu_sem = rt_sem_create("imu_sem", 0, RT_IPC_FLAG_PRIO);

    accgyro_init(&g_icm_accgyro);
}

void imu_set_Kp(int argc, char *argv[])
{
    imu_Kp = atof(argv[1]);
}

void imu_set_Ki(int argc, char *argv[])
{
    imu_Ki = atof(argv[1]);
}

void imu_show_Kp_Ki(void)
{
    char str[64];
    sprintf(str, "imu_Kp = %f, imu_Ki = %f\r\n", imu_Kp, imu_Ki);
    rt_kprintf(str);
}

// ==================== 手动校准功能 ====================
void imu_calibrate(int argc, char *argv[])
{
    rt_kprintf("Starting accelerometer calibration...\n");
    rt_kprintf("Please keep the aircraft level and stationary!\n");

    // 采样200次取平均
    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    int32_t count = 200;

    for (int i = 0; i < count; i++)
    {
        g_icm_accgyro.readAcc(&g_icm_accgyro);
        sum_ax += (float)g_icm_accgyro.accData[0] * g_icm_accgyro.accScale;
        sum_ay += (float)g_icm_accgyro.accData[1] * g_icm_accgyro.accScale;
        sum_az += (float)g_icm_accgyro.accData[2] * g_icm_accgyro.accScale;
        rt_thread_mdelay(5);
    }

    // 计算零偏（假设水平放置时Z轴应为1g）
    acc_offset[0] = sum_ax / count - 0.0f;  // X轴期望为0
    acc_offset[1] = sum_ay / count - 0.0f;  // Y轴期望为0
    acc_offset[2] = sum_az / count - 1.0f;  // Z轴期望为1g

    rt_kprintf("Calibration done!\n");
    rt_kprintf("acc_offset: X=%.4f Y=%.4f Z=%.4f\n",
               acc_offset[0], acc_offset[1], acc_offset[2]);
}

void imu_calibrate_show(void)
{
    rt_kprintf("Current acc_offset: X=%.4f Y=%.4f Z=%.4f\n",
               acc_offset[0], acc_offset[1], acc_offset[2]);
    rt_kprintf("init_acc_bias: X=%.3f Y=%.3f Z=%.3f\n",
               init_acc_bias[0], init_acc_bias[1], init_acc_bias[2]);
    rt_kprintf("init_bias_recorded: %d\n", init_bias_recorded);
}

MSH_CMD_EXPORT(imu_set_Kp, "set imu Kp");
MSH_CMD_EXPORT(imu_set_Ki, "set imu Ki");
MSH_CMD_EXPORT(imu_show_Kp_Ki, "show imu Kp and Ki");
MSH_CMD_EXPORT(imu_calibrate, "calibrate acc offset");
MSH_CMD_EXPORT(imu_calibrate_show, "show acc offset");
