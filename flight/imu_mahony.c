#include "imu_mahony.h"
#include <math.h>

// ==================== 常量定义 ====================
#define RAD_TO_DEG 57.2957795f  // 弧度转角度

// ==================== 快速平方根倒数（卡马克算法） ====================
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    union { float f; uint32_t i; } val;
    val.f = x;
    val.i = 0x5f375a86 - (val.i >> 1);
    val.f = val.f * (1.5f - halfx * val.f * val.f);
    return val.f;
}

// ==================== 动态Kp计算 ====================
static float calc_dynamic_Kp(mahony_filter_t *filter, float gx, float gy, float gz)
{
    // 计算角速度模值
    float gyro_mag = sqrtf(gx * gx + gy * gy + gz * gz);

    // 根据角速度动态调整Kp：base_Kp ~ (base_Kp + 1.5)
    float dynamic_Kp = filter->base_Kp + (gyro_mag * 0.1f);
    if (dynamic_Kp > filter->base_Kp + 1.5f) {
        dynamic_Kp = filter->base_Kp + 1.5f;
    }

    return dynamic_Kp;
}

// ==================== 动态加速度权重 ====================
static float calc_acc_weight(mahony_filter_t *filter, float ax, float ay, float az)
{
    // 计算加速度模值
    float acc_mag = sqrtf(ax * ax + ay * ay + az * az);

    // 计算加速度变化量（相对于1g）
    float acc_delta = fabsf(acc_mag - 1.0f);

    // 静止时(acc_delta < 0.1g)权重为1，高机动时降低权重
    float weight = 1.0f;
    if (acc_delta > 0.1f) {
        weight = 1.0f / (1.0f + acc_delta * 10.0f);
    }

    // 平滑过渡
    filter->acc_weight = 0.9f * filter->acc_weight + 0.1f * weight;
    filter->last_acc_mag = acc_mag;

    return filter->acc_weight;
}

// ==================== API实现 ====================

void mahony_init(mahony_filter_t *filter, float dt, float base_Kp, float Ki)
{
    // 初始化四元数为单位四元数
    filter->q0 = 1.0f;
    filter->q1 = 0.0f;
    filter->q2 = 0.0f;
    filter->q3 = 0.0f;

    // 积分误差清零
    filter->exInt = 0.0f;
    filter->eyInt = 0.0f;
    filter->ezInt = 0.0f;

    // 参数设置
    filter->base_Kp = base_Kp;
    filter->Kp = base_Kp;
    filter->Ki = Ki;

    // 加速度权重初始化
    filter->acc_weight = 1.0f;
    filter->last_acc_mag = 1.0f;

    // 欧拉角初始化
    filter->roll = 0.0f;
    filter->pitch = 0.0f;
    filter->yaw = 0.0f;

    // 采样周期
    filter->dt = dt;
}

void mahony_update(mahony_filter_t *filter,
                   float gx, float gy, float gz,
                   float ax, float ay, float az)
{
    float q0 = filter->q0;
    float q1 = filter->q1;
    float q2 = filter->q2;
    float q3 = filter->q3;

    float vx, vy, vz;  // 估计重力向量
    float ex, ey, ez;  // 误差
    float norm;

    // 计算四元数乘积（复用）
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
    if ((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)) {
        return;
    }

    // 归一化加速度计数据
    norm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    // 陀螺仪积分估计重力向量（机体坐标系）
    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    // 计算误差（向量叉乘）
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    // 计算动态加速度权重
    float acc_w = calc_acc_weight(filter, ax, ay, az);

    // 误差积分（乘以加速度权重）
    filter->exInt += ex * filter->Ki * acc_w;
    filter->eyInt += ey * filter->Ki * acc_w;
    filter->ezInt += ez * filter->Ki * acc_w;

    // 积分误差归一化（防止饱和）
    filter->exInt *= 0.99f;
    filter->eyInt *= 0.99f;
    filter->ezInt *= 0.99f;

    // 动态计算Kp
    float current_Kp = calc_dynamic_Kp(filter, gx, gy, gz);
    filter->Kp = current_Kp;

    // 将误差PI补偿到陀螺仪
    gx += current_Kp * ex * acc_w + filter->exInt;
    gy += current_Kp * ey * acc_w + filter->eyInt;
    gz += current_Kp * ez * acc_w + filter->ezInt;

    // 四元数微分方程
    float half_dt = filter->dt * 0.5f;
    q0 += (-q1 * gx - q2 * gy - q3 * gz) * half_dt;
    q1 += (q0 * gx + q2 * gz - q3 * gy) * half_dt;
    q2 += (q0 * gy - q1 * gz + q3 * gx) * half_dt;
    q3 += (q0 * gz + q1 * gy - q2 * gx) * half_dt;

    // 归一化四元数
    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    filter->q0 = q0 * norm;
    filter->q1 = q1 * norm;
    filter->q2 = q2 * norm;
    filter->q3 = q3 * norm;

    // 四元数符号修正（防止 -q 跳变）
    static float last_q0 = 1.0f;
    if (filter->q0 * last_q0 < 0.0f) {
        filter->q0 = -filter->q0;
        filter->q1 = -filter->q1;
        filter->q2 = -filter->q2;
        filter->q3 = -filter->q3;
    }
    last_q0 = filter->q0;

    // 计算欧拉角（Z->Y->X顺序）
    // Roll (横滚角)
    filter->roll = -asinf(2.0f * (q1 * q3 - q0 * q2)) * RAD_TO_DEG;

    // Pitch (俯仰角)
    filter->pitch = -atan2f(2.0f * q2 * q3 + 2.0f * q0 * q1,
                           q0q0 - q1q1 - q2q2 + q3q3) * RAD_TO_DEG;

    // Yaw (偏航角) - 使用积分
    filter->yaw += gz * RAD_TO_DEG * filter->dt;
}

void mahony_get_euler(mahony_filter_t *filter, float *roll, float *pitch, float *yaw)
{
    if (roll) *roll = filter->roll;
    if (pitch) *pitch = filter->pitch;
    if (yaw) *yaw = filter->yaw;
}

void mahony_reset(mahony_filter_t *filter)
{
    filter->q0 = 1.0f;
    filter->q1 = 0.0f;
    filter->q2 = 0.0f;
    filter->q3 = 0.0f;

    filter->exInt = 0.0f;
    filter->eyInt = 0.0f;
    filter->ezInt = 0.0f;

    filter->roll = 0.0f;
    filter->pitch = 0.0f;
    filter->yaw = 0.0f;

    filter->acc_weight = 1.0f;
}
