#ifndef __IMU_MAHONY_H
#define __IMU_MAHONY_H

#include <stdbool.h>
#include <stdint.h>

// ==================== Mahony滤波器结构体 ====================
typedef struct
{
    // 四元数
    float q0, q1, q2, q3;

    // 积分误差
    float exInt, eyInt, ezInt;

    // PI参数
    float Kp; // 比例增益
    float Ki; // 积分增益

    // 动态加速度权重
    float acc_weight;
    float last_acc_mag;

    // 基础Kp（用于动态调整）
    float base_Kp;

    // 输出欧拉角（弧度）
    float roll;
    float pitch;
    float yaw;

    // 采样周期
    float dt;
} mahony_filter_t;

// ==================== 函数声明 ====================

/**
 * @brief 初始化Mahony滤波器
 * @param filter 滤波器结构体指针
 * @param dt 采样周期（秒）
 * @param base_Kp 基础比例增益
 * @param Ki 积分增益
 */
void mahony_init(mahony_filter_t *filter, float dt, float base_Kp, float Ki);

/**
 * @brief Mahony滤波器更新
 * @param filter 滤波器结构体指针
 * @param gx 陀螺仪X轴 (rad/s)
 * @param gy 陀螺仪Y轴 (rad/s)
 * @param gz 陀螺仪Z轴 (rad/s)
 * @param ax 加速度计X轴 (g)
 * @param ay 加速度计Y轴 (g)
 * @param az 加速度计Z轴 (g)
 */
void mahony_update(mahony_filter_t *filter,
                   float gx, float gy, float gz,
                   float ax, float ay, float az);

/**
 * @brief 获取欧拉角
 * @param filter 滤波器结构体指针
 * @param roll 输出横滚角（度）
 * @param pitch 输出俯仰角（度）
 * @param yaw 输出偏航角（度）
 */
void mahony_get_euler(mahony_filter_t *filter, float *roll, float *pitch, float *yaw);

/**
 * @brief 重置滤波器
 * @param filter 滤波器结构体指针
 */
void mahony_reset(mahony_filter_t *filter);

#endif /* __IMU_MAHONY_H */
