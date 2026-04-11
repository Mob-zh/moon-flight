#ifndef _FLIGHT_CONTROL_H_
#define _FLIGHT_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

// ==================== 飞行模式 ====================
typedef enum
{
    FLIGHT_MODE_ACRO = 0, // 角速度模式（纯手动）
    FLIGHT_MODE_ANGLE,    // 角度自稳模式
    FLIGHT_MODE_HOLD,     // 定点悬停
    FLIGHT_MODE_MAX
} flight_mode_t;

// ==================== PID数据结构 ====================
typedef struct
{
    float kp;     // 比例增益
    float ki;     // 积分增益
    float kd;     // 微分增益
    float sum;    // 积分累计
    float prev;   // 上一次误差（用于微分）
    float output; // PID输出
    float limit;  // 输出限幅
} pid_controller_t;

// ==================== 飞行控制结构体 ====================
typedef struct
{
    // 飞行模式
    flight_mode_t mode;
    flight_mode_t prev_mode;

    // 遥控器输入（期望值）
    float rc_roll;     // 横滚期望角速度/角度
    float rc_pitch;    // 俯仰期望角速度/角度
    float rc_yaw;      // 偏航期望角速度/角度
    float rc_throttle; // 油门 0-1

    // 角度环PID
    pid_controller_t pid_angle_roll;
    pid_controller_t pid_angle_pitch;
    pid_controller_t pid_angle_yaw;

    // 角速度环PID
    pid_controller_t pid_rate_roll;
    pid_controller_t pid_rate_pitch;
    pid_controller_t pid_rate_yaw;

    // 位置环PID（GPS模式）
    pid_controller_t pid_pos_n;
    pid_controller_t pid_pos_e;
    pid_controller_t pid_alt; // 高度环

    // 实际姿态（来自Mahony）
    float actual_roll;  // 实际横滚角 (deg)
    float actual_pitch; // 实际俯仰角 (deg)
    float actual_yaw;   // 实际偏航角 (deg)

    // 实际角速度（来自IMU）
    float gyro_roll;  // 实际横滚角速度 (deg/s)
    float gyro_pitch; // 实际俯仰角速度 (deg/s)
    float gyro_yaw;   // 实际偏航角速度 (deg/s)

    // 期望角速度（Angle模式由角度环输出）
    float desired_rate_roll;
    float desired_rate_pitch;
    float desired_rate_yaw;

    // 位置（来自EKF）
    float pos_n;   // 北向位置 (m)
    float pos_e;   // 东向位置 (m)
    float pos_alt; // 高度 (m)

    // 期望位置（GPS模式）
    float desired_pos_n;
    float desired_pos_e;
    float desired_alt;    // 目标高度 (m)

    // 高度补偿（定高用）
    float altitude_compensation;  // 油门补偿值

    // 电机输出 (0-1)
    float motor[4];

    // 状态
    uint8_t armed;     // 武装状态
    uint8_t ekf_ready; // EKF就绪
} flight_control_t;

// ==================== 函数声明 ====================

/**
 * @brief 飞行控制初始化
 * @param fc 飞行控制结构体
 */
void flight_control_init(flight_control_t *fc);

/**
 * @brief 飞行控制更新（8kHz调用）
 * @param fc 飞行控制结构体
 */
void flight_control_update(flight_control_t *fc);

/**
 * @brief 设置飞行模式
 * @param fc 飞行控制结构体
 * @param mode 飞行模式
 */
void flight_control_set_mode(flight_control_t *fc, flight_mode_t mode);

/**
 * @brief 设置遥控器输入
 * @param fc 飞行控制结构体
 * @param roll 横滚 (-1 ~ 1)
 * @param pitch 俯仰 (-1 ~ 1)
 * @param yaw 偏航 (-1 ~ 1)
 * @param throttle 油门 (0 ~ 1)
 */
void flight_control_set_rc(flight_control_t *fc,
                           float roll, float pitch,
                           float yaw, float throttle);

/**
 * @brief 设置实际姿态（由IMU调用）
 * @param fc 飞行控制结构体
 * @param roll 横滚角 (deg)
 * @param pitch 俯仰角 (deg)
 * @param yaw 偏航角 (deg)
 */
void flight_control_set_attitude(flight_control_t *fc,
                                 float roll, float pitch, float yaw);

/**
 * @brief 设置实际角速度（由IMU调用）
 * @param fc 飞行控制结构体
 * @param roll 横滚角速度 (deg/s)
 * @param pitch 俯仰角速度 (deg/s)
 * @param yaw 偏航角速度 (deg/s)
 */
void flight_control_set_gyro(flight_control_t *fc,
                             float roll, float pitch, float yaw);

/**
 * @brief 设置位置（由EKF调用）
 * @param fc 飞行控制结构体
 * @param north 北向位置 (m)
 * @param east 东向位置 (m)
 * @param alt 高度 (m)
 */
void flight_control_set_position(flight_control_t *fc,
                                 float north, float east, float alt);

/**
 * @brief 武装/解缆
 * @param fc 飞行控制结构体
 * @param armed true-武装, false-解缆
 */
void flight_control_set_armed(flight_control_t *fc, uint8_t armed);

/**
 * @brief 获取电机输出
 * @param fc 飞行控制结构体
 * @param motor 电机输出数组（4个元素，0-1）
 */
void flight_control_get_motor(flight_control_t *fc, float *motor);

/**
 * @brief PID参数设置
 * @param fc 飞行控制结构体
 * @param type PID类型 (0=angle, 1=rate, 2=pos)
 * @param axis 轴 (0=roll/x, 1=pitch/y, 2=yaw/z)
 * @param kp P值
 * @param ki I值
 * @param kd D值
 */
void flight_control_set_pid(flight_control_t *fc, int type, int axis,
                            float kp, float ki, float kd);

// 全局飞行控制实例
extern flight_control_t g_flight_control;

#endif /* _FLIGHT_CONTROL_H_ */
