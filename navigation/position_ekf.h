#ifndef _POSITION_EKF_H_
#define _POSITION_EKF_H_

#include <stdbool.h>
#include <stdint.h>

// ==================== 位置EKF结构体 ====================
typedef struct {
    // 状态向量: [pos_n, pos_e, vel_n, vel_e]
    float pos_n;   // 北向位置 (m)
    float pos_e;   // 东向位置 (m)
    float vel_n;   // 北向速度 (m/s)
    float vel_e;   // 东向速度 (m/s)

    // 气压计零偏估计 (m)
    float baro_bias;

    // 协方差矩阵（简化为对角元素）
    float P[5];  // pos_n, pos_e, vel_n, vel_e, baro_bias 的方差

    // 参考位置（用于经纬度转米）
    float ref_lat;   // 参考纬度 (deg)
    float ref_lon;   // 参考经度 (deg)

    // 过程噪声
    float q_pos;     // 位置过程噪声
    float q_vel;     // 速度过程噪声

    // 测量噪声
    float r_gps_pos;    // GPS位置噪声 (m)
    float r_gps_vel;    // GPS速度噪声 (m/s)
    float r_baro;       // 气压高度噪声 (m)

    // 状态标志
    uint8_t gps_valid;     // GPS数据有效
    uint8_t baro_valid;    // 气压计数据有效
    uint8_t initialized;   // EKF初始化完成
} position_ekf_t;

// ==================== 函数声明 ====================

/**
 * @brief 初始化位置EKF
 * @param ekf EKF结构体指针
 * @param init_lat 初始纬度 (deg)
 * @param init_lon 初始经度 (deg)
 */
void position_ekf_init(position_ekf_t *ekf, float init_lat, float init_lon);

/**
 * @brief EKF预测步骤（每周期调用，8kHz）
 * @param ekf EKF结构体指针
 * @param dt 时间步长 (秒)
 */
void position_ekf_predict(position_ekf_t *ekf, float dt);

/**
 * @brief GPS位置更新
 * @param ekf EKF结构体指针
 * @param lat 纬度 (deg * 1e-7)
 * @param lon 经度 (deg * 1e-7)
 * @param vel_n 北向速度 (m/s)
 * @param vel_e 东向速度 (m/s)
 * @param hdop 水平精度因子 (*100)
 */
void position_ekf_update_gps(position_ekf_t *ekf,
                              int32_t lat, int32_t lon,
                              float vel_n, float vel_e,
                              uint16_t hdop);

/**
 * @brief 气压计高度更新
 * @param ekf EKF结构体指针
 * @param altitude 气压高度 (m)
 */
void position_ekf_update_baro(position_ekf_t *ekf, float altitude);

/**
 * @brief 获取估计位置
 * @param ekf EKF结构体指针
 * @param north 北向位置输出 (m)
 * @param east 东向位置输出 (m)
 * @param alt 高度输出 (m)
 */
void position_ekf_get_position(position_ekf_t *ekf, float *north, float *east, float *alt);

/**
 * @brief 获取估计速度
 * @param ekf EKF结构体指针
 * @param vel_n 北向速度输出 (m/s)
 * @param vel_e 东向速度输出 (m/s)
 */
void position_ekf_get_velocity(position_ekf_t *ekf, float *vel_n, float *vel_e);

/**
 * @brief 检查EKF是否就绪
 * @param ekf EKF结构体指针
 * @retval true-EKF就绪
 */
bool position_ekf_ready(position_ekf_t *ekf);

/**
 * @brief 重置EKF
 * @param ekf EKF结构体指针
 */
void position_ekf_reset(position_ekf_t *ekf);

#endif /* _POSITION_EKF_H_ */
