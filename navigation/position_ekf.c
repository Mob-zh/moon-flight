#include "position_ekf.h"
#include <math.h>
#include <rtthread.h>

// ==================== 常量 ====================
#define DEG_TO_RAD   0.0174532925f
#define EARTH_RADIUS 6371000.0f // 地球半径 (m)

// ==================== 工具函数 ====================

// 经纬度转米（简化）
static void lat_lon_to_meters(float lat, float lon,
                              float ref_lat, float ref_lon,
                              float *north, float *east)
{
    // 简单墨卡托投影近似
    float d_lat = lat - ref_lat;
    float d_lon = lon - ref_lon;

    *north = d_lat * 111320.0f; // 1度纬度 ≈ 111.32km
    *east  = d_lon * 111320.0f * cosf(ref_lat * DEG_TO_RAD);
}

// 米转经纬度
static void meters_to_lat_lon(float north, float east,
                              float ref_lat, float ref_lon,
                              float *lat, float *lon)
{
    *lat = north / 111320.0f + ref_lat;
    *lon = east / (111320.0f * cosf(ref_lat * DEG_TO_RAD)) + ref_lon;
}

// ==================== API实现 ====================

void position_ekf_init(position_ekf_t *ekf, float init_lat, float init_lon)
{
    // 初始化状态
    ekf->pos_n     = 0.0f;
    ekf->pos_e     = 0.0f;
    ekf->vel_n     = 0.0f;
    ekf->vel_e     = 0.0f;
    ekf->baro_bias = 0.0f;

    // 初始化协方差
    ekf->P[0] = 100.0f; // pos_n方差
    ekf->P[1] = 100.0f; // pos_e方差
    ekf->P[2] = 10.0f;  // vel_n方差
    ekf->P[3] = 10.0f;  // vel_e方差
    ekf->P[4] = 10.0f;  // baro_bias方差

    // 设置参考位置
    ekf->ref_lat = init_lat;
    ekf->ref_lon = init_lon;

    // 设置过程噪声
    ekf->q_pos = 0.1f;
    ekf->q_vel = 1.0f;

    // 设置测量噪声
    ekf->r_gps_pos = 5.0f; // GPS位置噪声 5m
    ekf->r_gps_vel = 2.0f; // GPS速度噪声 2m/s
    ekf->r_baro    = 2.0f; // 气压高度噪声 2m

    // 状态标志
    ekf->gps_valid   = 0;
    ekf->baro_valid  = 0;
    ekf->initialized = 0;

    rt_kprintf("[EKF] Position EKF initialized\n");
}

void position_ekf_predict(position_ekf_t *ekf, float dt)
{
    if (!ekf->initialized)
    {
        return;
    }

    // 状态预测（匀速运动模型）
    ekf->pos_n += ekf->vel_n * dt;
    ekf->pos_e += ekf->vel_e * dt;

    // 协方差预测
    // P = F * P * F' + Q
    // 简化：只更新对角元素
    ekf->P[0] += (ekf->P[2] * dt * dt + ekf->q_pos) * dt; // pos_n方差
    ekf->P[1] += (ekf->P[3] * dt * dt + ekf->q_pos) * dt; // pos_e方差
    ekf->P[2] += ekf->q_vel * dt;                         // vel_n方差
    ekf->P[3] += ekf->q_vel * dt;                         // vel_e方差

    // 气压零偏随机游走
    ekf->P[4] += 0.01f * dt; // baro_bias方差
}

void position_ekf_update_gps(position_ekf_t *ekf,
                             int32_t lat, int32_t lon,
                             float vel_n, float vel_e,
                             uint16_t hdop)
{
    // HDOP > 300 (3.0) 认为数据不可靠
    if (hdop > 300)
    {
        ekf->gps_valid = 0;
        return;
    }

    // 第一次GPS数据：初始化参考位置
    if (!ekf->initialized)
    {
        ekf->ref_lat     = lat / 10000000.0f;
        ekf->ref_lon     = lon / 10000000.0f;
        ekf->initialized = 1;
        ekf->pos_n       = 0.0f;
        ekf->pos_e       = 0.0f;
        ekf->vel_n       = vel_n;
        ekf->vel_e       = vel_e;
        ekf->gps_valid   = 1;
        return;
    }

    // 转换GPS坐标到米
    float meas_n, meas_e;
    lat_lon_to_meters(lat / 10000000.0f, lon / 10000000.0f,
                      ekf->ref_lat, ekf->ref_lon,
                      &meas_n, &meas_e);

    // 调整测量噪声（根据HDOP）
    float gps_pos_noise = ekf->r_gps_pos * (hdop / 100.0f);

    // ===== 位置更新 =====
    // 状态: pos_n, pos_e
    // 测量: meas_n, meas_e

    // 卡尔曼增益
    float K_n = ekf->P[0] / (ekf->P[0] + gps_pos_noise);
    float K_e = ekf->P[1] / (ekf->P[1] + gps_pos_noise);

    // 状态更新
    ekf->pos_n += K_n * (meas_n - ekf->pos_n);
    ekf->pos_e += K_e * (meas_e - ekf->pos_e);

    // 协方差更新
    ekf->P[0] *= (1.0f - K_n);
    ekf->P[1] *= (1.0f - K_e);

    // ===== 速度更新 =====
    float gps_vel_noise = ekf->r_gps_vel;

    // 卡尔曼增益
    float Kv_n = ekf->P[2] / (ekf->P[2] + gps_vel_noise);
    float Kv_e = ekf->P[3] / (ekf->P[3] + gps_vel_noise);

    // 状态更新
    ekf->vel_n += Kv_n * (vel_n - ekf->vel_n);
    ekf->vel_e += Kv_e * (vel_e - ekf->vel_e);

    // 协方差更新
    ekf->P[2] *= (1.0f - Kv_n);
    ekf->P[3] *= (1.0f - Kv_e);

    ekf->gps_valid = 1;
}

void position_ekf_update_baro(position_ekf_t *ekf, float altitude)
{
    if (!ekf->initialized)
    {
        return;
    }

    // 气压计更新（用于高度估计）
    // 使用卡尔曼滤波融合气压高度，修正baro_bias

    // 测量值 = 真实高度 + 气压零偏
    // 预测值 = 测量高度 - 估计高度 - 估计零偏
    float meas_alt = altitude;
    float pred_alt = meas_alt - ekf->baro_bias;

    // 卡尔曼增益（融合气压高度）
    float K = ekf->P[4] / (ekf->P[4] + ekf->r_baro);

    // 更新气压零偏
    ekf->baro_bias += K * pred_alt;

    // 协方差更新
    ekf->P[4] *= (1.0f - K);

    ekf->baro_valid = 1;
}

void position_ekf_get_position(position_ekf_t *ekf, float *north, float *east, float *alt)
{
    if (north)
        *north = ekf->pos_n;
    if (east)
        *east = ekf->pos_e;
    if (alt)
        *alt = ekf->baro_bias; // 返回修正后的气压高度
}

void position_ekf_get_velocity(position_ekf_t *ekf, float *vel_n, float *vel_e)
{
    if (vel_n)
        *vel_n = ekf->vel_n;
    if (vel_e)
        *vel_e = ekf->vel_e;
}

bool position_ekf_ready(position_ekf_t *ekf)
{
    return ekf->initialized && ekf->gps_valid;
}

void position_ekf_reset(position_ekf_t *ekf)
{
    ekf->pos_n       = 0.0f;
    ekf->pos_e       = 0.0f;
    ekf->vel_n       = 0.0f;
    ekf->vel_e       = 0.0f;
    ekf->baro_bias   = 0.0f;
    ekf->initialized = 0;
    ekf->gps_valid   = 0;
    ekf->baro_valid  = 0;
}
