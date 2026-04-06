/**
 * @file blackbox.h
 * @brief Blackbox 黑匣子主接口 - 与Betaflight协议兼容
 *
 * 适用于RT-Thread穿越机飞控系统
 */
#ifndef _BLACKBOX_H_
#define _BLACKBOX_H_

#include <stdint.h>
#include <stdbool.h>

// ==================== 配置 ====================

// 电机数量
#ifndef MOTOR_COUNT
#define MOTOR_COUNT 4
#endif

// I帧间隔（每多少迭代记录一次I帧）
#ifndef BB_I_INTERVAL
#define BB_I_INTERVAL 32
#endif

// P帧间隔（每多少迭代记录一次P帧）
#ifndef BB_P_INTERVAL
#define BB_P_INTERVAL 8
#endif

// ==================== 事件类型 ====================
typedef enum {
    BB_EVENT_SYNC_BEEP = 0,
    BB_EVENT_INFLIGHT_ADJUSTMENT = 13,
    BB_EVENT_LOGGING_RESUME = 14,
    BB_EVENT_DISARM = 15,
    BB_EVENT_FLIGHTMODE = 30,
    BB_EVENT_LOG_END = 255
} bb_event_t;

// ==================== 数据结构 ====================

/**
 * @brief Blackbox主状态数据结构
 *
 * 与Betaflight协议兼容的字段定义
 */
typedef struct {
    uint32_t time;               // 时间（微秒）

    // PID数据 (x, y, z) = (roll, pitch, yaw)
    int32_t axisPID_P[3];        // P项
    int32_t axisPID_I[3];        // I项
    int32_t axisPID_D[3];        // D项
    int32_t axisPID_F[3];        // F项(前馈)

    // RC指令 [0]=roll [1]=pitch [2]=yaw [3]=throttle
    int16_t rcCommand[4];

    // 设定点
    int16_t setpoint[4];

    // 陀螺仪原始值
    int16_t gyroADC[3];

    // 加速度计平滑值
    int16_t accSmooth[3];

    // 电机输出 (实际值 - 1000)
    int16_t motor[MOTOR_COUNT];

    // 电池电压 (0.1V为单位, 即电压=vbats Latest/10)
    uint16_t vbatLatest;

    // 电池电流 (0.01A为单位)
    int32_t amperageLatest;

    // 气压计高度
    int32_t baroAlt;

    // RSSI
    uint16_t rssi;
} bb_main_state_t;

/**
 * @brief Blackbox慢速状态
 */
typedef struct {
    uint32_t flightModeFlags;  // 飞行模式标志
    uint8_t stateFlags;        // 状态标志
    uint8_t failsafePhase;     // 失控保护阶段
    uint8_t rxSignalReceived; // 接收信号标志
    uint8_t rxFlightChannelsValid; // RC通道有效
} bb_slow_state_t;

// ==================== 函数声明 ====================

/**
 * @brief 初始化Blackbox
 */
void bb_init(void);

/**
 * @brief 打开日志
 */
void bb_open(void);

/**
 * @brief 关闭日志
 */
void bb_close(void);

/**
 * @brief 更新Blackbox（每循环调用）
 * @param state 主状态数据
 */
void bb_update(bb_main_state_t *state);

/**
 * @brief 写入事件
 * @param event 事件类型
 * @param data 事件数据
 */
void bb_log_event(bb_event_t event, uint32_t data);

/**
 * @brief 检查Blackbox是否已打开
 */
bool bb_is_logging(void);

/**
 * @brief 设置电压参考值（用于I帧编码）
 * @param vbat 电压值
 */
void bb_set_vbat_ref(uint16_t vbat);

/**
 * @brief 获取I帧间隔
 */
int bb_get_i_interval(void);

/**
 * @brief 获取P帧间隔
 */
int bb_get_p_interval(void);

#endif /* _BLACKBOX_H_ */
