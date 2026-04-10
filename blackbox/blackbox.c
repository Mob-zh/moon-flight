/**
 * @file blackbox.c
 * @brief Blackbox 黑匣子主实现 - 与Betaflight协议兼容
 *
 * 适用于RT-Thread穿越机飞控系统
 */
#include "blackbox.h"
#include "blackbox_encode.h"
#include "blackbox_sdio.h"
#include "blackbox_fatfs.h"
#include <rtthread.h>
#include <string.h>

// ==================== 常量定义 ====================

// I帧间隔
#define I_INTERVAL BB_I_INTERVAL
// P帧间隔
#define P_INTERVAL BB_P_INTERVAL
// 最小油门
#define MIN_THROTTLE 1000
// 最大油门
#define MAX_THROTTLE 2000

// ==================== 私有变量 ====================

// 全局存储设备实例（在blackbox.h中声明）
bbStorage_t g_bb_storage = {
    .ops         = NULL,
    .initialized = false,
    .logging     = false,
    .log_number  = 0,
};

static volatile int bb_logging       = 0; // 日志状态
static uint32_t     bb_iteration     = 0; // 循环迭代计数
static uint16_t     bb_p_frame_index = 0; // P帧索引
static uint16_t     bb_i_frame_index = 0; // I帧索引
static uint32_t     bb_start_time    = 0; // 开始时间（微秒）
static uint16_t     bb_vbat_ref      = 0; // 电压参考值

// 历史状态（用于P帧差分计算）
static bb_main_state_t bb_history[3];
static int             bb_history_index = 0;

// 慢速状态
static bb_slow_state_t bb_slow_state;
static int32_t         bb_slow_interval = 0;
static int32_t         bb_slow_timer    = 0;

// ==================== 私有函数声明 ====================

static void bb_write_intraframe(bb_main_state_t *state);
static void bb_write_interframe(bb_main_state_t *state);
static void bb_write_slow_frame(void);
static bool bb_should_log_i_frame(void);
static bool bb_should_log_p_frame(void);

// ==================== 公共函数实现 ====================

/**
 * @brief 注册存储设备
 * @param ops 存储操作函数集
 */
void bb_storage_register(bbStorageOps_t *ops)
{
    g_bb_storage.ops         = ops;
    g_bb_storage.initialized = false;
    g_bb_storage.logging     = false;
    g_bb_storage.log_number  = 0;
}

/**
 * @brief 初始化Blackbox
 */
void bb_init(void)
{
    bb_logging       = 0;
    bb_iteration     = 0;
    bb_p_frame_index = 0;
    bb_i_frame_index = 0;
    bb_start_time    = 0;
    bb_vbat_ref      = 0;
    bb_slow_interval = 256;
    bb_slow_timer    = 0;

    // 初始化历史状态
    memset(&bb_history, 0, sizeof(bb_history));
    memset(&bb_slow_state, 0, sizeof(bb_slow_state));

    // 注册FatFS存储设备
    bb_storage_register((bbStorageOps_t *)bb_fatfs_get_ops());

    // 初始化存储设备
    if (g_bb_storage.ops && g_bb_storage.ops->init)
    {
        g_bb_storage.ops->init();
    }
    g_bb_storage.initialized = true;
}

/**
 * @brief 打开日志
 */
void bb_open(void)
{
    if (bb_logging)
        return;

    // 打开存储设备
    if (g_bb_storage.ops && g_bb_storage.ops->open)
    {
        g_bb_storage.ops->open();
    }

    // 写入头信息
    bb_write_string("H Product:Blackbox flight data recorder\n");
    bb_write_string("H Data version:2\n");
    bb_write_string("H I interval:");
    bb_write_unsigned_vb(BB_I_INTERVAL);
    bb_write_string("\n");
    bb_write_string("H P interval:");
    bb_write_unsigned_vb(BB_P_INTERVAL);
    bb_write_string("\n");

    // 重置计数器和状态
    bb_iteration     = 0;
    bb_p_frame_index = 0;
    bb_i_frame_index = 0;
    bb_start_time    = 0;
    bb_slow_timer    = 0;

    // 清空历史
    memset(&bb_history, 0, sizeof(bb_history));

    bb_logging = 1;
}

/**
 * @brief 关闭日志
 */
void bb_close(void)
{
    if (!bb_logging)
        return;

    // 写入日志结束事件
    bb_write(0xE); // 事件标识
    bb_write_unsigned_vb(BB_EVENT_LOG_END);

    // 关闭存储设备
    if (g_bb_storage.ops && g_bb_storage.ops->close)
    {
        g_bb_storage.ops->close();
    }

    bb_logging = 0;
}

/**
 * @brief 更新Blackbox（每循环调用）
 * @param state 主状态数据
 */
void bb_update(bb_main_state_t *state)
{
    if (!bb_logging || !state)
        return;

    // 记录开始时间
    if (bb_start_time == 0)
    {
        bb_start_time = state->time;
        bb_vbat_ref   = state->vbatLatest;
    }

    // 推进计时器
    bb_iteration++;

    // 写入I帧
    if (bb_should_log_i_frame())
    {
        bb_write_intraframe(state);
        bb_i_frame_index = 0;
    }
    // 写入P帧
    else if (bb_should_log_p_frame())
    {
        bb_write_interframe(state);
        bb_p_frame_index = 0;
    }

    // 写入慢速帧
    bb_slow_timer++;
    if (bb_slow_timer >= bb_slow_interval)
    {
        bb_write_slow_frame();
        bb_slow_timer = 0;
    }

    // 更新历史状态
    bb_history_index = (bb_history_index + 1) % 3;
    memcpy(&bb_history[bb_history_index], state, sizeof(bb_main_state_t));
}

/**
 * @brief 写入事件
 * @param event 事件类型
 * @param data 事件数据
 */
void bb_log_event(bb_event_t event, uint32_t data)
{
    if (!bb_logging)
        return;

    bb_write(0xE); // 事件标识
    bb_write_unsigned_vb(event);
    bb_write_unsigned_vb(data);
}

/**
 * @brief 检查Blackbox是否已打开
 */
bool bb_is_logging(void)
{
    return bb_logging != 0;
}

/**
 * @brief 设置电压参考值（用于I帧编码）
 */
void bb_set_vbat_ref(uint16_t vbat)
{
    bb_vbat_ref = vbat;
}

/**
 * @brief 获取I帧间隔
 */
int bb_get_i_interval(void)
{
    return I_INTERVAL;
}

/**
 * @brief 获取P帧间隔
 */
int bb_get_p_interval(void)
{
    return P_INTERVAL;
}

// ==================== 私有函数实现 ====================

/**
 * @brief 检查是否应该记录I帧
 */
static bool bb_should_log_i_frame(void)
{
    bb_i_frame_index++;
    return bb_i_frame_index >= I_INTERVAL;
}

/**
 * @brief 检查是否应该记录P帧
 */
static bool bb_should_log_p_frame(void)
{
    bb_p_frame_index++;
    return bb_p_frame_index >= P_INTERVAL;
}

/**
 * @brief 写入I帧（完整数据帧）
 */
static void bb_write_intraframe(bb_main_state_t *state)
{
    bb_write('I'); // I帧标识

    // 迭代计数
    bb_write_unsigned_vb(bb_iteration);

    // 时间
    bb_write_unsigned_vb(state->time);

    // PID P项
    bb_write_signed_vb_array(state->axisPID_P, 3);

    // PID I项
    bb_write_signed_vb_array(state->axisPID_I, 3);

    // PID D项
    bb_write_signed_vb_array(state->axisPID_D, 3);

    // PID F项
    bb_write_signed_vb_array(state->axisPID_F, 3);

    // RC指令 (rpy + throttle)
    bb_write_signed16_vb_array(state->rcCommand, 3);
    bb_write_unsigned_vb(state->rcCommand[3]);

    // 设定点
    bb_write_signed16_vb_array(state->setpoint, 4);

    // 电压 (相对于参考值)
    bb_write_unsigned_vb((bb_vbat_ref - state->vbatLatest) & 0x3FFF);

    // 电流
    bb_write_signed_vb(state->amperageLatest);

    // 陀螺仪
    bb_write_signed16_vb_array(state->gyroADC, 3);

    // 加速度计
    bb_write_signed16_vb_array(state->accSmooth, 3);

    // 电机
    bb_write_unsigned_vb(state->motor[0] - MIN_THROTTLE);
    for (int i = 1; i < MOTOR_COUNT; i++)
    {
        bb_write_signed_vb(state->motor[i] - state->motor[0]);
    }

    // 气压高度
    bb_write_signed_vb(state->baroAlt);

    // RSSI
    bb_write_unsigned_vb(state->rssi);
}

/**
 * @brief 写入P帧（差分帧）
 */
static void bb_write_interframe(bb_main_state_t *state)
{
    // 获取历史状态
    int              prev_idx  = (bb_history_index + 2) % 3;
    int              prev2_idx = (bb_history_index + 1) % 3;
    bb_main_state_t *last      = &bb_history[prev_idx];
    bb_main_state_t *last2     = &bb_history[prev2_idx];

    bb_write('P'); // P帧标识

    // 时间二阶差分
    int32_t time_delta = (int32_t)(state->time - 2 * last->time + last2->time);
    bb_write_signed_vb(time_delta);

    int32_t deltas[8];

    // PID P项差分
    for (int i = 0; i < 3; i++)
    {
        deltas[i] = state->axisPID_P[i] - last->axisPID_P[i];
    }
    bb_write_signed_vb_array(deltas, 3);

    // PID I项差分（使用TAG2_3S32压缩）
    for (int i = 0; i < 3; i++)
    {
        deltas[i] = state->axisPID_I[i] - last->axisPID_I[i];
    }
    bb_write_tag2_3s32(deltas);

    // PID D项差分
    for (int i = 0; i < 3; i++)
    {
        deltas[i] = state->axisPID_D[i] - last->axisPID_D[i];
    }
    bb_write_signed_vb_array(deltas, 3);

    // PID F项差分
    for (int i = 0; i < 3; i++)
    {
        deltas[i] = state->axisPID_F[i] - last->axisPID_F[i];
    }
    bb_write_signed_vb_array(deltas, 3);

    // RC指令差分
    for (int i = 0; i < 4; i++)
    {
        deltas[i] = state->rcCommand[i] - last->rcCommand[i];
    }
    bb_write_tag8_4s16(deltas);

    // 设定点差分
    for (int i = 0; i < 4; i++)
    {
        deltas[i] = state->setpoint[i] - last->setpoint[i];
    }
    bb_write_tag8_4s16(deltas);

    // 电压差分
    int32_t vbat_delta = (int32_t)state->vbatLatest - last->vbatLatest;

    // 电流差分
    int32_t current_delta = state->amperageLatest - last->amperageLatest;

    // 合并为可选字段
    int     optional_count = 0;
    int32_t optional[8];

    optional[optional_count++] = vbat_delta;
    optional[optional_count++] = current_delta;

    bb_write_tag8_8svb(optional, optional_count);

    // 陀螺仪差分（使用AVERAGE_2预测器）
    for (int i = 0; i < 3; i++)
    {
        deltas[i] = state->gyroADC[i] - (last->gyroADC[i] + last2->gyroADC[i]) / 2;
    }
    bb_write_signed_vb_array(deltas, 3);

    // 加速度计差分
    for (int i = 0; i < 3; i++)
    {
        deltas[i] = state->accSmooth[i] - (last->accSmooth[i] + last2->accSmooth[i]) / 2;
    }
    bb_write_signed_vb_array(deltas, 3);

    // 电机差分
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        deltas[i] = state->motor[i] - (last->motor[i] + last2->motor[i]) / 2;
    }
    bb_write_tag8_4s16(deltas);
}

/**
 * @brief 写入慢速帧
 */
static void bb_write_slow_frame(void)
{
    // 慢速帧标识
    bb_write('S');

    // 飞行模式标志
    bb_write_unsigned_vb(bb_slow_state.flightModeFlags);

    // 状态标志
    bb_write_unsigned_vb(bb_slow_state.stateFlags);

    // 失控保护阶段
    bb_write_unsigned_vb(bb_slow_state.failsafePhase);

    // 接收信号
    bb_write_unsigned_vb(bb_slow_state.rxSignalReceived);

    // RC通道有效
    bb_write_unsigned_vb(bb_slow_state.rxFlightChannelsValid);
}

/* ==================== IO层接口实现 ==================== */

/**
 * @brief 写入单字节
 */
void bb_write(uint8_t value)
{
    if (!g_bb_storage.logging)
        return;
    uint8_t byte = value;
    if (g_bb_storage.ops && g_bb_storage.ops->write)
    {
        g_bb_storage.ops->write(&byte, 1);
    }
}

/**
 * @brief 写入字符串
 */
void bb_write_string(const char *s)
{
    if (!g_bb_storage.logging || !s)
        return;
    if (g_bb_storage.ops && g_bb_storage.ops->write)
    {
        g_bb_storage.ops->write((const uint8_t *)s, strlen(s));
    }
}

/**
 * @brief 刷新缓冲区
 */
void bb_flush(void)
{
    if (g_bb_storage.ops && g_bb_storage.ops->flush)
    {
        g_bb_storage.ops->flush();
    }
}

/**
 * @brief 检查Blackbox是否已打开
 */
bool bb_is_open(void)
{
    if (g_bb_storage.ops && g_bb_storage.ops->is_open)
    {
        return g_bb_storage.ops->is_open();
    }
    return false;
}

/**
 * @brief 获取日志编号
 */
int32_t bb_get_log_number(void)
{
    if (g_bb_storage.ops && g_bb_storage.ops->get_log_number)
    {
        return g_bb_storage.ops->get_log_number();
    }
    return 0;
}
