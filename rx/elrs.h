#ifndef _ELRS_H_
#define _ELRS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include "rtdevice.h"
#include "rtthread.h"

/************************ 串口相关定义（使用 RT-Thread 设备） ************************/
// 使用的 RT-Thread 串口设备名称（默认串口1）
#ifndef ELRS_UART_DEVICE_NAME
#define ELRS_UART_DEVICE_NAME "uart1"
#endif

// CRSF 协议波特率
#define ELRS_UART_BAUDRATE 420000

/************************ CRSF协议定义 ************************/
// CRSF帧头
#define CRSF_SYNC_BYTE 0xC8
// CRSF通道数据帧长度（含头+类型+数据+校验）
#define CRSF_CHANNEL_FRAME_LENGTH 26
// CRSF通道数据帧类型
#define CRSF_FRAME_TYPE_CHANNELS 0x16
// CRSF通道数（ELRS标准8通道）
#define ELRS_CHANNEL_COUNT 16
// CRSF通道值范围（172~1811对应PWM 988~2012）
#define CRSF_CHANNEL_MIN 172
#define CRSF_CHANNEL_MAX 1811
#define CRSF_CHANNEL_MID ((CRSF_CHANNEL_MAX + CRSF_CHANNEL_MIN) / 2)
// 标准化后通道值范围（-1000 ~ 1000）
#define ELRS_CHANNEL_SCALE_MIN -1000
#define ELRS_CHANNEL_SCALE_MAX 1000

/************************ 数据结构定义 ************************/
typedef struct elrsDev_s elrsDev_t;

/**
 * @brief ELRS接收机设备结构体
 * 包含串口配置、接收缓冲区、通道数据等核心信息
 */
struct elrsDev_s
{
    /* 串口相关 */
    rt_device_t uart_dev; // RT-Thread 串口设备

    /* CRSF 断帧与解析状态（线程中维护） */
    uint8_t crsf_frame[CRSF_CHANNEL_FRAME_LENGTH]; // 当前正在接收的一帧数据缓存
    uint8_t crsf_pos;                              // 已接收的字节数（索引）
    uint8_t crsf_expected_len;                     // 来自 length 字段的期望长度（类型+payload+校验）
    bool    crsf_in_frame;                         // 是否已经收到同步字节，正在接收一帧

    /* 通道数据（原始 + 标准化） */
    uint16_t raw_channels[ELRS_CHANNEL_COUNT];    // 原始通道值，协议定义：172 ~ 1811
    int16_t  scaled_channels[ELRS_CHANNEL_COUNT]; // 标准化通道值：-1000 ~ 1000，0 为中位

    /* 典型 RC 语义通道，便于业务直接使用 */
    int16_t ch1_roll;     // 通道1：横滚（Roll），范围：-1000 ~ 1000
    int16_t ch2_pitch;    // 通道2：俯仰（Pitch），范围：-1000 ~ 1000
    int16_t ch3_throttle; // 通道3：油门（Throttle），范围：-1000 ~ 1000（0 = 最低油门）
    int16_t ch4_yaw;      // 通道4：偏航（Yaw），范围：-1000 ~ 1000
    int16_t ch5_arm;      // 通道5：Arming，范围：-1000 ~ 1000
    int16_t ch6_aux2;     // 通道6：AUX2，范围：-1000 ~ 1000
    int16_t ch7_mode;     // 通道7：Mode，范围：-1000 ~ 1000
    int16_t ch8_aux4;     // 通道8：AUX4，范围：-1000 ~ 1000

    uint32_t last_update_time; // 最后一次成功解析的时间（ms）
    bool     is_connected;     // 是否在有效连接中（500ms 内有更新）

    /* 函数指针 */
    bool (*init)(struct elrsDev_s *);          // 初始化函数
    bool (*read_channels)(struct elrsDev_s *); // 读取通道数据
};

extern elrsDev_t g_elrs_receiver; // ELRS接收机实例

bool rx_init(void);

#endif