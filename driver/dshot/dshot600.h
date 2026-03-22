/**
 **************************************************************************
 * @file     dshot600.h
 * @brief    AT32F435/437 DShot600 测试驱动头文件
 **************************************************************************
 * Copyright (c) 2025, Artery Technology
 **************************************************************************
 */

#ifndef __DSHOT600_H
#define __DSHOT600_H

/* Includes ------------------------------------------------------------------*/
#include "at32f435_437.h"
#include "stdbool.h"
/* Exported defines ----------------------------------------------------------*/
// DShot600 核心参数（适配144MHz主频，RT-Thread配置）
#define DSHOT600_TIMER     TMR3       // 使用TIM3作为DShot600定时器
#define DSHOT600_TIMER_CLK 144000000U // TIM3计数时钟=144MHz（RT-Thread实际配置）
#define DSHOT600_BIT_FREQ  600000U    // DShot600单bit频率=600kHz
#define DSHOT600_ARR       239U       // 自动重装值=239 (144MHz/240=600kHz)
#define DSHOT600_PSC       1U         // 预分频值=1

// DShot600 脉宽定义（基于ARR=239）
#define DSHOT600_BIT_1_CCR 160U // 逻辑1脉宽=75% (240*0.75)
#define DSHOT600_BIT_0_CCR 80U  // 逻辑0脉宽=37.5% (240*0.375)

// DShot数据包参数
#define DSHOT_PACKET_LENGTH 16U   // DShot数据包长度=16bit
#define DSHOT_THROTTLE_MIN  48U   // 最小油门值
#define DSHOT_THROTTLE_MAX  2047U // 最大油门值

// DMA配置（多通道支持）
#define DSHOT_DMA_CHANNEL_1           DMA1_CHANNEL3
#define DSHOT_DMA_CHANNEL_2           DMA1_CHANNEL4
#define DSHOT_DMA_CHANNEL_3           DMA1_CHANNEL5
#define DSHOT_DMA_CHANNEL_4           DMA1_CHANNEL6
#define DSHOT_DMA_BUFFER_SIZE         (DSHOT_PACKET_LENGTH) // DMA缓冲区长度=16 (16位数据)
#define DSHOT_BITBANG_DMA_BUFFER_SIZE (DSHOT_PACKET_LENGTH + 1)

// 定时器通道配置
#define DSHOT_TMR3_CH1_CCR (&TMR3->c1dt)
#define DSHOT_TMR3_CH2_CCR (&TMR3->c2dt)
#define DSHOT_TMR4_CH1_CCR (&TMR4->c1dt)
#define DSHOT_TMR4_CH2_CCR (&TMR4->c2dt)

// DShot命令定义
#define DSHOT_CMD_MOTOR_STOP       0  // 电机停转（电调解锁）
#define DSHOT_CMD_BEEP_1           1  // 电调鸣叫1（最低频）
#define DSHOT_CMD_BEEP_2           2  // 电调鸣叫2
#define DSHOT_CMD_BEEP_3           3  // 电调鸣叫3
#define DSHOT_CMD_BEEP_4           4  // 电调鸣叫4
#define DSHOT_CMD_BEEP_5           5  // 电调鸣叫5（最高频）
#define DSHOT_CMD_ESC_VERSION      6  // ESC版本信息/序列号
#define DSHOT_CMD_ROTATION_DIR_1   7  // 旋转方向1
#define DSHOT_CMD_ROTATION_DIR_2   8  // 旋转方向2
#define DSHOT_CMD_3D_MODE_OFF      9  // 3D模式关闭
#define DSHOT_CMD_3D_MODE_ON       10 // 3D模式开启
#define DSHOT_CMD_GET_ESC_CONFIG   11 // 获取ESC配置
#define DSHOT_CMD_SAVE_ESC_CONFIG  12 // 保存ESC配置
#define DSHOT_CMD_TELEMETRY_ON     13 // telemetry扩展信息开启
#define DSHOT_CMD_TELEMETRY_OFF    14 // telemetry扩展信息关闭
#define DSHOT_CMD_ROTATION_DIR_2_2 20 // 旋转方向2（备选）
#define DSHOT_CMD_ROTATION_DIR_1_2 21 // 旋转方向1（备选）
#define DSHOT_CMD_LED_1_ON         22 // LED1亮
#define DSHOT_CMD_LED_1_OFF        23 // LED1灭
#define DSHOT_CMD_LED_2_ON         24 // LED2亮
#define DSHOT_CMD_LED_2_OFF        25 // LED2灭
#define DSHOT_CMD_LED_3_ON         26 // LED3亮
#define DSHOT_CMD_LED_3_OFF        27 // LED3灭
#define DSHOT_CMD_AUDIO_STREAM_ON  30 // 音频流传输开启（KISS电调）
#define DSHOT_CMD_AUDIO_STREAM_OFF 31 // 音频流传输关闭（KISS电调）
#define DSHOT_CMD_SILENT_MODE_ON   32 // 静音模式开启（KISS电调）
#define DSHOT_CMD_SILENT_MODE_OFF  33 // 静音模式关闭（KISS电调）

// 油门值范围
#define DSHOT_THROTTLE_ZERO      48   // 零油门（对应命令模式）
#define DSHOT_THROTTLE_MAX_CMD   47   // 命令模式最大值
#define DSHOT_THROTTLE_MIN_POWER 48   // 最小动力输出
#define DSHOT_THROTTLE_MAX_POWER 2047 // 最大动力输出

// DMA事件定义
#define DSHOT1_DMA_FDT_EVENT (1 << 0)
#define DSHOT2_DMA_FDT_EVENT (1 << 1)
#define DSHOT3_DMA_FDT_EVENT (1 << 2)
#define DSHOT4_DMA_FDT_EVENT (1 << 3)

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    DSHOT_CHANNEL_1 = 0,
    DSHOT_CHANNEL_2,
    DSHOT_CHANNEL_3,
    DSHOT_CHANNEL_4,
    DSHOT_CHANNEL_MAX
} dshot_channel_e;

/* Exported functions prototypes ---------------------------------------------*/
// DShot600初始化（定时器+DMA）
bool dshot600_init(void);

// 构建DShot数据包（含校验和）
uint16_t dshot600_compose_packet(uint16_t value, uint8_t telemetry);

// 填充DMA缓冲区（单通道）
void dshot600_fill_dma_buffer(uint16_t *dma_buf, uint16_t packet);

// 发送DShot数据包（启动DMA传输）
void dshot600_send_packet(dshot_channel_e channel, uint16_t value);

// 发送DShot命令
void dshot600_send_command(dshot_channel_e channel, uint8_t command);

// 发送油门值（48-2047）
void dshot600_send_throttle(dshot_channel_e channel, uint16_t throttle);

// 停止电机（发送0命令）
void dshot600_motor_stop(dshot_channel_e channel);

// 电调鸣叫测试
void dshot600_beep_test(dshot_channel_e channel, uint8_t beep_type);

// 设置旋转方向
void dshot600_set_rotation_direction(dshot_channel_e channel, uint8_t direction);

// LED控制
void dshot600_led_control(dshot_channel_e channel, uint8_t led_num, uint8_t on_off);

// 测试函数：发送固定油门值
void dshot600_test_send_fixed_throttle(dshot_channel_e channel, uint16_t throttle);

// 测试函数：渐变油门值（0→最大→0循环）
void dshot600_test_gradient_throttle(dshot_channel_e channel);

// 测试函数：多通道同步测试
void dshot600_test_multi_channel(void);

// 测试函数：命令发送测试
void dshot600_test_commands(dshot_channel_e channel);

#endif /* __DSHOT600_H */
