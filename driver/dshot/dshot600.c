/**
 **************************************************************************
 * @file     dshot600.c
 * @brief    AT32F435/437 DShot600 测试驱动源文件
 **************************************************************************
 * Copyright (c) 2025, Artery Technology
 **************************************************************************
 */
#include "dshot600.h"
#include "at32f435_437.h"
#include "at32f435_437_crm.h"
#include "at32f435_437_wk_config.h"
#include <rtdbg.h>
#include <rtthread.h>

/* Private variables ---------------------------------------------------------*/
// DMA缓冲区（4通道独立缓冲区）
static uint16_t dshot_dma_buffer[DSHOT_CHANNEL_MAX][DSHOT_DMA_BUFFER_SIZE] = {0};
// 渐变油门测试变量
static uint16_t test_throttle = DSHOT_THROTTLE_MIN;
static uint8_t  throttle_step = 1;
// 通道状态变量
static uint16_t channel_throttle[DSHOT_CHANNEL_MAX]  = {DSHOT_THROTTLE_MIN};
static uint8_t  channel_telemetry[DSHOT_CHANNEL_MAX] = {0};

/* Private functions prototypes ---------------------------------------------*/
// 计算DShot数据包校验和
static uint8_t dshot600_calc_checksum(uint16_t packet)
{
    uint8_t  checksum = 0;
    uint16_t temp     = packet;
    for (uint8_t i = 0; i < 3; i++)
    {
        checksum ^= temp & 0x0F;
        temp >>= 4;
    }
    return checksum & 0x0F;
}

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  DShot600初始化（定时器+DMA）
 * @param  无
 * @retval 无
 */
void dshot600_init(void)
{
    // 调试：打印实际时钟频率
    crm_clocks_freq_type clocks;
    crm_clocks_freq_get(&clocks);
    LOG_I("DShot600 Init - APB1 freq: %d, APB2 freq: %d", clocks.apb1_freq, clocks.apb2_freq);
    LOG_I("DShot600 Config - Timer CLK: %d, ARR: %d, PSC: %d",
          DSHOT600_TIMER_CLK, DSHOT600_ARR, DSHOT600_PSC);

    // 初始化DMA缓冲区
    for (uint8_t ch = 0; ch < DSHOT_CHANNEL_MAX; ch++)
    {
        for (uint8_t i = 0; i < DSHOT_DMA_BUFFER_SIZE; i++)
        {
            dshot_dma_buffer[ch][i] = DSHOT600_BIT_0_CCR; // 默认填充逻辑0
        }
    }

    // 1. 确保TMR3时钟使能（APB1）
    crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);

    // 2. 确保TMR4时钟使能（APB1）
    crm_periph_clock_enable(CRM_TMR4_PERIPH_CLOCK, TRUE);

    // 3. 确保DMA1时钟使能
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);

    // 5. 确保GPIOB时钟使能
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

    rt_thread_mdelay(100);

    // 配置并启用DMA通道1（TMR3_CH1）
    wk_dma1_channel1_init();
    wk_dma1_channel2_init();
    wk_dma1_channel3_init();
    wk_dma1_channel4_init();

    rt_thread_mdelay(100);

    wk_dma_channel_config(DSHOT_DMA_CHANNEL_1,
                          (uint32_t)DSHOT_TMR3_CH1_CCR,
                          (uint32_t)dshot_dma_buffer[0],
                          DSHOT_DMA_BUFFER_SIZE);
    dma_channel_enable(DSHOT_DMA_CHANNEL_1, TRUE);

    wk_dma_channel_config(DSHOT_DMA_CHANNEL_2,
                          (uint32_t)DSHOT_TMR3_CH2_CCR,
                          (uint32_t)dshot_dma_buffer[1],
                          DSHOT_DMA_BUFFER_SIZE);
    dma_channel_enable(DSHOT_DMA_CHANNEL_2, TRUE);

    wk_dma_channel_config(DSHOT_DMA_CHANNEL_3,
                          (uint32_t)DSHOT_TMR4_CH1_CCR,
                          (uint32_t)dshot_dma_buffer[2],
                          DSHOT_DMA_BUFFER_SIZE);
    dma_channel_enable(DSHOT_DMA_CHANNEL_3, TRUE);

    wk_dma_channel_config(DSHOT_DMA_CHANNEL_4,
                          (uint32_t)DSHOT_TMR4_CH2_CCR,
                          (uint32_t)dshot_dma_buffer[3],
                          DSHOT_DMA_BUFFER_SIZE);
    dma_channel_enable(DSHOT_DMA_CHANNEL_4, TRUE);

    wk_tmr3_init();
    wk_tmr4_init();
}

/**
 * @brief  构建DShot数据包（含校验和）
 * @param  value: 油门值(48-2047)或命令值(0-47)
 * @param  telemetry: 是否请求遥测
 * @retval 完整数据包（16bit）
 */
uint16_t dshot600_compose_packet(uint16_t value, uint8_t telemetry)
{
    uint16_t packet_value;

    // 检查是命令还是油门值
    if (value <= DSHOT_THROTTLE_MAX_CMD)
    {
        // 命令模式：值0-47直接使用
        packet_value = value;
    }
    else
    {
        // 油门模式：限制在有效范围内
        if (value < DSHOT_THROTTLE_MIN)
            packet_value = DSHOT_THROTTLE_MIN;
        else if (value > DSHOT_THROTTLE_MAX)
            packet_value = DSHOT_THROTTLE_MAX;
        else
            packet_value = value;
    }

    // 构建基础数据包（11bit值 + 1bit遥测）
    uint16_t packet = (packet_value << 1) | (telemetry ? 1 : 0);
    // 添加4bit校验和
    uint8_t checksum = dshot600_calc_checksum(packet);
    packet           = (packet << 4) | checksum;

    return packet;
}

/**
 * @brief  填充DMA缓冲区（单通道）
 * @param  dma_buf: DMA缓冲区指针
 * @param  packet: DShot数据包
 * @retval 无
 */
void dshot600_fill_dma_buffer(uint16_t *dma_buf, uint16_t packet)
{
    // 按位填充（MSB先行）
    for (uint8_t i = 0; i < DSHOT_PACKET_LENGTH; i++)
    {
        if (packet & (0x8000 >> i))
        {
            dma_buf[i] = DSHOT600_BIT_1_CCR; // 逻辑1
        }
        else
        {
            dma_buf[i] = DSHOT600_BIT_0_CCR; // 逻辑0
        }
    }
    // dma_buf[0] = DSHOT600_BIT_0_CCR; // 逻辑0
    // dma_buf[1] = DSHOT600_BIT_0_CCR; // 逻辑0
    // dma_buf[2] = DSHOT600_BIT_0_CCR; // 逻辑0
    // dma_buf[3] = DSHOT600_BIT_0_CCR; // 逻辑0
    // dma_buf[4] = DSHOT600_BIT_0_CCR; // 逻辑0
    // dma_buf[5] = DSHOT600_BIT_0_CCR; // 逻辑1
    // dma_buf[6] = DSHOT600_BIT_0_CCR; // 逻辑1
    // dma_buf[7] = DSHOT600_BIT_0_CCR; // 逻辑1

    // dma_buf[8]  = DSHOT600_BIT_1_CCR; // 逻辑1
    // dma_buf[9]  = DSHOT600_BIT_1_CCR; // 逻辑1
    // dma_buf[10] = DSHOT600_BIT_1_CCR; // 逻辑1
    // dma_buf[11] = DSHOT600_BIT_1_CCR; // 逻辑1
    // dma_buf[12] = DSHOT600_BIT_1_CCR; // 逻辑1
    // dma_buf[13] = DSHOT600_BIT_1_CCR; // 逻辑1
    // dma_buf[14] = DSHOT600_BIT_1_CCR; // 逻辑1
    // dma_buf[15] = DSHOT600_BIT_1_CCR; // 逻辑1
}

/**
 * @brief  发送DShot数据包（启动DMA传输）
 * @param  channel: 通道号
 * @param  value: 油门值(48-2047)或命令值(0-47)
 * @retval 无
 */
void dshot600_send_packet(dshot_channel_e channel, uint16_t value)
{
    if (channel >= DSHOT_CHANNEL_MAX)
        return;

    // 更新通道状态
    channel_throttle[channel] = value;

    // 1. 构建数据包
    uint16_t packet = dshot600_compose_packet(value, channel_telemetry[channel]);
    // 2. 填充DMA缓冲区
    dshot600_fill_dma_buffer(dshot_dma_buffer[channel], packet);

    // 3. 根据通道选择对应的DMA和定时器寄存器
    switch (channel)
    {
    case DSHOT_CHANNEL_1:
        dma_channel_enable(DMA1_CHANNEL1, FALSE);
        wk_dma_channel_config(DSHOT_DMA_CHANNEL_1,
                              (uint32_t)DSHOT_TMR3_CH1_CCR,
                              (uint32_t)dshot_dma_buffer[channel],
                              DSHOT_DMA_BUFFER_SIZE);
        dma_channel_enable(DMA1_CHANNEL1, TRUE);
        break;

    case DSHOT_CHANNEL_2:
        dma_channel_enable(DMA1_CHANNEL2, FALSE);
        wk_dma_channel_config(DSHOT_DMA_CHANNEL_2,
                              (uint32_t)DSHOT_TMR3_CH2_CCR,
                              (uint32_t)dshot_dma_buffer[channel],
                              DSHOT_DMA_BUFFER_SIZE);
        dma_channel_enable(DMA1_CHANNEL2, TRUE);
        break;

    case DSHOT_CHANNEL_3:
        dma_channel_enable(DMA1_CHANNEL3, FALSE);
        wk_dma_channel_config(DSHOT_DMA_CHANNEL_3,
                              (uint32_t)DSHOT_TMR4_CH1_CCR,
                              (uint32_t)dshot_dma_buffer[channel],
                              DSHOT_DMA_BUFFER_SIZE);
        dma_channel_enable(DMA1_CHANNEL3, TRUE);
        break;

    case DSHOT_CHANNEL_4:
        dma_channel_enable(DMA1_CHANNEL4, FALSE);
        wk_dma_channel_config(DSHOT_DMA_CHANNEL_4,
                              (uint32_t)DSHOT_TMR4_CH2_CCR,
                              (uint32_t)dshot_dma_buffer[channel],
                              DSHOT_DMA_BUFFER_SIZE);
        dma_channel_enable(DMA1_CHANNEL4, TRUE);
        break;

    default:
        break;
    }
}

/**
 * @brief  测试函数：发送固定油门值
 * @param  channel: 通道号
 * @param  throttle: 固定油门值
 * @retval 无
 */
void dshot600_test_send_fixed_throttle(dshot_channel_e channel, uint16_t throttle)
{
    while (1)
    {
        dshot600_send_packet(channel, throttle);
        rt_thread_mdelay(10); // 10ms发送一次（可调整频率）
    }
}

/**
 * @brief  发送DShot命令
 * @param  channel: 通道号
 * @param  command: 命令值(0-47)
 * @retval 无
 */
void dshot600_send_command(dshot_channel_e channel, uint8_t command)
{
    if (command > DSHOT_THROTTLE_MAX_CMD)
        command = DSHOT_THROTTLE_MAX_CMD;

    dshot600_send_packet(channel, command);
}

/**
 * @brief  发送油门值（48-2047）
 * @param  channel: 通道号
 * @param  throttle: 油门值
 * @retval 无
 */
void dshot600_send_throttle(dshot_channel_e channel, uint16_t throttle)
{
    // 限制在动力输出范围内
    if (throttle < DSHOT_THROTTLE_MIN_POWER)
        throttle = DSHOT_THROTTLE_MIN_POWER;
    if (throttle > DSHOT_THROTTLE_MAX_POWER)
        throttle = DSHOT_THROTTLE_MAX_POWER;

    dshot600_send_packet(channel, throttle);
}

/**
 * @brief  停止电机（发送0命令）
 * @param  channel: 通道号
 * @retval 无
 */
void dshot600_motor_stop(dshot_channel_e channel)
{
    dshot600_send_command(channel, DSHOT_CMD_MOTOR_STOP);
}

/**
 * @brief  电调鸣叫测试
 * @param  channel: 通道号
 * @param  beep_type: 鸣叫类型(1-5)
 * @retval 无
 */
void dshot600_beep_test(dshot_channel_e channel, uint8_t beep_type)
{
    if (beep_type < 1 || beep_type > 5)
        return;

    dshot600_send_command(channel, DSHOT_CMD_BEEP_1 - 1 + beep_type);
}

/**
 * @brief  设置旋转方向
 * @param  channel: 通道号
 * @param  direction: 方向(7或8，或20或21)
 * @retval 无
 */
void dshot600_set_rotation_direction(dshot_channel_e channel, uint8_t direction)
{
    if (direction == 1)
        dshot600_send_command(channel, DSHOT_CMD_ROTATION_DIR_1);
    else if (direction == 2)
        dshot600_send_command(channel, DSHOT_CMD_ROTATION_DIR_2);
    else if (direction == 3)
        dshot600_send_command(channel, DSHOT_CMD_ROTATION_DIR_1_2);
    else if (direction == 4)
        dshot600_send_command(channel, DSHOT_CMD_ROTATION_DIR_2_2);
}

/**
 * @brief  LED控制
 * @param  channel: 通道号
 * @param  led_num: LED编号(1-3)
 * @param  on_off: 开关状态
 * @retval 无
 */
void dshot600_led_control(dshot_channel_e channel, uint8_t led_num, uint8_t on_off)
{
    if (led_num < 1 || led_num > 3)
        return;

    uint8_t cmd_base = on_off ? DSHOT_CMD_LED_1_ON : DSHOT_CMD_LED_1_OFF;
    dshot600_send_command(channel, cmd_base + (led_num - 1) * 2);
}

/**
 * @brief  测试函数：渐变油门值（0→最大→0循环）
 * @param  channel: 通道号
 * @retval 无
 */
void dshot600_test_gradient_throttle(dshot_channel_e channel)
{
    // 更新油门值
    test_throttle += throttle_step;
    if (test_throttle >= DSHOT_THROTTLE_MAX || test_throttle <= DSHOT_THROTTLE_MIN)
    {
        throttle_step = -throttle_step; // 反向
    }
    // 发送数据包
    dshot600_send_packet(channel, test_throttle);
    // rt_thread_mdelay(5); // 渐变速度控制
}

/**
 * @brief  测试函数：多通道同步测试
 * @retval 无
 */
void dshot600_test_multi_channel(void)
{
    static uint8_t test_phase = 0;

    switch (test_phase)
    {
    case 0:
        // 所有通道停止
        for (uint8_t ch = 0; ch < DSHOT_CHANNEL_MAX; ch++)
        {
            dshot600_motor_stop((dshot_channel_e)ch);
        }
        break;

    case 1:
        // 通道1-4依次鸣叫
        dshot600_beep_test(DSHOT_CHANNEL_1, 1);
        break;

    case 2:
        dshot600_beep_test(DSHOT_CHANNEL_2, 2);
        break;

    case 3:
        dshot600_beep_test(DSHOT_CHANNEL_3, 3);
        break;

    case 4:
        dshot600_beep_test(DSHOT_CHANNEL_4, 4);
        break;

    case 5:
        // 所有通道同步油门测试
        for (uint8_t ch = 0; ch < DSHOT_CHANNEL_MAX; ch++)
        {
            dshot600_send_throttle((dshot_channel_e)ch, 1000 + ch * 100);
        }
        break;
    }

    test_phase = (test_phase + 1) % 6;
}

/**
 * @brief  测试函数：命令发送测试
 * @param  channel: 通道号
 * @retval 无
 */
void dshot600_test_commands(dshot_channel_e channel)
{
    static uint8_t  cmd_index = 0;
    static uint32_t last_time = 0;

    switch (cmd_index)
    {
    case 0:
        dshot600_beep_test(channel, 1);
        break;
    case 1:
        dshot600_beep_test(channel, 2);
        break;
    case 2:
        dshot600_led_control(channel, 1, 1);
        break;
    case 3:
        dshot600_led_control(channel, 1, 0);
        break;
    case 4:
        dshot600_set_rotation_direction(channel, 1);
        break;
    case 5:
        dshot600_set_rotation_direction(channel, 2);
        break;
    default:
        dshot600_motor_stop(channel);
        break;
    }

    cmd_index = (cmd_index + 1) % 6;
}