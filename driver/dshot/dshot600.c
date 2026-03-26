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
#include "at32f435_437_dma.h"
#include "at32f435_437_wk_config.h"
#include "dshot600.h"
#include <rtdbg.h>
#include <rtthread.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
// DMA缓冲区（4通道独立缓冲区）
static uint16_t dshot_dma_buffer[DSHOT_CHANNEL_MAX][DSHOT_BITBANG_DMA_BUFFER_SIZE] = {0};
// 通道状态变量
static uint16_t channel_throttle[DSHOT_CHANNEL_MAX]  = {DSHOT_THROTTLE_MIN};
static uint8_t  channel_telemetry[DSHOT_CHANNEL_MAX] = {0};

/* Private functions prototypes ---------------------------------------------*/
// 计算DShot数据包校验和（对12位数据计算4位校验和）
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
bool dshot600_init(void)
{

    // 初始化DMA缓冲区
    // 默认填充逻辑0（低电平），确保空闲状态正确
    for (uint8_t ch = 0; ch < DSHOT_CHANNEL_MAX; ch++)
    {
        for (uint8_t i = 0; i < DSHOT_DMA_BUFFER_SIZE; i++)
        {
            dshot_dma_buffer[ch][i] = DSHOT600_BIT_0_CCR; // 默认填充逻辑0（空闲状态）
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

    nvic_irq_enable(DMA1_Channel3_IRQn, 0, 0);
    nvic_irq_enable(DMA1_Channel4_IRQn, 0, 0);
    nvic_irq_enable(DMA1_Channel5_IRQn, 0, 0);
    nvic_irq_enable(DMA1_Channel6_IRQn, 0, 0);
    rt_thread_mdelay(100);

    // 配置并启用DMA通道1（TMR3_CH1）

    wk_dma1_channel3_init();
    wk_dma1_channel4_init();
    wk_dma1_channel5_init();
    wk_dma1_channel6_init();

    wk_dma_channel_config(DSHOT_DMA_CHANNEL_1,
                          (uint32_t)DSHOT_TMR3_CH1_CCR,
                          (uint32_t)dshot_dma_buffer[0],
                          DSHOT_BITBANG_DMA_BUFFER_SIZE);
    dma_channel_enable(DSHOT_DMA_CHANNEL_1, TRUE);

    wk_dma_channel_config(DSHOT_DMA_CHANNEL_2,
                          (uint32_t)DSHOT_TMR3_CH2_CCR,
                          (uint32_t)dshot_dma_buffer[1],
                          DSHOT_BITBANG_DMA_BUFFER_SIZE);
    dma_channel_enable(DSHOT_DMA_CHANNEL_2, TRUE);

    wk_dma_channel_config(DSHOT_DMA_CHANNEL_3,
                          (uint32_t)DSHOT_TMR4_CH1_CCR,
                          (uint32_t)dshot_dma_buffer[2],
                          DSHOT_BITBANG_DMA_BUFFER_SIZE);
    dma_channel_enable(DSHOT_DMA_CHANNEL_3, TRUE);

    wk_dma_channel_config(DSHOT_DMA_CHANNEL_4,
                          (uint32_t)DSHOT_TMR4_CH2_CCR,
                          (uint32_t)dshot_dma_buffer[3],
                          DSHOT_BITBANG_DMA_BUFFER_SIZE);
    dma_channel_enable(DSHOT_DMA_CHANNEL_4, TRUE);

    wk_tmr3_init();
    wk_tmr4_init();

    rt_thread_mdelay(5);

    return true;
}

/**
 * @brief  构建DShot数据包（含校验和）
 * @param  value: 油门值(48-2047)或命令值(0-47)
 * @param  telemetry: 是否请求遥测
 * @retval 完整数据包（16bit）
 */
static uint16_t dshot600_compose_packet(uint16_t value, uint8_t telemetry)
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

    packet = (packet << 4) | checksum;

    return packet;
}

/**
 * @brief  填充DMA缓冲区（单通道）
 * @param  dma_buf: DMA缓冲区指针
 * @param  packet: DShot数据包
 * @retval 无
 *
 * 根据betaflight实现，添加帧间隔位以避免CRC错误：
 * - 填充16位DShot数据
 * - 添加2位间隔位保持线路在空闲状态（低电平）
 * - 确保ESC能正确采样最后的bit
 */
static void dshot600_fill_dma_buffer(uint16_t *dma_buf, uint16_t packet)
{
    // 按位填充16位DShot数据（MSB先行）
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

    if (value > 1500)
        value = 1500;

    // 更新通道状态
    channel_throttle[channel] = value;

    // 1. 构建数据包
    uint16_t packet = dshot600_compose_packet(value, channel_telemetry[channel]);
    // 2. 填充DMA缓冲区
    dshot600_fill_dma_buffer(dshot_dma_buffer[channel], packet);
    // 3. 根据通道选择对应的DMA和定时器寄存器
    // 在Normal模式下，需要先禁用DMA通道，重新配置后再启用
    rt_uint32_t e;
    switch (channel)
    {
    case DSHOT_CHANNEL_1:
        dma_channel_enable(DSHOT_DMA_CHANNEL_1, FALSE); // 先禁用
        wk_dma_channel_config(DSHOT_DMA_CHANNEL_1,
                              (uint32_t)DSHOT_TMR3_CH1_CCR,
                              (uint32_t)dshot_dma_buffer[channel],
                              DSHOT_BITBANG_DMA_BUFFER_SIZE);
        dma_channel_enable(DSHOT_DMA_CHANNEL_1, TRUE); // 重新启用
        break;

    case DSHOT_CHANNEL_2:
        dma_channel_enable(DSHOT_DMA_CHANNEL_2, FALSE); // 先禁用
        wk_dma_channel_config(DSHOT_DMA_CHANNEL_2,
                              (uint32_t)DSHOT_TMR3_CH2_CCR,
                              (uint32_t)dshot_dma_buffer[channel],
                              DSHOT_BITBANG_DMA_BUFFER_SIZE);
        dma_channel_enable(DSHOT_DMA_CHANNEL_2, TRUE); // 重新启用
        break;

    case DSHOT_CHANNEL_3:
        dma_channel_enable(DSHOT_DMA_CHANNEL_3, FALSE); // 先禁用
        wk_dma_channel_config(DSHOT_DMA_CHANNEL_3,
                              (uint32_t)DSHOT_TMR4_CH1_CCR,
                              (uint32_t)dshot_dma_buffer[channel],
                              DSHOT_BITBANG_DMA_BUFFER_SIZE);
        dma_channel_enable(DSHOT_DMA_CHANNEL_3, TRUE); // 重新启用
        break;

    case DSHOT_CHANNEL_4:
        dma_channel_enable(DSHOT_DMA_CHANNEL_4, FALSE); // 先禁用
        wk_dma_channel_config(DSHOT_DMA_CHANNEL_4,
                              (uint32_t)DSHOT_TMR4_CH2_CCR,
                              (uint32_t)dshot_dma_buffer[channel],
                              DSHOT_BITBANG_DMA_BUFFER_SIZE);
        dma_channel_enable(DSHOT_DMA_CHANNEL_4, TRUE); // 重新启用
        break;

    default:
        break;
    }
}

/**
 * @brief  停止电机（发送0命令）
 * @param  channel: 通道号
 * @retval 无
 */
void dshot600_motor_stop(dshot_channel_e channel)
{
    dshot600_send_packet(channel, DSHOT_CMD_MOTOR_STOP);
}

/**
 * @brief  电调鸣叫测试
 * @param  channel: 通道号
 * @param  beep_type: 鸣叫类型(1-5)
 * @retval 无
 */
void dshot600_beeper(dshot_channel_e channel, uint8_t beep_type)
{
    if (beep_type < 1 || beep_type > 5)
        return;

    dshot600_send_packet(channel, DSHOT_CMD_BEEP_1 - 1 + beep_type);
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
        dshot600_send_packet(channel, DSHOT_CMD_ROTATION_DIR_1);
    else if (direction == 2)
        dshot600_send_packet(channel, DSHOT_CMD_ROTATION_DIR_2);
    else if (direction == 3)
        dshot600_send_packet(channel, DSHOT_CMD_ROTATION_DIR_1_2);
    else if (direction == 4)
        dshot600_send_packet(channel, DSHOT_CMD_ROTATION_DIR_2_2);
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
    dshot600_send_packet(channel, cmd_base + (led_num - 1) * 2);
}

/* 持续发送测试变量 */
uint8_t         g_dshot_run_enable   = 0;
uint16_t        g_dshot_run_throttle = 0;
dshot_channel_e g_dshot_run_channel  = DSHOT_CHANNEL_1;

static void dshot_channel_cmd(int argc, char **argv)
{
    if (argc < 2)
    {
        LOG_I("Usage: dshot_set <channel>\n");
        LOG_I("  channel: 0-4\n");
        return;
    }

    uint8_t channel = atoi(argv[1]);

    if (channel >= DSHOT_CHANNEL_MAX)
    {
        LOG_I("Error: channel must be 0-3\n");
        return;
    }

    g_dshot_run_channel = channel;
    LOG_I("Channel set to %d\n", channel);
}
MSH_CMD_EXPORT(dshot_channel_cmd, dshot_set<channel> - Set channel in real - time);

/**
 * @brief  dshot_set 命令 - 实时修改油门值
 * 用法: dshot_set <throttle>
 * 说明: 在 dshot_run 运行期间实时修改油门值
 */
static void dshot_throttle_cmd(int argc, char **argv)
{
    if (argc < 2)
    {
        LOG_I("Usage: dshot_set <throttle>\n");
        LOG_I("  throttle: 48-2047\n");
        return;
    }

    uint16_t throttle = atoi(argv[1]);

    if (throttle < DSHOT_THROTTLE_MIN_POWER || throttle > DSHOT_THROTTLE_MAX)
    {
        LOG_I("Error: throttle must be %d-%d\n", DSHOT_THROTTLE_MIN_POWER, DSHOT_THROTTLE_MAX);
        return;
    }

    g_dshot_run_throttle = throttle;
    LOG_I("Throttle set to %d\n", throttle);
}
MSH_CMD_EXPORT(dshot_throttle_cmd, dshot_set<throttle> - Set throttle in real - time);

/**
 * @brief  dshot_arm 命令 - 解锁电调，同时也是锁定电机的命令
 */
static void dshot_arm_cmd(int argc, char **argv)
{
    LOG_I("Starting ESC arming sequence ");

    g_dshot_run_enable   = 1;
    g_dshot_run_throttle = 0;
}
MSH_CMD_EXPORT(dshot_arm_cmd, dshot_arm - Arm ESC);

static void dshot_stop_cmd(int argc, char **argv)
{
    LOG_I("Starting ESC arming sequence ");

    g_dshot_run_enable = 0;
}
MSH_CMD_EXPORT(dshot_stop_cmd, dshot_stop - Stop ESC);
