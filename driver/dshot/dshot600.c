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
#include <rtdbg.h>
#include <rtthread.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
// DMA缓冲区（4通道独立缓冲区）
static uint16_t dshot_dma_buffer[DSHOT_CHANNEL_MAX][DSHOT_BITBANG_DMA_BUFFER_SIZE] = {0};
// 渐变油门测试变量
static uint16_t test_throttle = DSHOT_THROTTLE_MIN;
static uint8_t  throttle_step = 1;
// 通道状态变量
static uint16_t channel_throttle[DSHOT_CHANNEL_MAX]  = {DSHOT_THROTTLE_MIN};
static uint8_t  channel_telemetry[DSHOT_CHANNEL_MAX] = {0};

rt_event_t dshot_event;

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

    dshot_event = rt_event_create("dshot_event", RT_IPC_FLAG_PRIO);
    if (dshot_event == RT_NULL)
    {
        LOG_E("dshot_event create failed");
        return false;
    }

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
void dshot600_fill_dma_buffer(uint16_t *dma_buf, uint16_t packet)
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
        rt_event_recv(dshot_event, DSHOT1_DMA_FDT_EVENT, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &e);
        dma_channel_enable(DSHOT_DMA_CHANNEL_1, FALSE); // 先禁用
        wk_dma_channel_config(DSHOT_DMA_CHANNEL_1,
                              (uint32_t)DSHOT_TMR3_CH1_CCR,
                              (uint32_t)dshot_dma_buffer[channel],
                              DSHOT_BITBANG_DMA_BUFFER_SIZE);
        dma_channel_enable(DSHOT_DMA_CHANNEL_1, TRUE); // 重新启用
        break;

    case DSHOT_CHANNEL_2:
        rt_event_recv(dshot_event, DSHOT2_DMA_FDT_EVENT, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &e);
        dma_channel_enable(DSHOT_DMA_CHANNEL_2, FALSE); // 先禁用
        wk_dma_channel_config(DSHOT_DMA_CHANNEL_2,
                              (uint32_t)DSHOT_TMR3_CH2_CCR,
                              (uint32_t)dshot_dma_buffer[channel],
                              DSHOT_BITBANG_DMA_BUFFER_SIZE);
        dma_channel_enable(DSHOT_DMA_CHANNEL_2, TRUE); // 重新启用
        break;

    case DSHOT_CHANNEL_3:
        rt_event_recv(dshot_event, DSHOT3_DMA_FDT_EVENT, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &e);
        dma_channel_enable(DSHOT_DMA_CHANNEL_3, FALSE); // 先禁用
        wk_dma_channel_config(DSHOT_DMA_CHANNEL_3,
                              (uint32_t)DSHOT_TMR4_CH1_CCR,
                              (uint32_t)dshot_dma_buffer[channel],
                              DSHOT_BITBANG_DMA_BUFFER_SIZE);
        dma_channel_enable(DSHOT_DMA_CHANNEL_3, TRUE); // 重新启用
        break;

    case DSHOT_CHANNEL_4:
        rt_event_recv(dshot_event, DSHOT4_DMA_FDT_EVENT, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &e);
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
void dshot600_beeper(dshot_channel_e channel, uint8_t beep_type)
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

void dshot600_set_throttle(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage:\n");
        rt_kprintf("  set_throttle <throttle> - Set throttle to n%% (0-100)\n");
        return;
    }

    int throttle = atoi(argv[1]);
    dshot600_send_throttle(DSHOT_CHANNEL_1, throttle);
}
MSH_CMD_EXPORT(dshot600_set_throttle, Set throttle to n % % (0 - 100));

/* 持续发送测试变量 */
static uint8_t         g_dshot_run_enable   = 0;
static uint16_t        g_dshot_run_throttle = 0;
static dshot_channel_e g_dshot_run_channel  = DSHOT_CHANNEL_1;

/**
 * @brief  停止持续发送
 */
void dshot600_run_stop(void)
{
    g_dshot_run_enable = 0;
}

/**
 * @brief  持续发送任务
 */
static void dshot_run_task(void *parameter)
{
    LOG_I("DShot run: channel=%d, throttle=%d", g_dshot_run_channel, g_dshot_run_throttle);

    while (g_dshot_run_enable)
    {
        dshot600_send_throttle(g_dshot_run_channel, g_dshot_run_throttle);
    }

    LOG_I("DShot run stopped");
}

/**
 * @brief  dshot_run 命令 - 持续发送油门
 * 用法: dshot_run <channel> <throttle>
 *       channel: 0-3, throttle: 48-2047
 *       dshot_stop 停止
 */
static void dshot_run_cmd(int argc, char **argv)
{
    if (argc < 3)
    {
        rt_kprintf("Usage: dshot_run <channel> <throttle>\n");
        rt_kprintf("  channel: 0-3 (M1-M4)\n");
        rt_kprintf("  throttle: 48-2047 (建议 100-500 测试)\n");
        rt_kprintf("Example: dshot_run 0 200\n");
        return;
    }

    uint8_t  channel  = atoi(argv[1]);
    uint16_t throttle = atoi(argv[2]);

    if (channel >= DSHOT_CHANNEL_MAX)
    {
        rt_kprintf("Error: channel must be 0-3\n");
        return;
    }

    if (throttle < DSHOT_THROTTLE_MIN_POWER || throttle > DSHOT_THROTTLE_MAX)
    {
        rt_kprintf("Error: throttle must be %d-%d\n", DSHOT_THROTTLE_MIN_POWER, DSHOT_THROTTLE_MAX);
        return;
    }

    // 如果已经在运行，先停止
    if (g_dshot_run_enable)
    {
        g_dshot_run_enable = 0;
        rt_thread_mdelay(20);
    }

    g_dshot_run_channel  = (dshot_channel_e)channel;
    g_dshot_run_throttle = throttle;
    g_dshot_run_enable   = 1;

    // 创建线程持续发送
    rt_thread_t tid = rt_thread_create("dshot_run",
                                       dshot_run_task,
                                       RT_NULL,
                                       2048,
                                       15,
                                       10);
    if (tid)
    {
        rt_thread_startup(tid);
        rt_kprintf("DShot run started: channel=%d, throttle=%d\n", channel, throttle);
    }
    else
    {
        rt_kprintf("Failed to create dshot_run thread\n");
    }
}
MSH_CMD_EXPORT(dshot_run_cmd, dshot_run<channel><throttle> - Continuous throttle test);

static void dshot_channel_cmd(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: dshot_set <throttle>\n");
        rt_kprintf("  throttle: 48-2047\n");
        return;
    }

    uint8_t channel = atoi(argv[1]);

    if (channel >= DSHOT_CHANNEL_MAX)
    {
        rt_kprintf("Error: channel must be 0-3\n");
        return;
    }

    g_dshot_run_channel = channel;
    rt_kprintf("Channel set to %d\n", channel);
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
        rt_kprintf("Usage: dshot_set <throttle>\n");
        rt_kprintf("  throttle: 48-2047\n");
        return;
    }

    uint16_t throttle = atoi(argv[1]);

    if (throttle < DSHOT_THROTTLE_MIN_POWER || throttle > DSHOT_THROTTLE_MAX)
    {
        rt_kprintf("Error: throttle must be %d-%d\n", DSHOT_THROTTLE_MIN_POWER, DSHOT_THROTTLE_MAX);
        return;
    }

    g_dshot_run_throttle = throttle;
    rt_kprintf("Throttle set to %d\n", throttle);
}
MSH_CMD_EXPORT(dshot_throttle_cmd, dshot_set<throttle> - Set throttle in real - time);

/**
 * @brief  dshot_stop 命令 - 停止持续发送
 */
static void dshot_stop_cmd(int argc, char **argv)
{
    dshot600_run_stop();
    rt_kprintf("DShot stopped\n");

    // 发送停止命令到所有通道
    for (uint8_t ch = 0; ch < DSHOT_CHANNEL_MAX; ch++)
    {
        dshot600_motor_stop((dshot_channel_e)ch);
    }
}
MSH_CMD_EXPORT(dshot_stop_cmd, dshot_stop - Stop continuous throttle);

/**
 * @brief  dshot_arm 命令 - 解锁电调
 * 用法: dshot_arm
 * 说明: 发送命令0持续3秒来解锁电调
 */
static void dshot_arm_cmd(int argc, char **argv)
{
    LOG_I("Starting ESC arming sequence (3 seconds)...");

    uint32_t start = rt_tick_get();
    while (rt_tick_get() - start < 3000)
    {
        for (uint8_t ch = 0; ch < DSHOT_CHANNEL_MAX; ch++)
        {
            dshot600_motor_stop((dshot_channel_e)ch);
        }
    }

    LOG_I("ESC arming complete!");
}
MSH_CMD_EXPORT(dshot_arm_cmd, dshot_arm - Arm ESC with 3s command 0);