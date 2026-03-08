#include "elrs.h"
#include <rtdevice.h>
#include <rtthread.h>

/* 简单的最大/最小宏，避免依赖外部定义 */
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

// ELRS接收机实例与同步信号量
elrsDev_t       g_elrs_receiver;
static rt_sem_t elrs_sem = RT_NULL;

/************************ 私有函数声明 ************************/
static rt_err_t elrs_uart_rx_ind(rt_device_t dev, rt_size_t size);
static void     crsf_process_byte(elrsDev_t *dev, uint8_t ch);
static void     elrs_scale_channels(elrsDev_t *dev);
static void     elrs_update_thread_entry(void *parameter);

/************************ 串口回调（中断中只释放信号量） ************************/
/**
 * @brief 串口接收回调：中断中仅释放信号量，通知线程有数据到来
 */
static rt_err_t elrs_uart_rx_ind(rt_device_t dev, rt_size_t size)
{
    if (elrs_sem && size)
    {
        rt_sem_release(elrs_sem);
    }
    return RT_EOK;
}

/* 计算 CRSF 帧校验和（从 frame[2] 开始，共 len 字节） */
static uint8_t crsf_calc_checksum(const uint8_t *buf, uint16_t len)
{
    uint8_t crc = 0; // 初始值为0
    for (uint8_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0xD5;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

/**
 * @brief CRSF 字节流解析状态机（简单版）
 * 每收到 1 个字节都调用一次，内部在接收完整、校验通过的通道帧后，直接更新通道数据。
 */
static void crsf_process_byte(elrsDev_t *dev, uint8_t ch)
{
    if (!dev->crsf_in_frame)
    {
        /* 还没进入帧状态，只接受同步字节 */
        if (ch == CRSF_SYNC_BYTE)
        {
            dev->crsf_in_frame               = true;
            dev->crsf_pos                    = 0;
            dev->crsf_expected_len           = 0;
            dev->crsf_frame[dev->crsf_pos++] = ch; // 保存同步字节
        }
        return;
    }

    /* 已经处于“正在接收一帧”的状态 */
    if (dev->crsf_pos == 1)
    {
        /* 第二个字节：length 字段 */
        dev->crsf_frame[dev->crsf_pos++] = ch;
        dev->crsf_expected_len           = ch;

        /* 基本合理性检查：长度不能超过本地缓存能力 */
        if (dev->crsf_expected_len + 2 > CRSF_CHANNEL_FRAME_LENGTH)
        {
            /* 认为这帧无效，重新等待下一帧 */
            dev->crsf_in_frame     = false;
            dev->crsf_pos          = 0;
            dev->crsf_expected_len = 0;
        }
        return;
    }

    /* 后续字节：类型 + payload + 校验 */
    if (dev->crsf_pos < CRSF_CHANNEL_FRAME_LENGTH)
    {
        dev->crsf_frame[dev->crsf_pos++] = ch;
    }

    /* 判断一帧是否收完整：frame[0]=sync, frame[1]=len, 共 len+2 字节 */
    if (dev->crsf_in_frame && dev->crsf_expected_len > 0 &&
        dev->crsf_pos >= (uint8_t)(dev->crsf_expected_len + 2))
    {

        uint8_t *frame      = dev->crsf_frame;
        uint8_t  frame_len  = dev->crsf_expected_len;
        uint8_t  frame_type = frame[2];

        /* 只处理通道数据帧，且长度必须匹配预期 */
        if (frame_type == CRSF_FRAME_TYPE_CHANNELS &&
            frame_len == (CRSF_CHANNEL_FRAME_LENGTH - 2))
        {

            /* 校验和：从 frame[2] 开始，长度 = frame_len-1（含类型和 payload，不含最后 1 字节校验和） */
            uint8_t calc = crsf_calc_checksum(&frame[2], frame_len - 1);
            uint8_t recv = frame[2 + frame_len - 1];

            if (calc == recv)
            {
                /* 解析 16 通道，每通道 11bit，按 CRSF 标准打包 */
								uint64_t bitBuffer = 0;
								uint8_t bitsInBuffer = 0;
								int byteIndex = 3;

								for (int i = 0; i < ELRS_CHANNEL_COUNT; i++) {
										while (bitsInBuffer < 11) {
												// 从payload中读取字节
												bitBuffer |= ((uint64_t)frame[byteIndex++] << bitsInBuffer);
												bitsInBuffer += 8;
										}
										dev->raw_channels[i] = bitBuffer & 0x07FF; // 取低11位作为通道值
										bitBuffer >>= 11;
										bitsInBuffer -= 11;
								}


                dev->last_update_time = rt_tick_get_millisecond();
                dev->is_connected     = true;
                elrs_scale_channels(dev);
            }
        }

        /* 无论成功与否，当前帧结束，重新等待下一帧 */
        dev->crsf_in_frame     = false;
        dev->crsf_pos          = 0;
        dev->crsf_expected_len = 0;
    }
}

/************************ 通道值标准化 ************************/
/**
 * @brief 将原始CRSF通道值转换为-1000~1000的标准化值
 * @param dev: ELRS设备结构体
 */
static void elrs_scale_channels(elrsDev_t *dev)
{
    for (uint8_t ch = 0; ch < ELRS_CHANNEL_COUNT; ch++)
    {
        /* 限幅原始值，保证在协议定义范围内 */
        uint16_t raw = dev->raw_channels[ch];
        raw          = MAX(CRSF_CHANNEL_MIN, MIN(CRSF_CHANNEL_MAX, raw));

        /* 线性转换为 -1000 ~ 1000，0 为中位 */
        dev->scaled_channels[ch] =
            (int16_t)(((float)(raw - CRSF_CHANNEL_MID) /
                       (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MID)) *
                      ELRS_CHANNEL_SCALE_MAX);
    }

    /* 语义通道映射（按常见 RC 约定：CH1~4 为飞控主通道） */
    dev->ch1_roll  = dev->scaled_channels[0]; // CH1
    dev->ch2_pitch = dev->scaled_channels[1]; // CH2

    /* 油门通常使用 0~1000，更直观 */
    {
        int16_t s = dev->scaled_channels[2]; // CH3
        if (s < ELRS_CHANNEL_SCALE_MIN)
            s = ELRS_CHANNEL_SCALE_MIN;
        if (s > ELRS_CHANNEL_SCALE_MAX)
            s = ELRS_CHANNEL_SCALE_MAX;
        dev->ch3_throttle = (int16_t)((s + ELRS_CHANNEL_SCALE_MAX) / 2); // -1000~1000 -> 0~1000
    }

    dev->ch4_yaw  = dev->scaled_channels[3]; // CH4
    dev->ch5_aux1 = dev->scaled_channels[4]; // CH5
    dev->ch6_aux2 = dev->scaled_channels[5]; // CH6
    dev->ch7_aux3 = dev->scaled_channels[6]; // CH7
    dev->ch8_aux4 = dev->scaled_channels[7]; // CH8
}

/************************ 核心功能函数 ************************/
/**
 * @brief ELRS接收机初始化
 * @param dev: ELRS设备结构体
 * @return TRUE-初始化成功，FALSE-失败
 */
static bool elrs_receiver_init(elrsDev_t *dev)
{
    if (dev == NULL)
        return false;

    // 初始化 CRSF 断帧/解析状态
    rt_memset(dev->crsf_frame, 0, sizeof(dev->crsf_frame));
    dev->crsf_pos          = 0;
    dev->crsf_expected_len = 0;
    dev->crsf_in_frame     = false;

    // 初始化通道数据
    rt_memset(dev->raw_channels, 0, sizeof(dev->raw_channels));
    rt_memset(dev->scaled_channels, 0, sizeof(dev->scaled_channels));
    dev->last_update_time = 0;
    dev->is_connected     = false;

    // 绑定并配置 RT-Thread 串口设备
    dev->uart_dev = rt_device_find(ELRS_UART_DEVICE_NAME);
    if (dev->uart_dev == RT_NULL)
    {
        rt_kprintf("ELRS: uart device %s not found!\n", ELRS_UART_DEVICE_NAME);
        return false;
    }

    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate               = ELRS_UART_BAUDRATE;
    config.data_bits               = DATA_BITS_8;
    config.stop_bits               = STOP_BITS_1;
    config.parity                  = PARITY_NONE;

    if (rt_device_control(dev->uart_dev, RT_DEVICE_CTRL_CONFIG, &config) != RT_EOK)
    {
        rt_kprintf("ELRS: uart config failed!\n");
        return false;
    }

    if (rt_device_open(dev->uart_dev, RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("ELRS: uart open failed!\n");
        return false;
    }

    rt_device_set_rx_indicate(dev->uart_dev, elrs_uart_rx_ind);

    // 创建信号量和处理线程
    if (elrs_sem == RT_NULL)
    {
        elrs_sem = rt_sem_create("elrs_sem", 0, RT_IPC_FLAG_PRIO);
        if (elrs_sem == RT_NULL)
        {
            rt_kprintf("ELRS: sem create failed!\n");
            return false;
        }
    }

    rt_thread_t elrs_thread =
        rt_thread_create("elrs", elrs_update_thread_entry, RT_NULL, 1024, 20, 10);
    if (elrs_thread == RT_NULL)
    {
        rt_kprintf("ELRS: thread create failed!\n");
        return false;
    }
    rt_thread_startup(elrs_thread);

    rt_kprintf("ELRS receiver init success on %s, baud=%d\n",
               ELRS_UART_DEVICE_NAME, ELRS_UART_BAUDRATE);
    return true;
}

/**
 * @brief 读取ELRS通道数据
 * @param dev: ELRS设备结构体
 * @return TRUE-读取成功，FALSE-失败（超时/未连接）
 */
static bool elrs_read_channels(elrsDev_t *dev)
{
    if (dev == NULL)
        return false;

    // 检查连接超时（500ms 无更新则判定为断开）
    if (rt_tick_get_millisecond() - dev->last_update_time > 500)
    {
        dev->is_connected = false;
        return false;
    }

    return dev->is_connected;
}

/************************ 对外接口 ************************/
/**
 * @brief 初始化ELRS接收机
 * @param dev: ELRS设备结构体
 * @return TRUE-初始化成功，FALSE-失败
 */
bool elrs_init(elrsDev_t *dev)
{
    if (dev == NULL)
        return false;

    // 绑定函数指针
    dev->init          = elrs_receiver_init;
    dev->read_channels = elrs_read_channels;

    // 执行初始化
    if (!(dev->init(dev)))
    {
        rt_kprintf("ELRS receiver initialization failed!\n");
        return false;
    }

    return true;
}

/**
 * @brief ELRS 数据处理线程：等待信号量 -> 从串口读取 -> 断帧 -> 解析
 * 流程与 uart1_test_interrupt 一致：中断只通知，线程中完成读取和解析
 */
static void elrs_update_thread_entry(void *parameter)
{
    uint8_t ch;

    while (1)
    {
        rt_sem_take(elrs_sem, RT_WAITING_FOREVER);

        /* 从串口读取当前缓冲区的全部字节，逐字节断帧并解析 */
        while (rt_device_read(g_elrs_receiver.uart_dev, 0, &ch, 1) == 1)
        {
            crsf_process_byte(&g_elrs_receiver, ch);
        }
    }
}
