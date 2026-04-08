#include "bz121.h"
#include <rtdevice.h>
#include <rtthread.h>
#include <string.h>

// ==================== 北征121配置 ====================
#define BZ121_UART_DEVICE_NAME "uart2"
#define BZ121_UART_BAUDRATE    115200

// ==================== UBX 消息类/ID ====================
#define UBX_CLASS_NAV  0x01
#define UBX_CLASS_CFG  0x06
#define UBX_ID_NAV_PVT 0x07
#define UBX_ID_CFG_MSG 0x01

// ==================== 私有函数声明 ====================
static void ubx_send_cfg_msg(uint8_t msg_class, uint8_t msg_id, uint8_t rate);

// ==================== 私有变量 ====================
static rt_device_t bz121_uart_dev     = RT_NULL;
static rt_sem_t    bz121_rx_sem       = RT_NULL;
static rt_thread_t bz121_parse_thread = RT_NULL;

// UBX解析状态机
static enum {
    UBX_STATE_SYNC1 = 0, // 等待同步字符1 (0xB5)
    UBX_STATE_SYNC2,     // 等待同步字符2 (0x62)
    UBX_STATE_CLASS,     // 接收消息类
    UBX_STATE_ID,        // 接收消息ID
    UBX_STATE_LEN_L,     // 接收长度低字节
    UBX_STATE_LEN_H,     // 接收长度高字节
    UBX_STATE_PAYLOAD,   // 接收数据负载
    UBX_STATE_CK_A,      // 接收校验和A
    UBX_STATE_CK_B       // 接收校验和B
} ubx_state = UBX_STATE_SYNC1;

static uint8_t  ubx_class         = 0;
static uint8_t  ubx_id            = 0;
static uint16_t ubx_payload_len   = 0;
static uint16_t ubx_payload_count = 0;
static uint8_t  ubx_ck_a          = 0;
static uint8_t  ubx_ck_b          = 0;

// 接收缓冲区
static uint8_t rx_buffer[256];

// 全局GPS设备实例
gpsDev_t g_bz121_gps;

// ==================== 私有函数声明 ====================
static rt_err_t bz121_uart_rx_ind(rt_device_t dev, rt_size_t size);
static void     bz121_parse_thread_entry(void *parameter);
static void     parse_nav_pvt(gps_data_t *data, uint8_t *payload);
static bool     bz121_check_timeout(gpsDev_t *gps);

// ==================== 串口接收回调 ====================
static rt_err_t bz121_uart_rx_ind(rt_device_t dev, rt_size_t size)
{
    if (bz121_rx_sem && size)
    {
        rt_sem_release(bz121_rx_sem);
    }
    return RT_EOK;
}

// ==================== GPS数据解析线程 ====================
static void bz121_parse_thread_entry(void *parameter)
{
    uint8_t ch;

    while (1)
    {
        // 读取所有可用数据
        while (rt_device_read(bz121_uart_dev, 0, &ch, 1) == 1)
        {
            g_bz121_gps.update(&g_bz121_gps, ch);
        }
        // 使用延时5ms，避免数据积压
        rt_thread_mdelay(5);
    }
}

// ==================== NAV-PVT 解析 ====================
// UBX NAV-PVT (0x01 0x07) payload结构 (小端序, 92字节):
// Byte 0-3:   iTOW (GPS周内时间)
// Byte 4-5:   year
// Byte 6:     month
// Byte 7:     day
// Byte 8:     hour
// Byte 9:     min
// Byte 10:    sec
// Byte 11:    valid
// Byte 12-15: tAcc (时间精度ns)
// Byte 16-19: nano (纳秒小数)
// Byte 20:    fix_type (定位类型: 0=无, 1=推算, 2=2D, 3=3D)
// Byte 21:    flags
// Byte 22:    flags2
// Byte 23:    numSV (卫星数)
// Byte 24-27: lon (int32, deg * 1e-7)
// Byte 28-31: lat (int32, deg * 1e-7)
// Byte 32-35: height (椭球高度mm)
// Byte 36-39: hMSL (海平面高度mm)
// Byte 40-43: hAcc (水平精度mm)
// Byte 44-47: vAcc (垂直精度mm)
// Byte 48-51: velN (北向速度mm/s)
// Byte 52-55: velE (东向速度mm/s)
// Byte 56-59: velD (垂向速度mm/s)
// Byte 60-63: gSpeed (地面速度mm/s)
// Byte 64-67: heading (航向1e-5deg)
// Byte 68-71: headingAcc (航向精度)
// Byte 72-73: pDOP
// Byte 80-83: headingVeh
static void parse_nav_pvt(gps_data_t *data, uint8_t *payload)
{
    data->update_ms = rt_tick_get();

    // 定位状态 (Byte 20-23)
    data->fix_type   = payload[20]; // 定位类型: 0=无, 1=推算, 2=2D, 3=3D
    data->fix_status = payload[21]; // 定位标志
    data->num_sats   = payload[23]; // 卫星数

    // 位置 (Byte 24-31, 小端序, deg * 1e-7)
    data->lon           = (int32_t)(payload[24] | (payload[25] << 8) | (payload[26] << 16) | (payload[27] << 24));
    data->lat           = (int32_t)(payload[28] | (payload[29] << 8) | (payload[30] << 16) | (payload[31] << 24));
    data->alt_ellipsoid = (int32_t)(payload[32] | (payload[33] << 8) | (payload[34] << 16) | (payload[35] << 24));
    data->alt_msl       = (int32_t)(payload[36] | (payload[37] << 8) | (payload[38] << 16) | (payload[39] << 24));

    // 精度 (Byte 40-47, mm)
    data->hAcc = (uint32_t)(payload[40] | (payload[41] << 8) | (payload[42] << 16) | (payload[43] << 24));
    data->vAcc = (uint32_t)(payload[44] | (payload[45] << 8) | (payload[46] << 16) | (payload[47] << 24)); // Byte 44: cAcc

    // 速度 (Byte 48-59, mm/s)
    data->vel_n = (int32_t)(payload[48] | (payload[49] << 8) | (payload[50] << 16) | (payload[51] << 24));
    data->vel_e = (int32_t)(payload[52] | (payload[53] << 8) | (payload[54] << 16) | (payload[55] << 24));
    data->vel_d = (int32_t)(payload[56] | (payload[57] << 8) | (payload[58] << 16) | (payload[59] << 24));
    data->sAcc  = (uint32_t)(payload[60] | (payload[61] << 8) | (payload[62] << 16) | (payload[63] << 24));

    // DOP (Byte 76-79)
    data->hDOP = (uint16_t)(payload[76] | (payload[77] << 8));
    data->vDOP = (uint16_t)(payload[78] | (payload[79] << 8));

    // 时间 (Byte 0-10)
    data->year   = (uint16_t)(payload[4] | (payload[5] << 8));
    data->month  = payload[6];
    data->day    = payload[7];
    data->hour   = payload[8];
    data->minute = payload[9];
    data->sec    = payload[10];
    // ms使用iTOW的低16位
    data->ms = (uint16_t)(payload[0] | (payload[1] << 8));
}

// ==================== UBX 消息发送 ====================
static void ubx_calc_checksum(uint8_t *data, uint16_t len, uint8_t *ck_a, uint8_t *ck_b)
{
    *ck_a = 0;
    *ck_b = 0;
    for (uint16_t i = 0; i < len; i++)
    {
        *ck_a += data[i];
        *ck_b += *ck_a;
    }
}

static void ubx_send_message(uint8_t msg_class, uint8_t msg_id, uint8_t *payload, uint16_t payload_len)
{
    if (bz121_uart_dev == RT_NULL)
        return;

    uint8_t  tx_buffer[256];
    uint16_t idx = 0;

    // UBX 帧头
    tx_buffer[idx++] = UBX_SYNC1; // 0xB5
    tx_buffer[idx++] = UBX_SYNC2; // 0x62

    // Class, ID, Length
    tx_buffer[idx++] = msg_class;
    tx_buffer[idx++] = msg_id;
    tx_buffer[idx++] = (uint8_t)(payload_len & 0xFF);
    tx_buffer[idx++] = (uint8_t)((payload_len >> 8) & 0xFF);

    // Payload
    memcpy(&tx_buffer[idx], payload, payload_len);
    idx += payload_len;

    // Checksum
    uint8_t ck_a, ck_b;
    ubx_calc_checksum(&tx_buffer[2], idx - 2, &ck_a, &ck_b);
    tx_buffer[idx++] = ck_a;
    tx_buffer[idx++] = ck_b;

    // 发送
    rt_device_write(bz121_uart_dev, 0, tx_buffer, idx);
}

static void ubx_send_cfg_msg(uint8_t msg_class, uint8_t msg_id, uint8_t rate)
{
    // CFG-MSG: 设置消息输出率
    // Payload: [msgClass, msgId, rate(UART1), rate(UART2), rate(USB), rate(SPI), reserved]
    uint8_t payload[8] = {msg_class, msg_id, rate, rate, rate, rate, 0, 0};
    ubx_send_message(UBX_CLASS_CFG, UBX_ID_CFG_MSG, payload, 8);
}

// ==================== PCA 协议配置 ====================
// 北征121特有配置协议：$PCAS03 关闭NMEA、开启UBX
static void pca_send_command(const char *cmd)
{
    if (bz121_uart_dev == RT_NULL)
        return;
    rt_device_write(bz121_uart_dev, 0, cmd, strlen(cmd));
}

static void gps_configure(void)
{
    rt_thread_mdelay(1000); // 等待GPS模块启动

    // 1. 配置：关闭所有NMEA消息，只开启UBX输出
    // $PCAS03,GGA,RMC,VTG,GSA,GSV,GLL,ZDA,Reserved,UBX
    // = 0,0,0,0,0,0,0,0,1
    pca_send_command("$PCAS03,0,0,0,0,0,0,0,0,1*2C\r\n");
    rt_thread_mdelay(100);

    // 2. 重启使配置生效
    pca_send_command("$PCAS10*51\r\n");
    rt_thread_mdelay(500);

    rt_kprintf("[BZ121] GPS configured: NMEA disabled, UBX enabled\r\n");
}

// ==================== GPS字节解析 ====================
void ubx_update_byte(gpsDev_t *gps, uint8_t byte)
{
    switch (ubx_state)
    {
    case UBX_STATE_SYNC1:
        if (byte == UBX_SYNC1)
        {
            ubx_state = UBX_STATE_SYNC2;
        }
        break;

    case UBX_STATE_SYNC2:
        if (byte == UBX_SYNC2)
        {
            ubx_state = UBX_STATE_CLASS;
            ubx_ck_a  = 0;
            ubx_ck_b  = 0;
        }
        else
        {
            ubx_state = UBX_STATE_SYNC1;
        }
        break;

    case UBX_STATE_CLASS:
        ubx_class = byte;
        ubx_ck_a += byte;
        ubx_ck_b += ubx_ck_a;
        ubx_state = UBX_STATE_ID;
        break;

    case UBX_STATE_ID:
        ubx_id = byte;
        ubx_ck_a += byte;
        ubx_ck_b += ubx_ck_a;
        ubx_state = UBX_STATE_LEN_L;
        break;

    case UBX_STATE_LEN_L:
        ubx_payload_len = byte;
        ubx_ck_a += byte;
        ubx_ck_b += ubx_ck_a;
        ubx_state = UBX_STATE_LEN_H;
        break;

    case UBX_STATE_LEN_H:
        ubx_payload_len |= (byte << 8);
        ubx_ck_a += byte;
        ubx_ck_b += ubx_ck_a;
        ubx_payload_count = 0;
        ubx_state         = UBX_STATE_PAYLOAD;
        break;

    case UBX_STATE_PAYLOAD:
        if (ubx_payload_count < sizeof(rx_buffer))
        {
            rx_buffer[ubx_payload_count] = byte;
        }
        ubx_ck_a += byte;
        ubx_ck_b += ubx_ck_a;
        ubx_payload_count++;

        if (ubx_payload_count >= ubx_payload_len)
        {
            ubx_state = UBX_STATE_SYNC1;
            if (ubx_class == UBX_CLASS_NAV && ubx_id == UBX_ID_NAV_PVT)
            {
                parse_nav_pvt(&gps->data, rx_buffer);
            }
        }
        break;
    }
}

// ==================== 超时检查 ====================
static bool bz121_check_timeout(gpsDev_t *gps)
{
    uint32_t current_time = rt_tick_get();
    uint32_t time_diff    = current_time - gps->data.update_ms;
    return (time_diff < 200); // 2秒超时
}

// ==================== 公开函数 ====================

bool gps_data_ready(gpsDev_t *gps)
{
    return (gps->data.fix_type >= GPS_FIX_3D) && bz121_check_timeout(gps);
}

gps_data_t *gps_get_data(gpsDev_t *gps)
{
    return &gps->data;
}

/**
 * @brief 北征121 GPS初始化
 */
bool bz121_init(gpsDev_t *gps)
{

    gps->update     = ubx_update_byte;
    gps->data_ready = gps_data_ready;
    gps->get_data   = gps_get_data;

    // 初始化GPS数据
    gps->data.fix_type = GPS_FIX_NONE;
    gps->data.num_sats = 0;
    gps->data.lat      = 0;
    gps->data.lon      = 0;
    gps->data.alt_msl  = 0;

    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

    // 查找UART设备
    bz121_uart_dev = rt_device_find(BZ121_UART_DEVICE_NAME);
    if (bz121_uart_dev == RT_NULL)
    {
        rt_kprintf("[BZ121] Error: uart device %s not found!\n", BZ121_UART_DEVICE_NAME);
        return false;
    }

    // 配置串口参数
    config.baud_rate = BZ121_UART_BAUDRATE;
    config.data_bits = DATA_BITS_8;
    config.stop_bits = STOP_BITS_1;
    config.parity    = PARITY_NONE;
    config.bufsz     = 512;

    if (rt_device_control(bz121_uart_dev, RT_DEVICE_CTRL_CONFIG, &config) != RT_EOK)
    {
        rt_kprintf("[BZ121] Error: uart config failed!\n");
        return false;
    }

    // 打开串口（中断接收模式）
    if (rt_device_open(bz121_uart_dev, RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("[BZ121] Error: uart open failed!\n");
        return false;
    }

    // 设置接收回调
    // 关闭中断回调，防止线程调度问题导致飞控线程失效
    // rt_device_set_rx_indicate(bz121_uart_dev, bz121_uart_rx_ind);

    // 创建信号量
    bz121_rx_sem = rt_sem_create("bz121_rx", 0, RT_IPC_FLAG_PRIO);
    if (bz121_rx_sem == RT_NULL)
    {
        rt_kprintf("[BZ121] Error: create semaphore failed!\n");
        return false;
    }

    // 配置GPS输出UBX NAV-PVT消息
    gps_configure();

    // 创建解析线程
    bz121_parse_thread = rt_thread_create("bz121_parse",
                                          bz121_parse_thread_entry,
                                          RT_NULL,
                                          2048,
                                          15,
                                          10);
    if (bz121_parse_thread != RT_NULL)
    {
        rt_thread_startup(bz121_parse_thread);
    }

    rt_kprintf("[BZ121] GPS driver initialized on %s, baud=%d\n",
               BZ121_UART_DEVICE_NAME, BZ121_UART_BAUDRATE);

    return true;
}

bool gps_init(gpsDev_t *gps)
{
    // 设置函数指针
    gps->init = bz121_init;

    return gps->init(gps);
}