#ifndef _GPS_H_
#define _GPS_H_

#include <rtdevice.h>
#include <stdbool.h>
#include <stdint.h>

// ==================== UART配置 ====================
#define GPS_UART_DEVICE_NAME "uart2"
#define GPS_UART_BAUDRATE    115200

// ==================== u-blox协议定义 ====================

// UBX消息类
#define UBX_CLASS_NAV 0x01 // Navigation

// UBX消息ID
#define UBX_ID_NAV_PVT    0x07 // Position Velocity Time
#define UBX_ID_NAV_POSLLH 0x02 // Position LLA
#define UBX_ID_NAV_VELNED 0x12 // Velocity NED

// UBX协议头
#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62

// GPS固定类型
#define GPS_FIX_NONE 0 // 无定位
#define GPS_FIX_DEAD 1 // 推算定位
#define GPS_FIX_2D   2 // 2D定位
#define GPS_FIX_3D   3 // 3D定位
#define GPS_FIX_GNSS 4 // GNSS+DR
#define GPS_FIX_TIME 5 // 时间

// ==================== GPS数据状态 ====================
typedef struct
{
    // 定位状态
    uint8_t fix_type;   // 定位类型
    uint8_t num_sats;   // 卫星数量
    uint8_t fix_status; // 定位状态标志

    // 位置（度，1e-7）
    int32_t lat;           // 纬度 (deg * 1e-7)
    int32_t lon;           // 经度 (deg * 1e-7)
    int32_t alt_ellipsoid; // 椭球高度 (mm)
    int32_t alt_msl;       // 海拔高度 (mm)

    // 速度 (mm/s)
    int32_t vel_n; // 北向速度
    int32_t vel_e; // 东向速度
    int32_t vel_d; // 向下速度

    // 时间
    uint16_t year;   // 年
    uint8_t  month;  // 月
    uint8_t  day;    // 日
    uint8_t  hour;   // 时
    uint8_t  minute; // 分
    uint8_t  sec;    // 秒
    uint16_t ms;     // 毫秒

    // 精度
    uint16_t hDOP; // 水平精度因子 (HDOP * 100)
    uint16_t vDOP; // 垂直精度因子 (VDOP * 100)
    int32_t  hAcc; // 水平精度估计 (mm)
    int32_t  vAcc; // 垂直精度估计 (mm)

    // 速度精度
    uint32_t sAcc; // 速度精度估计 (mm/s)

    // 数据有效性
    uint32_t update_ms; // 最后更新时刻
} gps_data_t;

// ==================== GPS设备结构体 ====================
typedef struct gpsDev_s gpsDev_t;

struct gpsDev_s
{
    // GPS数据
    gps_data_t data;

    // 初始化函数
    bool (*init)(gpsDev_t *);

    // 更新函数（由串口中断调用）
    void (*update)(gpsDev_t *, uint8_t byte);

    // 数据就绪检查
    bool (*data_ready)(gpsDev_t *);

    // 获取原始数据指针
    gps_data_t *(*get_data)(gpsDev_t *);
};

extern gpsDev_t g_bz121_gps;

// ==================== 函数声明 ====================

/**
 * @brief GPS初始化
 * @retval true-成功, false-失败
 */
bool gps_init(gpsDev_t *gps);

#endif /* _GPS_H_ */
