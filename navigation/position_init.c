#include "position_init.h"
#include "bmp280.h"
#include "bz121.h"
#include "position_ekf.h"
#include <rtthread.h>

// ==================== 全局变量 ====================
extern baroDev_t      g_bmp280_baro;
extern gpsDev_t       g_bz121_gps;
extern position_ekf_t g_position_ekf;

// ==================== 私有变量 ====================
static rt_thread_t position_thread = RT_NULL;

#define GPS_TEMP_BUF_SIZE 64
static uint8_t gps_temp_buf[GPS_TEMP_BUF_SIZE];

// ==================== 外部函数声明 ====================
extern rt_device_t bz121_get_uart_dev(void);

// ==================== 位置传感器线程 ====================
static void position_thread_entry(void *parameter)
{
    rt_device_t uart_dev = bz121_get_uart_dev();

    while (1)
    {
        // ===== GPS 更新 =====
        if (uart_dev != RT_NULL)
        {
            // 一次性读取64字节
            rt_size_t len = rt_device_read(uart_dev, 0, gps_temp_buf, sizeof(gps_temp_buf));

            for (rt_size_t i = 0; i < len; i++)
            {
                g_bz121_gps.update(&g_bz121_gps, gps_temp_buf[i]);
            }
        }

        // ===== 气压计更新 =====
        if (g_bmp280_baro.read_press)
        {
            g_bmp280_baro.read_press(&g_bmp280_baro);
            if (g_bmp280_baro.get_altitude)
            {
                g_bmp280_baro.altitude = g_bmp280_baro.get_altitude(&g_bmp280_baro, 101325);
            }
        }

        // ===== EKF 更新 =====
        if (g_bmp280_baro.altitude > 0)
        {
            position_ekf_update_baro(&g_position_ekf, g_bmp280_baro.altitude / 100.0f);
        }

        // ===== GPS数据更新到EKF =====
        gps_data_t *gps_data = g_bz121_gps.get_data(&g_bz121_gps);
        if (gps_data->fix_type >= 3) // 3D定位
        {
            // vel单位是mm/s，转换为m/s
            position_ekf_update_gps(&g_position_ekf,
                                    gps_data->lat,
                                    gps_data->lon,
                                    gps_data->vel_n / 1000.0f,
                                    gps_data->vel_e / 1000.0f,
                                    gps_data->hDOP);
        }

        rt_thread_mdelay(5); // 200Hz
    }
}

// ==================== 初始化 ====================
void position_sensor_init(void)
{
    baro_init(&g_bmp280_baro);
    gps_init(&g_bz121_gps);
    position_ekf_init(&g_position_ekf, 0.0f, 0.0f);

    position_thread = rt_thread_create("position",
                                       position_thread_entry,
                                       RT_NULL,
                                       2048,
                                       15,
                                       10);
    if (position_thread != RT_NULL)
        rt_thread_startup(position_thread);
}