#include "ano_data.h"
#include "bmp280.h"
#include "bz121.h"
#include "elrs.h"
#include "flight_control.h"
#include "icm42688.h"
#include "imu.h"
#include <rtthread.h>

// ==================== 配置 ====================
#define ANO_SEND_PERIOD 10 // 发送周期 (ms)

// ==================== 全局对象引用 ====================
extern FLOAT_ANGLE  Att_Angle;       // 欧拉角
extern gpsDev_t     g_bz121_gps;     // GPS
extern baroDev_t    g_bmp280_baro;   // 气压计
extern accgyroDev_t g_icm_accgyro;   // 惯导设备
extern elrsDev_t    g_elrs_receiver; // 遥控器接收
extern FLOAT_XYZ    Gyr_filt;
extern FLOAT_XYZ    Acc_filt;
extern float        rate_limit_max;
// ==================== ANO发送线程 ====================
static void ano_send_thread_entry(void *parameter)
{
    (void)parameter; // 避免警告

    while (1)
    {
#if ANO_SEND_EULER
        // 发送欧拉角 (0x03)
        ANO_DT_Send_Euler_Angles(Att_Angle.pit, Att_Angle.rol, Att_Angle.yaw);
        // 转弧度为角度再*100
        // ANO_DT_Send_IMU_RawData(0, 0, 0, (int16_t)(Gyr_filt.X * 180 / M_PI * 100), (int16_t)(Gyr_filt.Y * 180 / M_PI * 100), (int16_t)(Gyr_filt.Z * 180 / M_PI * 100), 0);
#endif
#if PRINT_EULER
        rt_kprintf("pit:%d, rol:%d, yaw:%d\n", (int16_t)(Att_Angle.pit * 100), (int16_t)(Att_Angle.rol * 100), (int16_t)(Att_Angle.yaw * 100));
#endif

#if ANO_SEND_IMU_RAW
        // 发送IMU原始数据 (0x01)
        ANO_DT_Send_IMU_RawData(g_icm_accgyro.accData[0], g_icm_accgyro.accData[1],
                                g_icm_accgyro.accData[2], g_icm_accgyro.gyroData[0],
                                g_icm_accgyro.gyroData[1], g_icm_accgyro.gyroData[2], 0);
#endif

#if ANO_SEND_PID
        ANO_DT_Send_PID_Params(g_flight_control.pid_rate_roll.kp, g_flight_control.pid_rate_pitch.kp, g_flight_control.pid_rate_yaw.kp,
                               g_flight_control.pid_rate_roll.ki, g_flight_control.pid_rate_pitch.ki, g_flight_control.pid_rate_yaw.ki,
                               g_flight_control.pid_rate_roll.kd, g_flight_control.pid_rate_pitch.kd, g_flight_control.pid_rate_yaw.kd,
                               rate_limit_max);
#endif

#if ANO_SEND_GPS
        // 发送GPS数据 (0x30)
        gps_data_t *gps_data = g_bz121_gps.get_data(&g_bz121_gps);

        // PDOP: 0-20000 -> 0-200
        uint8_t pdop = (gps_data->hDOP > 20000) ? 200 : (uint8_t)(gps_data->hDOP / 100);
        // SACC: mm/s -> mm/s/100
        uint8_t sacc = (gps_data->sAcc > 20000) ? 200 : (uint8_t)(gps_data->sAcc / 100);
        // VACC: mm -> mm/100
        uint8_t vacc = (gps_data->vAcc > 20000) ? 200 : (uint8_t)(gps_data->vAcc / 100);

        ANO_DT_Send_GPS_Data(
            gps_data->fix_type,              // FIX_STA
            gps_data->num_sats,              // S_NUM
            gps_data->lon,                   // LNG (deg * 1e-7)
            gps_data->lat,                   // LAT (deg * 1e-7)
            gps_data->alt_msl,               // ALT_GPS (mm)
            (int16_t)(gps_data->vel_n / 10), // N_SPE (cm/s)
            (int16_t)(gps_data->vel_e / 10), // E_SPE (cm/s)
            (int16_t)(gps_data->vel_d / 10), // D_SPE (cm/s)
            pdop,                            // PDOP
            sacc,                            // SACC
            vacc                             // VACC
        );
#endif
#if PRINT_GPS
        gps_data_t *gps_data = g_bz121_gps.get_data(&g_bz121_gps);
        // PDOP: 0-20000 -> 0-200
        uint8_t pdop = (gps_data->hDOP > 20000) ? 200 : (uint8_t)(gps_data->hDOP / 100);
        // SACC: mm/s -> mm/s/100
        uint8_t sacc = (gps_data->sAcc > 20000) ? 200 : (uint8_t)(gps_data->sAcc / 100);
        // VACC: mm -> mm/100
        uint8_t vacc = (gps_data->vAcc > 20000) ? 200 : (uint8_t)(gps_data->vAcc / 100);
        rt_kprintf("GPS data:LNG=%d, LAT=%d, ALT=%d, N_SPE=%d, E_SPE=%d, D_SPE=%d, PDOP=%d, SACC=%d, VACC=%d\n", gps_data->lon, gps_data->lat, gps_data->alt_msl, gps_data->vel_n, gps_data->vel_e, gps_data->vel_d, pdop, sacc, vacc);
#endif

#if ANO_SEND_BARO
        // 发送气压计数据 (0x02)
        ANO_DT_Send_Sensor_Data(0, 0, 0, g_bmp280_baro.altitude,
                                (int16_t)(g_bmp280_baro.temp / 10), 0, 0);

#endif
#if PRINT_BARO
        rt_kprintf("BARO data:ALT=%d, TEMP=%d\n", g_bmp280_baro.altitude, (int16_t)(g_bmp280_baro.temp / 10));
#endif

#if ANO_SEND_RC
        // 发送遥控器数据 (0x40)
        ANO_DT_Send_RC_ChData(
            g_elrs_receiver.ch1_roll,     // ROL
            g_elrs_receiver.ch2_pitch,    // PIT
            g_elrs_receiver.ch3_throttle, // THR
            g_elrs_receiver.ch4_yaw,      // YAW
            g_elrs_receiver.ch5_arm,      // AUX1
            g_elrs_receiver.ch6_aux2,     // AUX2
            g_elrs_receiver.ch7_mode,     // AUX3
            g_elrs_receiver.ch8_aux4,     // AUX4
            0,                            // AUX5
            0);                           // AUX6
#endif

        rt_thread_mdelay(ANO_SEND_PERIOD);
    }
}

// ==================== ANO初始化 ====================
int ano_update_init(void)
{
    rt_thread_t ano_thread = rt_thread_create("ano_send",
                                              ano_send_thread_entry,
                                              RT_NULL,
                                              1024,
                                              20,
                                              10);
    if (ano_thread != RT_NULL)
    {
        rt_thread_startup(ano_thread);
    }

    rt_kprintf("[ANO] ANO update thread started\n");
    return 0;
}
