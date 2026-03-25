#ifndef _ANO_DATA_H
#define _ANO_DATA_H

#include <rtthread.h>

// 调试模式开关：定义ANO_DT_DEBUG时，函数为空实现（不发送数据，节省运行时间）
#define ANO_DT_DEBUG 1
// ==================== 数据发送开关 ====================
#define ANO_SEND_EULER   1 // 发送欧拉角 (0x03)
#define ANO_SEND_IMU_RAW 0 // 发送IMU原始数据 (0x01)
#define ANO_SEND_GPS     0 // 发送GPS数据 (0x30)
#define ANO_SEND_BARO    0 // 发送气压计数据 (0x02)
#define ANO_SEND_RC      1 // 发送遥控器数据 (0x40)

void ANO_DT_Send_Data(float A, float B, float C);
void ANO_DT_Send_IMU_RawData(int16_t Ax, int16_t Ay, int16_t Az, int16_t Gx, int16_t Gy, int16_t Gz, uint8_t SHOCK_STA);
void ANO_DT_Send_Euler_Angles(float A, float B, float C);
void ANO_DT_Send_Att_RawData(float V0, float V1, float V2, float V3);
void ANO_DT_Send_Sensor_Data(int16_t mag_x, int16_t mag_y, int16_t mag_z, int32_t alt_bar, int16_t temp, uint8_t bar_sta, uint8_t mag_sta);
void ANO_DT_Send_RC_ChData(int16_t rol, int16_t pit, int16_t thr, int16_t yaw,
                           int16_t aux1, int16_t aux2, int16_t aux3, int16_t aux4,
                           int16_t aux5, int16_t aux6);
void ANO_DT_Send_RC_ExData(int16_t aux7, int16_t aux8, int16_t aux9, int16_t aux10);
void ANO_DT_Send_GPS_Data(uint8_t fix_sta, uint8_t s_num,
                          int32_t lng, int32_t lat, int32_t alt_gps,
                          int16_t n_spe, int16_t e_spe, int16_t d_spe,
                          uint8_t pdop, uint8_t sacc, uint8_t vacc);
#endif
