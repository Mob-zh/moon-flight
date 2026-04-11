#ifndef _ANO_DATA_H
#define _ANO_DATA_H

#include <rtthread.h>

// 调试模式开关：定义ANO_DT_DEBUG时，函数为空实现（不发送数据，节省运行时间）
#define ANO_DT_DEBUG 1
// ==================== 数据发送开关 ====================
#define ANO_SEND_EULER   1 // 发送欧拉角 (0x03)
#define ANO_SEND_IMU_RAW 0 // 发送IMU原始数据 (0x01)
#define ANO_SEND_GPS     1 // 发送GPS数据 (0x30)
#define ANO_SEND_BARO    1 // 发送气压计数据 (0x02)
#define ANO_SEND_RC      0 // 发送遥控器数据 (0x40)
#define ANO_SEND_PID     0 // 发送PID参数 (0xF1)

// PID参数发送函数
void ANO_DT_Send_Euler_Angles(float A, float B, float C);

void ANO_DT_Send_PID_Params(float rate_kp_roll, float rate_kp_pitch, float rate_kp_yaw,
                            float rate_ki_roll, float rate_ki_pitch, float rate_ki_yaw,
                            float rate_kd_roll, float rate_kd_pitch, float rate_kd_yaw,
                            float rate_limit);
void ANO_DT_Send_Angle_PID(float angle_kp_roll, float angle_kp_pitch, float angle_kp_yaw,
                           float angle_ki_roll, float angle_ki_pitch, float angle_ki_yaw,
                           float angle_kd_roll, float angle_kd_pitch, float angle_kd_yaw,
                           float angle_limit);
void ANO_DT_Send_Pos_PID(float pos_kp_n, float pos_kp_e, float pos_ki_n, float pos_ki_e,
                         float pos_kd_n, float pos_kd_e, float alt_kp, float alt_ki, float alt_kd,
                         float alt_limit);
#endif
