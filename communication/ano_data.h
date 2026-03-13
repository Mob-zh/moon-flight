#ifndef _ANO_DATA_H
#define _ANO_DATA_H

#include <rtthread.h>

void ANO_DT_Send_Data(float A, float B, float C);
void ANO_DT_Send_IMU_RawData(int16_t Ax, int16_t Ay, int16_t Az, int16_t Gx, int16_t Gy, int16_t Gz, uint8_t SHOCK_STA);
void ANO_DT_Send_Euler_Angles(float A, float B, float C);
void ANO_DT_Send_Att_RawData(float V0, float V1, float V2, float V3);
#endif
