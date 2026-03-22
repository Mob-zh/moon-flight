#include "ano_data.h"
#include <rtthread.h>

// 数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp) (*((char *)(&dwTemp)))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

// 调试模式开关：定义ANO_DT_DEBUG时，函数为空实现（不发送数据，节省运行时间）
#define ANO_DT_DEBUG 0

#if ANO_DT_DEBUG

static void ano_send_data(uint8_t *buff, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++)
    {
        rt_kprintf("%c", buff[i]);
    }
}

void ANO_DT_Send_Euler_Angles(float A, float B, float C)
{
    uint8_t BUFF[16];
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
    uint8_t _cnt     = 0;
    int16_t A_int    = (int16_t)(A * 100);
    int16_t B_int    = (int16_t)(B * 100);
    int16_t C_int    = (int16_t)(C * 100);

    BUFF[_cnt++] = 0xAA; // 帧头
    BUFF[_cnt++] = 0xFF; // 目标地址
    BUFF[_cnt++] = 0X03; // 功能码
    BUFF[_cnt++] = 0x07; // 数据长度
    BUFF[_cnt++] = BYTE0(A_int);
    BUFF[_cnt++] = BYTE1(A_int);
    BUFF[_cnt++] = BYTE0(B_int);
    BUFF[_cnt++] = BYTE1(B_int);
    BUFF[_cnt++] = BYTE0(C_int);
    BUFF[_cnt++] = BYTE1(C_int);
    BUFF[_cnt++] = 0x00;

    for (uint8_t i = 0; i < BUFF[3] + 4; i++)
    {
        sumcheck += BUFF[i];
        addcheck += sumcheck;
    }
    BUFF[_cnt++] = sumcheck;
    BUFF[_cnt++] = addcheck;

    ano_send_data(BUFF, _cnt);
}

void ANO_DT_Send_IMU_RawData(int16_t Ax, int16_t Ay, int16_t Az, int16_t Gx, int16_t Gy, int16_t Gz, uint8_t SHOCK_STA)
{
    uint8_t BUFF[32];
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
    uint8_t _cnt     = 0;

    BUFF[_cnt++] = 0xAA;
    BUFF[_cnt++] = 0xFF;
    BUFF[_cnt++] = 0X01;
    BUFF[_cnt++] = 0x0D;
    BUFF[_cnt++] = BYTE0(Ax);
    BUFF[_cnt++] = BYTE1(Ax);
    BUFF[_cnt++] = BYTE0(Ay);
    BUFF[_cnt++] = BYTE1(Ay);
    BUFF[_cnt++] = BYTE0(Az);
    BUFF[_cnt++] = BYTE1(Az);
    BUFF[_cnt++] = BYTE0(Gx);
    BUFF[_cnt++] = BYTE1(Gx);
    BUFF[_cnt++] = BYTE0(Gy);
    BUFF[_cnt++] = BYTE1(Gy);
    BUFF[_cnt++] = BYTE0(Gz);
    BUFF[_cnt++] = BYTE1(Gz);
    BUFF[_cnt++] = SHOCK_STA;

    for (uint8_t i = 0; i < BUFF[3] + 4; i++)
    {
        sumcheck += BUFF[i];
        addcheck += sumcheck;
    }
    BUFF[_cnt++] = sumcheck;
    BUFF[_cnt++] = addcheck;

    ano_send_data(BUFF, _cnt);
}

void ANO_DT_Send_Att_RawData(float V0, float V1, float V2, float V3)
{
    uint8_t BUFF[16];
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
    uint8_t _cnt     = 0;

    int16_t V0_int = (int16_t)(V0 * 10000);
    int16_t V1_int = (int16_t)(V1 * 10000);
    int16_t V2_int = (int16_t)(V2 * 10000);
    int16_t V3_int = (int16_t)(V3 * 10000);

    BUFF[_cnt++] = 0xAA;
    BUFF[_cnt++] = 0xFF;
    BUFF[_cnt++] = 0X04;
    BUFF[_cnt++] = 0x09;
    BUFF[_cnt++] = BYTE0(V0_int);
    BUFF[_cnt++] = BYTE1(V0_int);
    BUFF[_cnt++] = BYTE0(V1_int);
    BUFF[_cnt++] = BYTE1(V1_int);
    BUFF[_cnt++] = BYTE0(V2_int);
    BUFF[_cnt++] = BYTE1(V2_int);
    BUFF[_cnt++] = BYTE0(V3_int);
    BUFF[_cnt++] = BYTE1(V3_int);
    BUFF[_cnt++] = 0x00;

    for (uint8_t i = 0; i < BUFF[3] + 4; i++)
    {
        sumcheck += BUFF[i];
        addcheck += sumcheck;
    }
    BUFF[_cnt++] = sumcheck;
    BUFF[_cnt++] = addcheck;

    ano_send_data(BUFF, _cnt);
}

void ANO_DT_Send_Sensor_Data(int16_t mag_x, int16_t mag_y, int16_t mag_z, int32_t alt_bar, int16_t temp, uint8_t bar_sta, uint8_t mag_sta)
{
    uint8_t BUFF[32];
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
    uint8_t _cnt     = 0;

    BUFF[_cnt++] = 0xAA;
    BUFF[_cnt++] = 0xFF;
    BUFF[_cnt++] = 0x02;
    BUFF[_cnt++] = 0x0E;
    BUFF[_cnt++] = BYTE0(mag_x);
    BUFF[_cnt++] = BYTE1(mag_x);
    BUFF[_cnt++] = BYTE0(mag_y);
    BUFF[_cnt++] = BYTE1(mag_y);
    BUFF[_cnt++] = BYTE0(mag_z);
    BUFF[_cnt++] = BYTE1(mag_z);
    BUFF[_cnt++] = BYTE0(alt_bar);
    BUFF[_cnt++] = BYTE1(alt_bar);
    BUFF[_cnt++] = BYTE2(alt_bar);
    BUFF[_cnt++] = BYTE3(alt_bar);
    BUFF[_cnt++] = BYTE0(temp);
    BUFF[_cnt++] = BYTE1(temp);
    BUFF[_cnt++] = bar_sta;
    BUFF[_cnt++] = mag_sta;

    for (uint8_t i = 0; i < BUFF[3] + 4; i++)
    {
        sumcheck += BUFF[i];
        addcheck += sumcheck;
    }
    BUFF[_cnt++] = sumcheck;
    BUFF[_cnt++] = addcheck;

    ano_send_data(BUFF, _cnt);
}

void ANO_DT_Send_RC_ChData(int16_t rol, int16_t pit, int16_t thr, int16_t yaw,
                           int16_t aux1, int16_t aux2, int16_t aux3, int16_t aux4,
                           int16_t aux5, int16_t aux6)
{
    uint8_t BUFF[32];
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
    uint8_t _cnt     = 0;

    BUFF[_cnt++] = 0xAA;
    BUFF[_cnt++] = 0xFF;
    BUFF[_cnt++] = 0x40;
    BUFF[_cnt++] = 0x14;
    BUFF[_cnt++] = BYTE0(rol);
    BUFF[_cnt++] = BYTE1(rol);
    BUFF[_cnt++] = BYTE0(pit);
    BUFF[_cnt++] = BYTE1(pit);
    BUFF[_cnt++] = BYTE0(thr);
    BUFF[_cnt++] = BYTE1(thr);
    BUFF[_cnt++] = BYTE0(yaw);
    BUFF[_cnt++] = BYTE1(yaw);
    BUFF[_cnt++] = BYTE0(aux1);
    BUFF[_cnt++] = BYTE1(aux1);
    BUFF[_cnt++] = BYTE0(aux2);
    BUFF[_cnt++] = BYTE1(aux2);
    BUFF[_cnt++] = BYTE0(aux3);
    BUFF[_cnt++] = BYTE1(aux3);
    BUFF[_cnt++] = BYTE0(aux4);
    BUFF[_cnt++] = BYTE1(aux4);
    BUFF[_cnt++] = BYTE0(aux5);
    BUFF[_cnt++] = BYTE1(aux5);
    BUFF[_cnt++] = BYTE0(aux6);
    BUFF[_cnt++] = BYTE1(aux6);

    for (uint8_t i = 0; i < BUFF[3] + 4; i++)
    {
        sumcheck += BUFF[i];
        addcheck += sumcheck;
    }
    BUFF[_cnt++] = sumcheck;
    BUFF[_cnt++] = addcheck;

    ano_send_data(BUFF, _cnt);
}

void ANO_DT_Send_RC_ExData(int16_t aux7, int16_t aux8, int16_t aux9, int16_t aux10)
{
    uint8_t BUFF[16];
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
    uint8_t _cnt     = 0;

    BUFF[_cnt++] = 0xAA;
    BUFF[_cnt++] = 0xFF;
    BUFF[_cnt++] = 0x41;
    BUFF[_cnt++] = 0x08;
    BUFF[_cnt++] = BYTE0(aux7);
    BUFF[_cnt++] = BYTE1(aux7);
    BUFF[_cnt++] = BYTE0(aux8);
    BUFF[_cnt++] = BYTE1(aux8);
    BUFF[_cnt++] = BYTE0(aux9);
    BUFF[_cnt++] = BYTE1(aux9);
    BUFF[_cnt++] = BYTE0(aux10);
    BUFF[_cnt++] = BYTE1(aux10);

    for (uint8_t i = 0; i < BUFF[3] + 4; i++)
    {
        sumcheck += BUFF[i];
        addcheck += sumcheck;
    }
    BUFF[_cnt++] = sumcheck;
    BUFF[_cnt++] = addcheck;

    ano_send_data(BUFF, _cnt);
}

#else

void ANO_DT_Send_Euler_Angles(float A, float B, float C) {}
void ANO_DT_Send_IMU_RawData(int16_t Ax, int16_t Ay, int16_t Az, int16_t Gx, int16_t Gy, int16_t Gz, uint8_t SHOCK_STA) {}
void ANO_DT_Send_Att_RawData(float V0, float V1, float V2, float V3) {}
void ANO_DT_Send_Sensor_Data(int16_t mag_x, int16_t mag_y, int16_t mag_z, int32_t alt_bar, int16_t temp, uint8_t bar_sta, uint8_t mag_sta) {}
void ANO_DT_Send_RC_ChData(int16_t rol, int16_t pit, int16_t thr, int16_t yaw, int16_t aux1, int16_t aux2, int16_t aux3, int16_t aux4, int16_t aux5, int16_t aux6) {}
void ANO_DT_Send_RC_ExData(int16_t aux7, int16_t aux8, int16_t aux9, int16_t aux10) {}

#endif
