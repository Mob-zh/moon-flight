#include "ano_data.h"
#include <rtthread.h>

// 数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp) (*((char *)(&dwTemp)))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

uint8_t BUFF[64];

void ANO_DT_Send_Euler_Angles(float A, float B, float C)
{
    int     i;
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
    // 发送俯仰角
    BUFF[_cnt++] = BYTE0(A_int); // 数据内容,小段模式，低位在前
    BUFF[_cnt++] = BYTE1(A_int); // 需要将字节进行拆分，调用上面的宏定义即可。
    // 发送横滚角
    BUFF[_cnt++] = BYTE0(B_int);
    BUFF[_cnt++] = BYTE1(B_int);
    // 发送偏航角
    BUFF[_cnt++] = BYTE0(C_int);
    BUFF[_cnt++] = BYTE1(C_int);

    BUFF[_cnt++] = 0x00;

    // SC和AC的校验直接抄最上面上面简介的即可
    for (i = 0; i < BUFF[3] + 4; i++)
    {
        sumcheck += BUFF[i];
        addcheck += sumcheck;
    }
    BUFF[_cnt++] = sumcheck;
    BUFF[_cnt++] = addcheck;

    for (i = 0; i < _cnt; i++)
    {
        rt_kprintf("%c", BUFF[i]);
    }
}

void ANO_DT_Send_IMU_RawData(int16_t Ax, int16_t Ay, int16_t Az, int16_t Gx, int16_t Gy, int16_t Gz, uint8_t SHOCK_STA)
{
    int     i;
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
    uint8_t _cnt     = 0;

    BUFF[_cnt++] = 0xAA; // 帧头
    BUFF[_cnt++] = 0xFF; // 目标地址
    BUFF[_cnt++] = 0X01; // 功能码
    BUFF[_cnt++] = 0x0D; // 数据长度
    // 发送加速度
    BUFF[_cnt++] = BYTE0(Ax); // 数据内容,小段模式，低位在前,需要将字节进行拆分，调用上面的宏定义即可。
    BUFF[_cnt++] = BYTE1(Ax);
    BUFF[_cnt++] = BYTE0(Ay);
    BUFF[_cnt++] = BYTE1(Ay);
    BUFF[_cnt++] = BYTE0(Az);
    BUFF[_cnt++] = BYTE1(Az);
    // 发送角速度
    BUFF[_cnt++] = BYTE0(Gx); // 数据内容,小段模式，低位在前,需要将字节进行拆分，调用上面的宏定义即可。
    BUFF[_cnt++] = BYTE1(Gx);
    BUFF[_cnt++] = BYTE0(Gy);
    BUFF[_cnt++] = BYTE1(Gy);
    BUFF[_cnt++] = BYTE0(Gz);
    BUFF[_cnt++] = BYTE1(Gz);
    // 发送震动状态
    BUFF[_cnt++] = SHOCK_STA;

    // SC和AC的校验直接抄最上面上面简介的即可
    for (i = 0; i < BUFF[3] + 4; i++)
    {
        sumcheck += BUFF[i];
        addcheck += sumcheck;
    }
    BUFF[_cnt++] = sumcheck;
    BUFF[_cnt++] = addcheck;

    for (i = 0; i < _cnt; i++)
    {
        rt_kprintf("%c", BUFF[i]);
    }
}

// 发送四元数数据给上位机
void ANO_DT_Send_Att_RawData(float V0, float V1, float V2, float V3)
{
    int     i;
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
    uint8_t _cnt     = 0;

    // 先转换为int16_t再取字节，避免浮点数问题
    int16_t V0_int = (int16_t)(V0 * 10000);
    int16_t V1_int = (int16_t)(V1 * 10000);
    int16_t V2_int = (int16_t)(V2 * 10000);
    int16_t V3_int = (int16_t)(V3 * 10000);

    BUFF[_cnt++] = 0xAA; // 帧头
    BUFF[_cnt++] = 0xFF; // 目标地址
    BUFF[_cnt++] = 0X04; // 功能码
    BUFF[_cnt++] = 0x09; // 数据长度
    // 发送四元数数据
    BUFF[_cnt++] = BYTE0(V0_int); // 数据内容,小段模式，低位在前,需要将字节进行拆分，调用上面的宏定义即可。
    BUFF[_cnt++] = BYTE1(V0_int);

    BUFF[_cnt++] = BYTE0(V1_int);
    BUFF[_cnt++] = BYTE1(V1_int);

    BUFF[_cnt++] = BYTE0(V2_int);
    BUFF[_cnt++] = BYTE1(V2_int);

    BUFF[_cnt++] = BYTE0(V3_int); // 数据内容,小段模式，低位在前,需要将字节进行拆分，调用上面的宏定义即可。
    BUFF[_cnt++] = BYTE1(V3_int);

    BUFF[_cnt++] = 0x00;

    // SC和AC的校验直接抄最上面上面简介的即可
    for (i = 0; i < BUFF[3] + 4; i++)
    {
        sumcheck += BUFF[i];
        addcheck += sumcheck;
    }
    BUFF[_cnt++] = sumcheck;
    BUFF[_cnt++] = addcheck;

    for (i = 0; i < _cnt; i++)
    {
        rt_kprintf("%c", BUFF[i]);
    }
}

// 发送传感器数据（罗盘、气压、温度）ID=0x02
void ANO_DT_Send_Sensor_Data(int16_t mag_x, int16_t mag_y, int16_t mag_z, int32_t alt_bar, int16_t temp, uint8_t bar_sta, uint8_t mag_sta)
{
    int     i;
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
    uint8_t _cnt     = 0;

    BUFF[_cnt++] = 0xAA;  // 帧头
    BUFF[_cnt++] = 0xFF;  // 目标地址
    BUFF[_cnt++] = 0x02;  // 功能码
    BUFF[_cnt++] = 0x0E;  // 数据长度 14

    // MAG_X (int16)
    BUFF[_cnt++] = BYTE0(mag_x);
    BUFF[_cnt++] = BYTE1(mag_x);
    // MAG_Y (int16)
    BUFF[_cnt++] = BYTE0(mag_y);
    BUFF[_cnt++] = BYTE1(mag_y);
    // MAG_Z (int16)
    BUFF[_cnt++] = BYTE0(mag_z);
    BUFF[_cnt++] = BYTE1(mag_z);
    // ALT_BAR (int32, 单位cm)
    BUFF[_cnt++] = BYTE0(alt_bar);
    BUFF[_cnt++] = BYTE1(alt_bar);
    BUFF[_cnt++] = BYTE2(alt_bar);
    BUFF[_cnt++] = BYTE3(alt_bar);
    // TMP (int16, 0.1°C)
    BUFF[_cnt++] = BYTE0(temp);
    BUFF[_cnt++] = BYTE1(temp);
    // BAR_STA (uint8)
    BUFF[_cnt++] = bar_sta;
    // MAG_STA (uint8)
    BUFF[_cnt++] = mag_sta;

    // 校验和
    for (i = 0; i < BUFF[3] + 4; i++)
    {
        sumcheck += BUFF[i];
        addcheck += sumcheck;
    }
    BUFF[_cnt++] = sumcheck;
    BUFF[_cnt++] = addcheck;

    for (i = 0; i < _cnt; i++)
    {
        rt_kprintf("%c", BUFF[i]);
    }
}

// 发送遥控器通道数据 ID=0x40 (通道1-10: ROL, PIT, THR, YAW, AUX1-6)
void ANO_DT_Send_RC_ChData(int16_t rol, int16_t pit, int16_t thr, int16_t yaw,
                            int16_t aux1, int16_t aux2, int16_t aux3, int16_t aux4,
                            int16_t aux5, int16_t aux6)
{
    int     i;
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
    uint8_t _cnt     = 0;

    BUFF[_cnt++] = 0xAA;  // 帧头
    BUFF[_cnt++] = 0xFF;  // 目标地址
    BUFF[_cnt++] = 0x40;  // 功能码 ID
    BUFF[_cnt++] = 0x14;  // 数据长度 20字节 (10通道 × 2字节)

    // ROL (int16)
    BUFF[_cnt++] = BYTE0(rol);
    BUFF[_cnt++] = BYTE1(rol);
    // PIT (int16)
    BUFF[_cnt++] = BYTE0(pit);
    BUFF[_cnt++] = BYTE1(pit);
    // THR (int16)
    BUFF[_cnt++] = BYTE0(thr);
    BUFF[_cnt++] = BYTE1(thr);
    // YAW (int16)
    BUFF[_cnt++] = BYTE0(yaw);
    BUFF[_cnt++] = BYTE1(yaw);
    // AUX1 (int16)
    BUFF[_cnt++] = BYTE0(aux1);
    BUFF[_cnt++] = BYTE1(aux1);
    // AUX2 (int16)
    BUFF[_cnt++] = BYTE0(aux2);
    BUFF[_cnt++] = BYTE1(aux2);
    // AUX3 (int16)
    BUFF[_cnt++] = BYTE0(aux3);
    BUFF[_cnt++] = BYTE1(aux3);
    // AUX4 (int16)
    BUFF[_cnt++] = BYTE0(aux4);
    BUFF[_cnt++] = BYTE1(aux4);
    // AUX5 (int16)
    BUFF[_cnt++] = BYTE0(aux5);
    BUFF[_cnt++] = BYTE1(aux5);
    // AUX6 (int16)
    BUFF[_cnt++] = BYTE0(aux6);
    BUFF[_cnt++] = BYTE1(aux6);

    // 校验和
    for (i = 0; i < BUFF[3] + 4; i++)
    {
        sumcheck += BUFF[i];
        addcheck += sumcheck;
    }
    BUFF[_cnt++] = sumcheck;
    BUFF[_cnt++] = addcheck;

    for (i = 0; i < _cnt; i++)
    {
        rt_kprintf("%c", BUFF[i]);
    }
}

// 发送遥控器额外通道数据 ID=0x41 (通道11-14: AUX7-10)
void ANO_DT_Send_RC_ExData(int16_t aux7, int16_t aux8, int16_t aux9, int16_t aux10)
{
    int     i;
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
    uint8_t _cnt     = 0;

    BUFF[_cnt++] = 0xAA;  // 帧头
    BUFF[_cnt++] = 0xFF;  // 目标地址
    BUFF[_cnt++] = 0x41;  // 功能码 ID
    BUFF[_cnt++] = 0x08;  // 数据长度 8字节 (4通道 × 2字节)

    // AUX7 (int16)
    BUFF[_cnt++] = BYTE0(aux7);
    BUFF[_cnt++] = BYTE1(aux7);
    // AUX8 (int16)
    BUFF[_cnt++] = BYTE0(aux8);
    BUFF[_cnt++] = BYTE1(aux8);
    // AUX9 (int16)
    BUFF[_cnt++] = BYTE0(aux9);
    BUFF[_cnt++] = BYTE1(aux9);
    // AUX10 (int16)
    BUFF[_cnt++] = BYTE0(aux10);
    BUFF[_cnt++] = BYTE1(aux10);

    // 校验和
    for (i = 0; i < BUFF[3] + 4; i++)
    {
        sumcheck += BUFF[i];
        addcheck += sumcheck;
    }
    BUFF[_cnt++] = sumcheck;
    BUFF[_cnt++] = addcheck;

    for (i = 0; i < _cnt; i++)
    {
        rt_kprintf("%c", BUFF[i]);
    }
}