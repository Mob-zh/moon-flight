#include "ano_data.h"
#include "bz121.h"
#include <rtthread.h>

// 数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp) (*((char *)(&dwTemp)))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

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
    BUFF[_cnt++] = 7;    // 数据长度
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
    BUFF[_cnt++] = 13;
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
    BUFF[_cnt++] = 9;
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
    BUFF[_cnt++] = 14;
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
    BUFF[_cnt++] = 20;
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
    BUFF[_cnt++] = 8;
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

void ANO_DT_Send_GPS_Data(
    uint8_t fix_sta, // FIX_STA: 定位状态
    uint8_t s_num,   // S_NUM: 卫星数量
    int32_t lng,     // LNG: 经度 (deg * 1e7)
    int32_t lat,     // LAT: 纬度 (deg * 1e7)
    int32_t alt_gps, // ALT_GPS: GPS高度 (mm)
    int16_t n_spe,   // N_SPE: 北向速度 (cm/s)
    int16_t e_spe,   // E_SPE: 东向速度 (cm/s)
    int16_t d_spe,   // D_SPE: 下向速度 (cm/s)
    uint8_t pdop,    // PDOP: 定位精度 (0-200)
    uint8_t sacc,    // SACC: 速度精度 (mm/s / 100)
    uint8_t vacc)    // VACC: 高度精度 (mm / 100)
{
    uint8_t BUFF[32];
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
    uint8_t _cnt     = 0;

    // 帧头
    BUFF[_cnt++] = 0xAA; // HEAD
    BUFF[_cnt++] = 0xFF; // D_ADDR
    BUFF[_cnt++] = 0x30; // ID
    BUFF[_cnt++] = 23;   // LEN

    // DATA (23字节)
    BUFF[_cnt++] = fix_sta;        // FIX_STA
    BUFF[_cnt++] = s_num;          // S_NUM
    BUFF[_cnt++] = BYTE0(lng);     // LNG byte0
    BUFF[_cnt++] = BYTE1(lng);     // LNG byte1
    BUFF[_cnt++] = BYTE2(lng);     // LNG byte2
    BUFF[_cnt++] = BYTE3(lng);     // LNG byte3
    BUFF[_cnt++] = BYTE0(lat);     // LAT byte0
    BUFF[_cnt++] = BYTE1(lat);     // LAT byte1
    BUFF[_cnt++] = BYTE2(lat);     // LAT byte2
    BUFF[_cnt++] = BYTE3(lat);     // LAT byte3
    BUFF[_cnt++] = BYTE0(alt_gps); // ALT_GPS byte0
    BUFF[_cnt++] = BYTE1(alt_gps); // ALT_GPS byte1
    BUFF[_cnt++] = BYTE2(alt_gps); // ALT_GPS byte2
    BUFF[_cnt++] = BYTE3(alt_gps); // ALT_GPS byte3
    BUFF[_cnt++] = BYTE0(n_spe);   // N_SPE byte0
    BUFF[_cnt++] = BYTE1(n_spe);   // N_SPE byte1
    BUFF[_cnt++] = BYTE0(e_spe);   // E_SPE byte0
    BUFF[_cnt++] = BYTE1(e_spe);   // E_SPE byte1
    BUFF[_cnt++] = BYTE0(d_spe);   // D_SPE byte0
    BUFF[_cnt++] = BYTE1(d_spe);   // D_SPE byte1
    BUFF[_cnt++] = pdop;           // PDOP
    BUFF[_cnt++] = sacc;           // SACC
    BUFF[_cnt++] = vacc;           // VACC

    // 校验
    for (uint8_t i = 0; i < _cnt; i++)
    {
        sumcheck += BUFF[i];
        addcheck += sumcheck;
    }
    BUFF[_cnt++] = sumcheck; // SC
    BUFF[_cnt++] = addcheck; // AC

    ano_send_data(BUFF, _cnt);
}

// PID参数发送函数 (0xF1)
// 发送角速度环的Kp、Ki、Kd参数和角速度上限
void ANO_DT_Send_PID_Params(float rate_kp_roll, float rate_kp_pitch, float rate_kp_yaw,
                            float rate_ki_roll, float rate_ki_pitch, float rate_ki_yaw,
                            float rate_kd_roll, float rate_kd_pitch, float rate_kd_yaw,
                            float rate_limit)
{
#if ANO_SEND_PID
    uint8_t BUFF[32];
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
    uint8_t _cnt     = 0;

    // 将float转为int16乘以100发送（保持2位小数精度）
    int16_t kp_r  = (int16_t)(rate_kp_roll * 100);
    int16_t kp_p  = (int16_t)(rate_kp_pitch * 100);
    int16_t kp_y  = (int16_t)(rate_kp_yaw * 100);
    int16_t ki_r  = (int16_t)(rate_ki_roll * 100);
    int16_t ki_p  = (int16_t)(rate_ki_pitch * 100);
    int16_t ki_y  = (int16_t)(rate_ki_yaw * 100);
    int16_t kd_r  = (int16_t)(rate_kd_roll * 100);
    int16_t kd_p  = (int16_t)(rate_kd_pitch * 100);
    int16_t kd_y  = (int16_t)(rate_kd_yaw * 100);
    int16_t limit = (int16_t)(rate_limit); // 角速度上限

    BUFF[_cnt++] = 0xAA; // 帧头
    BUFF[_cnt++] = 0xFF; // 目标地址
    BUFF[_cnt++] = 0xF1; // 功能码：PID参数
    BUFF[_cnt++] = 20;   // 数据长度：22字节

    // Kp: Roll, Pitch, Yaw
    BUFF[_cnt++] = BYTE0(kp_r);
    BUFF[_cnt++] = BYTE1(kp_r);
    BUFF[_cnt++] = BYTE0(kp_p);
    BUFF[_cnt++] = BYTE1(kp_p);
    BUFF[_cnt++] = BYTE0(kp_y);
    BUFF[_cnt++] = BYTE1(kp_y);

    // Ki: Roll, Pitch, Yaw
    BUFF[_cnt++] = BYTE0(ki_r);
    BUFF[_cnt++] = BYTE1(ki_r);
    BUFF[_cnt++] = BYTE0(ki_p);
    BUFF[_cnt++] = BYTE1(ki_p);
    BUFF[_cnt++] = BYTE0(ki_y);
    BUFF[_cnt++] = BYTE1(ki_y);

    // Kd: Roll, Pitch, Yaw
    BUFF[_cnt++] = BYTE0(kd_r);
    BUFF[_cnt++] = BYTE1(kd_r);
    BUFF[_cnt++] = BYTE0(kd_p);
    BUFF[_cnt++] = BYTE1(kd_p);
    BUFF[_cnt++] = BYTE0(kd_y);
    BUFF[_cnt++] = BYTE1(kd_y);

    // 角速度上限
    BUFF[_cnt++] = BYTE0(limit);
    BUFF[_cnt++] = BYTE1(limit);

    // 校验和
    for (uint8_t i = 0; i < BUFF[3] + 4; i++)
    {
        sumcheck += BUFF[i];
        addcheck += sumcheck;
    }
    BUFF[_cnt++] = sumcheck;
    BUFF[_cnt++] = addcheck;

    ano_send_data(BUFF, _cnt);
#endif
}

// 角度环PID参数发送函数 (0xF2)
void ANO_DT_Send_Angle_PID(float angle_kp_roll, float angle_kp_pitch, float angle_kp_yaw,
                           float angle_ki_roll, float angle_ki_pitch, float angle_ki_yaw,
                           float angle_kd_roll, float angle_kd_pitch, float angle_kd_yaw,
                           float angle_limit)
{
#if ANO_SEND_PID
    uint8_t BUFF[32];
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
    uint8_t _cnt     = 0;

    int16_t kp_r  = (int16_t)(angle_kp_roll * 100);
    int16_t kp_p  = (int16_t)(angle_kp_pitch * 100);
    int16_t kp_y  = (int16_t)(angle_kp_yaw * 100);
    int16_t ki_r  = (int16_t)(angle_ki_roll * 100);
    int16_t ki_p  = (int16_t)(angle_ki_pitch * 100);
    int16_t ki_y  = (int16_t)(angle_ki_yaw * 100);
    int16_t kd_r  = (int16_t)(angle_kd_roll * 100);
    int16_t kd_p  = (int16_t)(angle_kd_pitch * 100);
    int16_t kd_y  = (int16_t)(angle_kd_yaw * 100);
    int16_t limit = (int16_t)(angle_limit);

    BUFF[_cnt++] = 0xAA;
    BUFF[_cnt++] = 0xFF;
    BUFF[_cnt++] = 0xF2; // 功能码：角度环PID
    BUFF[_cnt++] = 20;

    BUFF[_cnt++] = BYTE0(kp_r);
    BUFF[_cnt++] = BYTE1(kp_r);
    BUFF[_cnt++] = BYTE0(kp_p);
    BUFF[_cnt++] = BYTE1(kp_p);
    BUFF[_cnt++] = BYTE0(kp_y);
    BUFF[_cnt++] = BYTE1(kp_y);
    BUFF[_cnt++] = BYTE0(ki_r);
    BUFF[_cnt++] = BYTE1(ki_r);
    BUFF[_cnt++] = BYTE0(ki_p);
    BUFF[_cnt++] = BYTE1(ki_p);
    BUFF[_cnt++] = BYTE0(ki_y);
    BUFF[_cnt++] = BYTE1(ki_y);
    BUFF[_cnt++] = BYTE0(kd_r);
    BUFF[_cnt++] = BYTE1(kd_r);
    BUFF[_cnt++] = BYTE0(kd_p);
    BUFF[_cnt++] = BYTE1(kd_p);
    BUFF[_cnt++] = BYTE0(kd_y);
    BUFF[_cnt++] = BYTE1(kd_y);
    BUFF[_cnt++] = BYTE0(limit);
    BUFF[_cnt++] = BYTE1(limit);

    for (uint8_t i = 0; i < BUFF[3] + 4; i++)
    {
        sumcheck += BUFF[i];
        addcheck += sumcheck;
    }
    BUFF[_cnt++] = sumcheck;
    BUFF[_cnt++] = addcheck;

    ano_send_data(BUFF, _cnt);
#endif
}

// 位置环PID参数发送函数 (0xF3)
void ANO_DT_Send_Pos_PID(float pos_kp_n, float pos_kp_e, float pos_ki_n, float pos_ki_e,
                         float pos_kd_n, float pos_kd_e, float alt_kp, float alt_ki, float alt_kd,
                         float alt_limit)
{
#if ANO_SEND_PID
    uint8_t BUFF[32];
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
    uint8_t _cnt     = 0;

    int16_t p_kp_n = (int16_t)(pos_kp_n * 100);
    int16_t p_kp_e = (int16_t)(pos_kp_e * 100);
    int16_t p_ki_n = (int16_t)(pos_ki_n * 100);
    int16_t p_ki_e = (int16_t)(pos_ki_e * 100);
    int16_t p_kd_n = (int16_t)(pos_kd_n * 100);
    int16_t p_kd_e = (int16_t)(pos_kd_e * 100);
    int16_t a_kp   = (int16_t)(alt_kp * 100);
    int16_t a_ki   = (int16_t)(alt_ki * 100);
    int16_t a_kd   = (int16_t)(alt_kd * 100);
    int16_t limit  = (int16_t)(alt_limit);

    BUFF[_cnt++] = 0xAA;
    BUFF[_cnt++] = 0xFF;
    BUFF[_cnt++] = 0xF3; // 功能码：位置环PID
    BUFF[_cnt++] = 20;

    BUFF[_cnt++] = BYTE0(p_kp_n);
    BUFF[_cnt++] = BYTE1(p_kp_n);
    BUFF[_cnt++] = BYTE0(p_kp_e);
    BUFF[_cnt++] = BYTE1(p_kp_e);
    BUFF[_cnt++] = BYTE0(p_ki_n);
    BUFF[_cnt++] = BYTE1(p_ki_n);
    BUFF[_cnt++] = BYTE0(p_ki_e);
    BUFF[_cnt++] = BYTE1(p_ki_e);
    BUFF[_cnt++] = BYTE0(p_kd_n);
    BUFF[_cnt++] = BYTE1(p_kd_n);
    BUFF[_cnt++] = BYTE0(p_kd_e);
    BUFF[_cnt++] = BYTE1(p_kd_e);
    BUFF[_cnt++] = BYTE0(a_kp);
    BUFF[_cnt++] = BYTE1(a_kp);
    BUFF[_cnt++] = BYTE0(a_ki);
    BUFF[_cnt++] = BYTE1(a_ki);
    BUFF[_cnt++] = BYTE0(a_kd);
    BUFF[_cnt++] = BYTE1(a_kd);
    BUFF[_cnt++] = BYTE0(limit);
    BUFF[_cnt++] = BYTE1(limit);

    for (uint8_t i = 0; i < BUFF[3] + 4; i++)
    {
        sumcheck += BUFF[i];
        addcheck += sumcheck;
    }
    BUFF[_cnt++] = sumcheck;
    BUFF[_cnt++] = addcheck;

    ano_send_data(BUFF, _cnt);
#endif
}

#else

void ANO_DT_Send_Euler_Angles(float A, float B, float C) {}
void ANO_DT_Send_IMU_RawData(int16_t Ax, int16_t Ay, int16_t Az, int16_t Gx, int16_t Gy, int16_t Gz, uint8_t SHOCK_STA) {}
void ANO_DT_Send_Att_RawData(float V0, float V1, float V2, float V3) {}
void ANO_DT_Send_Sensor_Data(int16_t mag_x, int16_t mag_y, int16_t mag_z, int32_t alt_bar, int16_t temp, uint8_t bar_sta, uint8_t mag_sta) {}
void ANO_DT_Send_RC_ChData(int16_t rol, int16_t pit, int16_t thr, int16_t yaw, int16_t aux1, int16_t aux2, int16_t aux3, int16_t aux4, int16_t aux5, int16_t aux6) {}
void ANO_DT_Send_RC_ExData(int16_t aux7, int16_t aux8, int16_t aux9, int16_t aux10) {}
void ANO_DT_Send_GPS_Data(uint8_t fix_sta, uint8_t s_num,
                          int32_t lng, int32_t lat, int32_t alt_gps,
                          int16_t n_spe, int16_t e_spe, int16_t d_spe,
                          uint8_t pdop, uint8_t sacc, uint8_t vacc) {}

void ANO_DT_Send_PID_Params(float rate_kp_roll, float rate_kp_pitch, float rate_kp_yaw,
                            float rate_ki_roll, float rate_ki_pitch, float rate_ki_yaw,
                            float rate_kd_roll, float rate_kd_pitch, float rate_kd_yaw,
                            float rate_limit) {}

void ANO_DT_Send_Angle_PID(float angle_kp_roll, float angle_kp_pitch, float angle_kp_yaw,
                           float angle_ki_roll, float angle_ki_pitch, float angle_ki_yaw,
                           float angle_kd_roll, float angle_kd_pitch, float angle_kd_yaw,
                           float angle_limit) {}

void ANO_DT_Send_Pos_PID(float pos_kp_n, float pos_kp_e, float pos_ki_n, float pos_ki_e,
                         float pos_kd_n, float pos_kd_e, float alt_kp, float alt_ki, float alt_kd,
                         float alt_limit) {}

#endif
