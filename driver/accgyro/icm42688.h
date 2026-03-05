#ifndef _ICM42688_H_
#define _ICM42688_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include "at32f435_437_spi.h"
#include "at32f435_437_wk_config.h"
#include "drv_gpio.h"
#include "rtthread.h"
/************************ 硬件相关定义 ************************/
// SPI配置
#define SPI1_CS_PIN             GET_PIN(C, 4) // CS引脚
#define ICM42688_MAX_SPI_CLK_HZ 24000000      // SPI最大时钟24MHz

// ICM42688P核心寄存器定义
#define ICM42688P_RA_REG_BANK_SEL 0x76 // Bank选择寄存器
#define ICM42688P_BANK_SELECT0    0x00 // Bank 0
#define ICM42688P_BANK_SELECT1    0x01 // Bank 1
#define ICM42688P_BANK_SELECT2    0x02 // Bank 2

#define ICM42688P_RA_DEVICE_CONFIG   0x11     // 设备配置寄存器
#define DEVICE_CONFIG_SOFT_RESET_BIT (1 << 0) // 软复位位

#define ICM42688P_RA_PWR_MGMT0   0x4E              // 电源管理0
#define PWR_MGMT0_ACCEL_MODE_LN  (3 << 0)          // 加速度计低噪声模式
#define PWR_MGMT0_GYRO_MODE_LN   (3 << 2)          // 陀螺仪低噪声模式
#define PWR_MGMT0_GYRO_ACCEL_OFF (0 << 0 | 0 << 2) // 关闭传感器

#define ICM42688P_RA_GYRO_CONFIG0  0x4F // 陀螺仪配置0
#define ICM42688P_RA_ACCEL_CONFIG0 0x50 // 加速度计配置0

// 抗混叠滤波器(AAF)寄存器
#define ICM42688P_RA_GYRO_CONFIG_STATIC3  0x0C // Bank1 - 陀螺仪AAF
#define ICM42688P_RA_GYRO_CONFIG_STATIC4  0x0D
#define ICM42688P_RA_GYRO_CONFIG_STATIC5  0x0E
#define ICM42688P_RA_ACCEL_CONFIG_STATIC2 0x03 // Bank2 - 加速度计AAF
#define ICM42688P_RA_ACCEL_CONFIG_STATIC3 0x04
#define ICM42688P_RA_ACCEL_CONFIG_STATIC4 0x05

// UI滤波器配置
#define ICM42688P_RA_GYRO_ACCEL_CONFIG0 0x52 // Bank0 - UI滤波器
#define ACCEL_UI_FILT_BW_LOW_LATENCY    (15 << 4)
#define GYRO_UI_FILT_BW_LOW_LATENCY     (15 << 0)

// 数据寄存器
#define ICM42688P_RA_GYRO_DATA_X1  0x25 // 陀螺仪数据起始地址
#define ICM42688P_RA_ACCEL_DATA_X1 0x1F // 加速度计数据起始地址
#define ICM42688P_RA_TEMP_DATA1    0x1E // 温度数据起始地址

// WHO AM I
#define MPU_RA_WHO_AM_I          0x75
#define ICM42688P_WHO_AM_I_CONST 0x47 // ICM42688P的ID

// 修复陀螺仪卡顿相关
#define ICM42688P_INTF_CONFIG1    0x4D
#define INTF_CONFIG1_AFSR_MASK    0xC0
#define INTF_CONFIG1_AFSR_DISABLE 0x40

/************************ 数据结构定义 ************************/

typedef struct accgyroDev_s accgyroDev_t;

/**
 * @brief 统一的惯性传感器设备结构体
 *
 * 包含陀螺仪和加速度计的所有配置和缓存数据，
 * 并提供两种读取接口（只读陀螺仪或只读加速度计）。
 */
struct accgyroDev_s
{
    /* 基础参数 */
    uint8_t gyroRateKHz;
    uint8_t mpuDividerDrops;
    float   scale;
    float   tempScale;
    float   gyroScale;
    float   accScale;
    float   tempZero;
    float   acc_1G;

    /* 寄存器地址 */
    uint8_t gyroDataReg;
    uint8_t accDataReg;
    uint8_t tempDataReg;

    /* 数据缓存 */
    int16_t gyroData[3];
    int16_t accData[3];
    float   tempData;
    float   accScaled[3];

    /* 函数指针 */
    bool (*init)(struct accgyroDev_s *);
    bool (*readGyro)(struct accgyroDev_s *);
    bool (*readAcc)(struct accgyroDev_s *);
};

/************************ 滤波器配置 ************************/
// 输出数据率(ODR)配置
typedef enum
{
    ODR_8K = 0,
    ODR_4K,
    ODR_2K,
    ODR_1K
} odrConfig_e;

// 抗混叠滤波器(AAF)配置
typedef struct
{
    uint8_t  delt;
    uint16_t deltSqr;
    uint8_t  bitshift;
} aafConfig_t;

extern accgyroDev_t g_icm_accgyro; // 惯导设备实例

bool accgyro_init(accgyroDev_t *dev);

#endif
