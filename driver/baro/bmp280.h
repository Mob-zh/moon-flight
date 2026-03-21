#ifndef _BMP280_H_
#define _BMP280_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include "drv_gpio.h"
#include "drv_spi.h"
#include "rtthread.h"

/************************ 硬件相关定义 ************************/
// SPI配置
#define SPI2_CS_PIN GET_PIN(C, 8) // CS引脚

/************************ BMP280寄存器定义 ************************/
// 基础寄存器
#define BMP280_CHIPID_REG   0xD0 // 芯片ID寄存器
#define BMP280_RESET_REG    0xE0 // 复位寄存器
#define BMP280_STATUS_REG   0xF3 // 状态寄存器
#define BMP280_CTRLMEAS_REG 0xF4 // 控制测量寄存器
#define BMP280_CONFIG_REG   0xF5 // 配置寄存器

// 校准参数寄存器（出厂校准系数）
#define BMP280_DIG_T1_LSB_REG 0x88
#define BMP280_DIG_T2_LSB_REG 0x8A
#define BMP280_DIG_T3_LSB_REG 0x8C
#define BMP280_DIG_P1_LSB_REG 0x8E
#define BMP280_DIG_P2_LSB_REG 0x90
#define BMP280_DIG_P3_LSB_REG 0x92
#define BMP280_DIG_P4_LSB_REG 0x94
#define BMP280_DIG_P5_LSB_REG 0x96
#define BMP280_DIG_P6_LSB_REG 0x98
#define BMP280_DIG_P7_LSB_REG 0x9A
#define BMP280_DIG_P8_LSB_REG 0x9C
#define BMP280_DIG_P9_LSB_REG 0x9E

// 数据输出寄存器
#define BMP280_TEMPERATURE_MSB_REG 0xFA // 温度MSB
#define BMP280_PRESSURE_MSB_REG    0xF7 // 压力MSB

/************************ 常量定义 ************************/
#define BMP280_DUMMY_BYTE  0xFF // SPI虚拟字节
#define BMP280_RESET_VALUE 0xB6 // 复位寄存器写入值
#define BMP280_CHIP_ID     0x58 // BMP280默认ID值

/************************ 数据结构定义 ************************/

typedef struct baroDev_s baroDev_t;

/**
 * @brief 气压计设备结构体
 */
struct baroDev_s
{
    // 基础数据
    uint32_t pressure; // 气压值 (Pa)
    uint32_t temp;     // 温度值 (0.01°C)
    int32_t  altitude; // 海拔高度 (cm)

    uint8_t ID; // 芯片ID

    // 函数指针
    bool (*init)(baroDev_t *);
    bool (*read_press)(baroDev_t *);
    int32_t (*get_altitude)(baroDev_t *, uint32_t);
};

extern baroDev_t g_bmp280_baro;

/************************ 函数声明 ************************/

/************************ 函数声明 ************************/

/**
 * @brief  气压计初始化
 * @param  baro: 气压计设备结构体
 * @retval true-成功, false-失败
 */
bool baro_init(baroDev_t *baro);

#endif /* _BMP280_H_ */
