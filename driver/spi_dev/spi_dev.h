#ifndef _SPI_DEV_H_
#define _SPI_DEV_H_

#include <rtdevice.h>
#include <rtthread.h>
/************************ SPI底层操作 ************************/
/**
 * @brief SPI写寄存器
 * @param reg: 寄存器地址
 * @param data: 写入数据
 */
void spi_write_reg(struct rt_spi_device *spi_dev, uint8_t reg, uint8_t data);

/**
 * @brief SPI读寄存器
 * @param reg: 寄存器地址
 * @return 读取到的数据
 */
uint8_t spi_read_reg(struct rt_spi_device *spi_dev, uint8_t reg);

/**
 * @brief 连续读取多个寄存器
 * @param reg: 起始寄存器地址
 * @param buf: 数据缓冲区
 * @param len: 读取长度
 */
void spi_read_regs(struct rt_spi_device *spi_dev, uint8_t reg, uint8_t *buf, uint8_t len);

#endif
