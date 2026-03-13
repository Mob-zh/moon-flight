#include "spi_dev.h"

/************************ SPI底层操作 ************************/
/**
 * @brief SPI写寄存器
 * @param reg: 寄存器地址
 * @param data: 写入数据
 */
void spi_write_reg(struct rt_spi_device *spi_dev, uint8_t reg, uint8_t data)
{
    uint8_t tx_buf[2] = {0};
    tx_buf[0]         = reg & 0x7F; // 写操作：最高位0
    tx_buf[1]         = data;

    // SPI发送（自动控制CS，底层驱动自动用DMA）
    rt_spi_send(spi_dev, tx_buf, sizeof(tx_buf));
}

/**
 * @brief SPI读寄存器
 * @param reg: 寄存器地址
 * @return 读取到的数据
 */
uint8_t spi_read_reg(struct rt_spi_device *spi_dev, uint8_t reg)
{
    uint8_t tx_buf = reg | 0x80; // 读操作：最高位1
    uint8_t rx_buf = 0;

    // 先发送地址（DMA），再接收数据（DMA）
    rt_spi_send_then_recv(spi_dev, &tx_buf, 1, &rx_buf, 1);

    return rx_buf;
}

/**
 * @brief 连续读取多个寄存器
 * @param reg: 起始寄存器地址
 * @param buf: 数据缓冲区
 * @param len: 读取长度
 */
void spi_read_regs(struct rt_spi_device *spi_dev, uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (len == 0 || buf == NULL)
        return;

    uint8_t tx_buf = reg | 0x80; // 读操作：最高位1

    // 批量读取（DMA模式，高效传输）
    rt_spi_send_then_recv(spi_dev, &tx_buf, 1, buf, len);
}