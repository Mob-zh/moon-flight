/**
 * @file blackbox_sdio.h
 * @brief Blackbox SD卡存储实现
 */
#ifndef _BLACKBOX_SDIO_H_
#define _BLACKBOX_SDIO_H_

#include <stdbool.h>
#include <stdint.h>

// ==================== 配置 ====================
#define BB_SECTOR_SIZE 512  // SD卡扇区大小
#define BB_BUFFER_SIZE 2048 // 写入缓冲区大小

// ==================== 函数声明 ====================

/**
 * @brief 初始化SD卡存储
 */
bool bb_sdio_init(void);

/**
 * @brief 打开日志文件
 */
bool bb_sdio_open(void);

/**
 * @brief 关闭日志文件
 */
void bb_sdio_close(void);

/**
 * @brief 写入数据
 * @param data 数据指针
 * @param len 数据长度
 */
bool bb_sdio_write(const uint8_t *data, uint32_t len);

/**
 * @brief 刷新缓冲区
 */
bool bb_sdio_flush(void);

/**
 * @brief 检查是否正在记录
 */
bool bb_sdio_is_open(void);

/**
 * @brief 获取日志编号
 */
int32_t bb_sdio_get_log_number(void);

/**
 * @brief 获取SDIO存储操作函数集
 */
extern void *bb_sdio_get_ops(void);

#endif /* _BLACKBOX_SDIO_H_ */