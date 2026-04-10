/**
 * @file blackbox_fatfs.h
 * @brief Blackbox FatFS 文件系统存储接口
 */

#ifndef _BLACKBOX_FATFS_H_
#define _BLACKBOX_FATFS_H_

#include <stdbool.h>
#include <stdint.h>

// ==================== 函数声明 ====================

/**
 * @brief 初始化 FatFS 存储
 */
bool bb_fatfs_init(void);

/**
 * @brief 打开日志文件
 */
bool bb_fatfs_open(void);

/**
 * @brief 关闭日志文件
 */
void bb_fatfs_close(void);

/**
 * @brief 写入数据
 */
bool bb_fatfs_write(const uint8_t *data, uint32_t len);

/**
 * @brief 刷新缓冲区
 */
bool bb_fatfs_flush(void);

/**
 * @brief 检查是否正在记录
 */
bool bb_fatfs_is_open(void);

/**
 * @brief 获取日志编号
 */
int32_t bb_fatfs_get_log_number(void);

/**
 * @brief 获取 FatFS 存储操作函数集
 */
void *bb_fatfs_get_ops(void);

#endif /* _BLACKBOX_FATFS_H_ */