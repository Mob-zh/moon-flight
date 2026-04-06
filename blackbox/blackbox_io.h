/**
 * @file blackbox_io.h
 * @brief Blackbox IO层 - 串口输出接口
 */
#ifndef _BLACKBOX_IO_H_
#define _BLACKBOX_IO_H_

#include <stdint.h>
#include <stdbool.h>

// 头部预算（用于控制每次迭代的写入量）
extern int32_t blackboxHeaderBudget;

/**
 * @brief 初始化IO层（内部使用）
 */
void bb_init_io(void);

/**
 * @brief 初始化Blackbox（串口）
 */
void bb_init(void);

/**
 * @brief 打开Blackbox日志
 */
void bb_open(void);

/**
 * @brief 关闭Blackbox日志
 */
void bb_close(void);

/**
 * @brief 刷新缓冲区
 */
void bb_flush(void);

/**
 * @brief 写入单字节
 * @param value 要写入的字节
 */
void bb_write(uint8_t value);

/**
 * @brief 写入字符串
 * @param s 字符串指针
 */
void bb_write_string(const char *s);

/**
 * @brief 格式化写入头部行
 * @param name 字段名
 * @param fmt 格式字符串
 */
void bb_printf_header(const char *name, const char *fmt, ...);

/**
 * @brief 检查Blackbox是否已打开
 * @return true 已打开
 */
bool bb_is_open(void);

/**
 * @brief 获取日志编号
 * @return 日志编号
 */
int32_t bb_get_log_number(void);

#endif /* _BLACKBOX_IO_H_ */
