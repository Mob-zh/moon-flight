/**
 * @file blackbox_io.c
 * @brief Blackbox IO层实现 - 使用RT-Thread串口输出
 */
#include "blackbox_io.h"
#include <rtthread.h>
#include <stdarg.h>
#include <string.h>

// Blackbox状态
static volatile int bb_log_number = 0;
static volatile int bbopened = 0;

int32_t blackboxHeaderBudget = 0;

/**
 * @brief 初始化IO层（内部使用）
 */
void bb_init_io(void)
{
    bbopened = 0;
    bb_log_number = 0;
    blackboxHeaderBudget = 64;
}

/**
 * @brief 初始化Blackbox（串口）
 */
void bb_init(void)
{
    bb_init_io();
}

/**
 * @brief 打开Blackbox日志
 */
void bb_open(void)
{
    bbopened = 1;
    bb_log_number++;
    blackboxHeaderBudget = 64;
}

/**
 * @brief 关闭Blackbox日志
 */
void bb_close(void)
{
    bbopened = 0;
}

/**
 * @brief 刷新缓冲区
 */
void bb_flush(void)
{
    // RT-Thread的rt_kprintf是同步的，无需额外刷新
}

/**
 * @brief 写入单字节
 * @param value 要写入的字节
 */
void bb_write(uint8_t value)
{
    if (!bbopened) return;
    rt_kprintf("%c", value);
}

/**
 * @brief 写入字符串
 * @param s 字符串指针
 */
void bb_write_string(const char *s)
{
    if (!bbopened || !s) return;
    while (*s) {
        rt_kprintf("%c", *s);
        s++;
    }
}

/**
 * @brief 格式化写入头部行
 * @param name 字段名
 * @param fmt 格式字符串
 */
void bb_printf_header(const char *name, const char *fmt, ...)
{
    if (!bbopened) return;

    // 写入 "H name:"
    rt_kprintf("H %s:", name);

    // 格式化并写入
    char buffer[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    rt_kprintf("%s\n", buffer);

    // 减少预算
    blackboxHeaderBudget -= strlen(name) + strlen(buffer) + 4;
}

/**
 * @brief 检查Blackbox是否已打开
 * @return true 已打开
 */
bool bb_is_open(void)
{
    return bbopened != 0;
}

/**
 * @brief 获取日志编号
 * @return 日志编号
 */
int32_t bb_get_log_number(void)
{
    return bb_log_number;
}
