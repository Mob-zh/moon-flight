/**
 * @file blackbox_encode.h
 * @brief Blackbox 变长字节编码函数
 *
 * 协议与Betaflight完全兼容，支持:
 * - Variable Byte 编码
 * - ZigZag 编码（处理负数）
 * - 标签压缩编码
 */
#ifndef _BLACKBOX_ENCODE_H_
#define _BLACKBOX_ENCODE_H_

#include <stdint.h>

// 声明IO层函数（在blackbox_io.h中实现）
extern void bb_write(uint8_t value);
extern void bb_write_string(const char *s);

/**
 * @brief 写入无符号整数使用VB编码
 * @param value 要写入的值
 */
void bb_write_unsigned_vb(uint32_t value);

/**
 * @brief 写入有符号整数使用ZigZag+VB编码
 * @param value 要写入的值
 */
void bb_write_signed_vb(int32_t value);

/**
 * @brief 写入16位有符号整数（小端序）
 * @param value 要写入的值
 */
void bb_write_s16(int16_t value);

/**
 * @brief 写入32位无符号整数（小端序）
 * @param value 要写入的值
 */
void bb_write_u32(uint32_t value);

/**
 * @brief 写入浮点数（按原始字节）
 * @param value 要写入的值
 */
void bb_write_float(float value);

/**
 * @brief 写入有符号VB数组
 * @param array 数组指针
 * @param count 元素数量
 */
void bb_write_signed_vb_array(int32_t *array, int count);

/**
 * @brief 写入16位有符号VB数组
 * @param array 数组指针
 * @param count 元素数量
 */
void bb_write_signed16_vb_array(int16_t *array, int count);

/**
 * @brief 写入2位标签+3个有符号字段 (2/4/6/32位)
 * @param values 3个值的数组
 */
void bb_write_tag2_3s32(int32_t *values);

/**
 * @brief 写入8位标签+4个有符号字段 (0/4/8/16位)
 * @param values 4个值的数组
 */
void bb_write_tag8_4s16(int32_t *values);

/**
 * @brief 写入8位标签+8个有符号字段（非零值使用VB）
 * @param values 值数组
 * @param valueCount 元素数量（≤8）
 */
void bb_write_tag8_8svb(int32_t *values, int valueCount);

#endif /* _BLACKBOX_ENCODE_H_ */
