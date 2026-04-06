/**
 * @file blackbox_encode.c
 * @brief Blackbox 变长字节编码实现
 */
#include "blackbox_encode.h"
#include "blackbox_io.h"
#include <stdint.h>
#include <string.h>

/**
 * @brief ZigZag编码：将带符号整数转换为无符号
 * @param value 带符号整数
 * @return ZigZag编码后的无符号整数
 */
static uint32_t zigzag_encode(int32_t value)
{
    // (value << 1) ^ (value >> 31)
    return (uint32_t)((value << 1) ^ (value >> 31));
}

/**
 * @brief 写入无符号整数使用VB编码
 * @param value 要写入的值
 */
void bb_write_unsigned_vb(uint32_t value)
{
    // 每个字节7位数据，最高位表示"后续有更多字节"
    while (value > 127) {
        bb_write((uint8_t)(value | 0x80)); // 高位设为1表示还有字节
        value >>= 7;
    }
    bb_write((uint8_t)value); // 最后一个字节最高位为0
}

/**
 * @brief 写入有符号整数使用ZigZag+VB编码
 * @param value 要写入的值
 */
void bb_write_signed_vb(int32_t value)
{
    bb_write_unsigned_vb(zigzag_encode(value));
}

/**
 * @brief 写入16位有符号整数（小端序）
 * @param value 要写入的值
 */
void bb_write_s16(int16_t value)
{
    bb_write((uint8_t)(value & 0xFF));
    bb_write((uint8_t)((value >> 8) & 0xFF));
}

/**
 * @brief 写入32位无符号整数（小端序）
 * @param value 要写入的值
 */
void bb_write_u32(uint32_t value)
{
    bb_write((uint8_t)(value & 0xFF));
    bb_write((uint8_t)((value >> 8) & 0xFF));
    bb_write((uint8_t)((value >> 16) & 0xFF));
    bb_write((uint8_t)((value >> 24) & 0xFF));
}

/**
 * @brief 写入浮点数（按原始字节）
 * @param value 要写入的值
 */
void bb_write_float(float value)
{
    // 通过内存拷贝将float转换为uint32_t
    uint32_t bits;
    memcpy(&bits, &value, sizeof(float));
    bb_write_u32(bits);
}

/**
 * @brief 写入有符号VB数组
 * @param array 数组指针
 * @param count 元素数量
 */
void bb_write_signed_vb_array(int32_t *array, int count)
{
    for (int i = 0; i < count; i++) {
        bb_write_signed_vb(array[i]);
    }
}

/**
 * @brief 写入16位有符号VB数组
 * @param array 数组指针
 * @param count 元素数量
 */
void bb_write_signed16_vb_array(int16_t *array, int count)
{
    for (int i = 0; i < count; i++) {
        bb_write_signed_vb(array[i]);
    }
}

/**
 * @brief 写入2位标签+3个有符号字段 (2/4/6/32位)
 * @param values 3个值的数组
 */
void bb_write_tag2_3s32(int32_t *values)
{
    static const int NUM_FIELDS = 3;

    enum {
        BITS_2  = 0,
        BITS_4  = 1,
        BITS_6  = 2,
        BITS_32 = 3
    };

    enum {
        BYTES_1 = 0,
        BYTES_2 = 1,
        BYTES_3 = 2,
        BYTES_4 = 3
    };

    int selector = BITS_2;

    // 计算所需位数
    for (int x = 0; x < NUM_FIELDS; x++) {
        if (values[x] >= 32 || values[x] < -32) {
            selector = BITS_32;
            break;
        }
        if (values[x] >= 8 || values[x] < -8) {
            if (selector < BITS_6) {
                selector = BITS_6;
            }
        } else if (values[x] >= 2 || values[x] < -2) {
            if (selector < BITS_4) {
                selector = BITS_4;
            }
        }
    }

    switch (selector) {
    case BITS_2:
        // 2位/字段: ss11 2233
        bb_write((selector << 6) |
                ((values[0] & 0x03) << 4) |
                ((values[1] & 0x03) << 2) |
                (values[2] & 0x03));
        break;

    case BITS_4:
        // 4位/字段: ss00 1111 2222 3333
        bb_write((selector << 6) | (values[0] & 0x0F));
        bb_write((values[1] << 4) | (values[2] & 0x0F));
        break;

    case BITS_6:
        // 6位/字段
        bb_write((selector << 6) | (values[0] & 0x3F));
        bb_write((uint8_t)values[1]);
        bb_write((uint8_t)values[2]);
        break;

    case BITS_32: {
        // 每个字段独立选择字节数
        int selector2 = 0;
        for (int x = NUM_FIELDS - 1; x >= 0; x--) {
            selector2 <<= 2;
            if (values[x] < 128 && values[x] >= -128) {
                selector2 |= BYTES_1;
            } else if (values[x] < 32768 && values[x] >= -32768) {
                selector2 |= BYTES_2;
            } else if (values[x] < 8388608 && values[x] >= -8388608) {
                selector2 |= BYTES_3;
            } else {
                selector2 |= BYTES_4;
            }
        }

        bb_write((selector << 6) | selector2);

        for (int x = 0; x < NUM_FIELDS; x++, selector2 >>= 2) {
            switch (selector2 & 0x03) {
            case BYTES_1:
                bb_write(values[x]);
                break;
            case BYTES_2:
                bb_write(values[x]);
                bb_write(values[x] >> 8);
                break;
            case BYTES_3:
                bb_write(values[x]);
                bb_write(values[x] >> 8);
                bb_write(values[x] >> 16);
                break;
            case BYTES_4:
                bb_write(values[x]);
                bb_write(values[x] >> 8);
                bb_write(values[x] >> 16);
                bb_write(values[x] >> 24);
                break;
            }
        }
        break;
    }
    }
}

/**
 * @brief 写入8位标签+4个有符号字段 (0/4/8/16位)
 * @param values 4个值的数组
 */
void bb_write_tag8_4s16(int32_t *values)
{
    enum {
        FIELD_ZERO   = 0,
        FIELD_4BIT  = 1,
        FIELD_8BIT  = 2,
        FIELD_16BIT = 3
    };

    uint8_t selector = 0;

    // 反向编码使第一个字段在低位
    for (int x = 3; x >= 0; x--) {
        selector <<= 2;

        if (values[x] == 0) {
            selector |= FIELD_ZERO;
        } else if (values[x] < 8 && values[x] >= -8) {
            selector |= FIELD_4BIT;
        } else if (values[x] < 128 && values[x] >= -128) {
            selector |= FIELD_8BIT;
        } else {
            selector |= FIELD_16BIT;
        }
    }

    bb_write(selector);

    int nibbleIndex = 0;
    uint8_t buffer = 0;

    for (int x = 0; x < 4; x++, selector >>= 2) {
        switch (selector & 0x03) {
        case FIELD_ZERO:
            // 无操作
            break;
        case FIELD_4BIT:
            if (nibbleIndex == 0) {
                buffer = (uint8_t)(values[x] << 4);
                nibbleIndex = 1;
            } else {
                bb_write(buffer | (values[x] & 0x0F));
                nibbleIndex = 0;
            }
            break;
        case FIELD_8BIT:
            if (nibbleIndex == 0) {
                bb_write((uint8_t)values[x]);
            } else {
                bb_write(buffer | ((values[x] >> 4) & 0x0F));
                buffer = (uint8_t)(values[x] << 4);
            }
            break;
        case FIELD_16BIT:
            if (nibbleIndex == 0) {
                bb_write((uint8_t)(values[x] >> 8));
                bb_write((uint8_t)values[x]);
            } else {
                bb_write(buffer | ((values[x] >> 12) & 0x0F));
                bb_write((uint8_t)(values[x] >> 4));
                buffer = (uint8_t)(values[x] << 4);
            }
            break;
        }
    }

    if (nibbleIndex == 1) {
        bb_write(buffer);
    }
}

/**
 * @brief 写入8位标签+8个有符号字段（非零值使用VB）
 * @param values 值数组
 * @param valueCount 元素数量（≤8）
 */
void bb_write_tag8_8svb(int32_t *values, int valueCount)
{
    if (valueCount <= 0) return;

    if (valueCount == 1) {
        bb_write_signed_vb(values[0]);
        return;
    }

    // 写入头部标记哪些字段非零
    uint8_t header = 0;
    for (int i = valueCount - 1; i >= 0; i--) {
        header <<= 1;
        if (values[i] != 0) {
            header |= 0x01;
        }
    }

    bb_write(header);

    // 只写入非零值
    for (int i = 0; i < valueCount; i++) {
        if (values[i] != 0) {
            bb_write_signed_vb(values[i]);
        }
    }
}
