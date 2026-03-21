#ifndef __FLIGHT_INIT_H__
#define __FLIGHT_INIT_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bmp280.h"

#define IMU_CLEAR_OFFSET_CHECK 0x01
#define BARO_INIT_CHECK       0x02

// 初始化检查列表掩码(仅包含需要检查的位)
#define FLIGHT_INIT_CHECKLIST_MASK (IMU_CLEAR_OFFSET_CHECK)

// 判断是否需要检查对应的位
bool get_init_check_flag(uint32_t flag);

// 清除初始化检查位函数
bool clear_init_check_bit(uint32_t flag);

// 设置初始化检查位函数
bool set_init_check_flag(uint32_t flag);

bool flight_init(void);

#endif