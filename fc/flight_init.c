#include "flight_init.h"
#include "imu.h"
#include "elrs.h"

// 初始化检查列表
// 32位分别对应一个初始化检查项,需要初始化的项的位初始值为1,
// 若需要初始化则对应位设为1,初始化完成后对应位设为0
union
{
    uint32_t all;
    struct
    {
        uint32_t imu_clear_offset : 1;
    } bit;
} flight_init_checklist;

// 判断是否需要检查对应的位
bool get_init_check_flag(uint32_t flag)
{
    return (flight_init_checklist.all & flag) == flag;
}

// 清除初始化检查位函数
bool clear_init_check_bit(uint32_t flag)
{
    return flight_init_checklist.all &= ~flag;
}

// 设置初始化检查位函数
bool set_init_check_flag(uint32_t flag)
{
    return flight_init_checklist.all |= flag;
}

// 飞行器初始化函数
bool flight_init(void)
{
    // 遥控器接收初始化
    rx_init();

    // IMU初始化
    IMU_init();

    // 气压计初始化
    baro_init(&g_bmp280_baro);
}
