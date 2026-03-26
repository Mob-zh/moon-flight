#include "flight_init.h"
#include "bz121.h"
#include "dshot600.h"
#include "elrs.h"
#include "flight_control.h"
#include "imu.h"
#include "position_ekf.h"

extern int ano_update_init(void);

// 全局飞行控制实例

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

    // DShot600初始化
    dshot600_init();

    // IMU初始化（会创建imu_sem和control_sem）
    IMU_init();

    // 飞行控制初始化（创建控制线程，等待control_sem）
    flight_control_init(&g_flight_control);

    // 气压计初始化
    // baro_init(&g_bmp280_baro);

    // GPS初始化
    // gps_init(&g_bz121_gps);

    ano_update_init();

    // 初始化定时器1,定时触发控制中断
    crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
    nvic_irq_enable(TMR1_OVF_TMR10_IRQn, 0, 0);
    wk_tmr1_init();

    return true;
}
