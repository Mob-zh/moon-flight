#include "at32f435_437_conf.h"
#include "dshot600.h"
#include "imu.h"
// 外部信号量声明 (供中断处理函数使用)
extern rt_sem_t        imu_sem;
extern int8_t          g_dshot_run_enable;
extern uint16_t        g_dshot_run_throttle;
extern dshot_channel_e g_dshot_run_channel;

void TMR1_OVF_TMR10_IRQHandler(void)
{
    rt_interrupt_enter();

    /* overflow interrupt management */
    if (tmr_interrupt_flag_get(TMR1, TMR_OVF_FLAG) != RESET)
    {
        // 发送信号量，通知imu线程进行姿态解算
        rt_sem_release(imu_sem);
        // 暂时放到这，之后放到控制线程中
        if (g_dshot_run_enable)
        {
            dshot600_send_throttle(0, g_dshot_run_throttle);
            dshot600_send_throttle(1, g_dshot_run_throttle);
            dshot600_send_throttle(2, g_dshot_run_throttle);
            dshot600_send_throttle(3, g_dshot_run_throttle);
        }
        /* clear flag */
        tmr_flag_clear(TMR1, TMR_OVF_FLAG);
    }
    rt_interrupt_leave();
}