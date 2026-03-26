#include "at32f435_437_conf.h"
#include "imu.h"

// 外部信号量声明 (供中断处理函数使用)
extern rt_sem_t control_sem;

void TMR1_OVF_TMR10_IRQHandler(void)
{
    rt_interrupt_enter();
    /* overflow interrupt management */
    if (tmr_interrupt_flag_get(TMR1, TMR_OVF_FLAG) != RESET)
    {
        // 发送信号量，通知控制线程进行姿态解算和PID运算
        rt_sem_release(control_sem);
        /* clear flag */
        tmr_flag_clear(TMR1, TMR_OVF_FLAG);
    }
    rt_interrupt_leave();
}
