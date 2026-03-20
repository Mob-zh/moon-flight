
#include "at32f435_437_wk_config.h"
#include "dshot600.h"
#include <rtthread.h>

extern rt_event_t dshot_event;
/**
 * @brief  this function handles DMA1 Channel 1 handler.
 * @param  none
 * @retval none
 */
void DMA1_Channel3_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    if (dma_interrupt_flag_get(DMA1_FDT3_FLAG) != RESET)
    {
        /* add user code begin DMA1_FDT3_FLAG */
        /* handle full data transfer and clear flag */
        rt_event_send(dshot_event, DSHOT1_DMA_FDT_EVENT);
        set_tmr_pwm_duty(TMR3, TMR_SELECT_CHANNEL_1, 0);

        dma_flag_clear(DMA1_FDT3_FLAG);
        /* add user code end DMA1_FDT3_FLAG */
    }

    /* leave interrupt */
    rt_interrupt_leave();
}

/**
 * @brief  this function handles DMA1 Channel 2 handler.
 * @param  none
 * @retval none
 */
void DMA1_Channel4_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    if (dma_interrupt_flag_get(DMA1_FDT4_FLAG) != RESET)
    {
        /* add user code begin DMA1_FDT4_FLAG */
        /* handle full data transfer and clear flag */
        rt_event_send(dshot_event, DSHOT2_DMA_FDT_EVENT);
        set_tmr_pwm_duty(TMR3, TMR_SELECT_CHANNEL_2, 0);
        dma_flag_clear(DMA1_FDT4_FLAG);
        /* add user code end DMA1_FDT4_FLAG */
    }

    /* leave interrupt */
    rt_interrupt_leave();
}

/**
 * @brief  this function handles DMA1 Channel 5 handler.
 * @param  none
 * @retval none
 */
void DMA1_Channel5_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    if (dma_interrupt_flag_get(DMA1_FDT5_FLAG) != RESET)
    {
        /* add user code begin DMA1_FDT5_FLAG */
        /* handle full data transfer and clear flag */
        rt_event_send(dshot_event, DSHOT3_DMA_FDT_EVENT);
        set_tmr_pwm_duty(TMR4, TMR_SELECT_CHANNEL_1, 0);
        dma_flag_clear(DMA1_FDT5_FLAG);
        /* add user code end DMA1_FDT5_FLAG */
    }

    /* leave interrupt */
    rt_interrupt_leave();
}

/**
 * @brief  this function handles DMA1 Channel 6 handler.
 * @param  none
 * @retval none
 */
void DMA1_Channel6_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    if (dma_interrupt_flag_get(DMA1_FDT6_FLAG) != RESET)
    {
        /* add user code begin DMA1_FDT6_FLAG */
        /* handle full data transfer and clear flag */
        rt_event_send(dshot_event, DSHOT4_DMA_FDT_EVENT);
        set_tmr_pwm_duty(TMR4, TMR_SELECT_CHANNEL_2, 0);
        dma_flag_clear(DMA1_FDT6_FLAG);
        /* add user code end DMA1_FDT6_FLAG */
    }

    /* leave interrupt */
    rt_interrupt_leave();
}