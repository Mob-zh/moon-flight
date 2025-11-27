/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-05-16     shelton      first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "drv_common.h"
#include "drv_gpio.h"
#include "at32f435_437_wk_config.h"
#include "wk_system.h"

/* defined the led1 pin: pc2 */
#define LED1_PIN    GET_PIN(C, 2)


int main(void)
{
    rt_uint32_t speed = 200;
    /* set led2 pin mode to output */
    rt_pin_mode(LED1_PIN, PIN_MODE_OUTPUT);

    while (1)
    {
        rt_pin_write(LED1_PIN, PIN_LOW);
        rt_thread_mdelay(speed);

        rt_pin_write(LED1_PIN, PIN_HIGH);
        rt_thread_mdelay(speed);
    }
}
