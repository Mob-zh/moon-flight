#include "ano_data.h"
#include "at32f435_437_wk_config.h"
#include "bmp280.h"
#include "drv_common.h"
#include "drv_gpio.h"
#include "flight_init.h"
#include "imu.h"
#include "wk_system.h"
#include <rtdevice.h>
#include <rtthread.h>

/* defined the led1 pin: pc2 */
#define LED1_PIN GET_PIN(C, 2)
rt_uint32_t speed = 100;

uint16_t imu_cnt = 0;
uint16_t pid_cnt = 0;

void flight_init_entry(void *parameter)
{
    flight_init();
    speed = 500;
}

int main(void)
{
    /* set led2 pin mode to output */
    rt_pin_mode(LED1_PIN, PIN_MODE_OUTPUT);

    rt_thread_t flight_init_thread = rt_thread_create("flight_init", flight_init_entry, NULL, 2048, 5, 10);
    rt_thread_startup(flight_init_thread);

    while (1)
    {
        rt_pin_write(LED1_PIN, PIN_LOW);
        rt_thread_mdelay(speed);

        rt_pin_write(LED1_PIN, PIN_HIGH);
        rt_thread_mdelay(speed);
    }
}
