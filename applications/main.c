#include "ano_data.h"
#include "at32f435_437_wk_config.h"
#include "drv_common.h"
#include "drv_gpio.h"
#include "flight_init.h"
#include "imu.h"
#include "wk_system.h"
#include <rtdevice.h>
#include <rtthread.h>

/* defined the led1 pin: pc2 */
#define LED1_PIN GET_PIN(C, 2)

int main(void)
{
    rt_uint32_t speed = 200;
    /* set led2 pin mode to output */
    rt_pin_mode(LED1_PIN, PIN_MODE_OUTPUT);

    flight_init();

    while (1)
    {
        rt_pin_write(LED1_PIN, PIN_LOW);
        rt_thread_mdelay(speed);

        rt_pin_write(LED1_PIN, PIN_HIGH);
        rt_thread_mdelay(speed);
    }
}
