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
int32_t BMP280_GetAltitude(uint32_t pressure, uint32_t seaLevelPressure);

int main(void)
{
    rt_uint32_t speed = 500;
    /* set led2 pin mode to output */
    rt_pin_mode(LED1_PIN, PIN_MODE_OUTPUT);

    flight_init();
    baro_init(&g_bmp280_baro);
    while (1)
    {
        g_bmp280_baro.read_press(&g_bmp280_baro);
        int32_t altitude = BMP280_GetAltitude(g_bmp280_baro.pressure, 101325);
        rt_kprintf("press: %d, altitude: %d, temp: %d\n", g_bmp280_baro.pressure, altitude, g_bmp280_baro.temp);

        rt_pin_write(LED1_PIN, PIN_LOW);
        rt_thread_mdelay(speed);

        rt_pin_write(LED1_PIN, PIN_HIGH);
        rt_thread_mdelay(speed);
    }
}
