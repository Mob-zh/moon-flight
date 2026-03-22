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

int main(void)
{
    rt_uint32_t speed = 500;
    /* set led2 pin mode to output */
    rt_pin_mode(LED1_PIN, PIN_MODE_OUTPUT);

    flight_init();
    dshot600_init();

    while (1)
    {
        g_bmp280_baro.read_press(&g_bmp280_baro);
        g_bmp280_baro.get_altitude(&g_bmp280_baro, 101325);

        // // 发送传感器数据到上位机(MAG_X, Y, Z, ALT_BAR, TMP, BAR_STA, MAG_STA)
        ANO_DT_Send_Sensor_Data(0, 0, 0, g_bmp280_baro.altitude,
                                (int16_t)(g_bmp280_baro.temp / 10), 0, 0);

        rt_pin_write(LED1_PIN, PIN_LOW);
        rt_thread_mdelay(speed);

        rt_pin_write(LED1_PIN, PIN_HIGH);
        rt_thread_mdelay(speed);
    }
}
