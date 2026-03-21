#include "bmp280.h"
#include "math.h"
#include "spi_dev.h"
// -------------------------- SPI硬件配置 --------------------------
#define BMP280_SPI_BUS_NAME "spi2"      // SPI2总线名称（对应menuconfig初始化的SPI2）
#define BMP280_SPI_DEV_NAME "spi20"     // 挂载后的SPI设备名称
#define SPI2_CS_GPIO_PORT   GPIOC       // CS引脚端口
#define SPI2_CS_GPIO_PIN    GPIO_PINS_8 // CS引脚号（PC8）

static struct rt_spi_device *bmp280_spi_dev = RT_NULL;

// -------------------------- 测试线程配置 --------------------------
#define BMP280_TEST_THREAD_STACK_SIZE 1024 // 线程栈大小
#define BMP280_TEST_THREAD_PRIORITY   20   // 线程优先级（数值越大优先级越低）
#define BMP280_TEST_THREAD_TICK       10   // 线程调度周期（100ms）

/************************ 全局变量定义 ************************/
int32_t  t_fine; // 温度校准中间值（全局共享）
uint16_t Dig_T1; // 温度校准参数
int16_t  Dig_T2;
int16_t  Dig_T3;
uint16_t Dig_P1; // 压力校准参数
int16_t  Dig_P2;
int16_t  Dig_P3;
int16_t  Dig_P4;
int16_t  Dig_P5;
int16_t  Dig_P6;
int16_t  Dig_P7;
int16_t  Dig_P8;
int16_t  Dig_P9;

baroDev_t g_bmp280_baro; // 气压计设备实例

// 函数声明
static bool bmp280_detect(void);

/************************ SPI设备挂载+初始化（RT-Thread DMA模式） ************************/
/**
 * @brief 挂载SPI1设备（绑定CS引脚）+ 初始化SPI配置
 */
static int baro_device_attach(void)
{
    rt_err_t ret = RT_EOK;
    // 1. 挂载SPI设备：将CS引脚绑定到SPI2总线，生成spi20设备
    ret = rt_hw_spi_device_attach(BMP280_SPI_BUS_NAME,
                                  BMP280_SPI_DEV_NAME,
                                  SPI2_CS_GPIO_PORT,
                                  SPI2_CS_GPIO_PIN);
    if (ret != RT_EOK)
    {
        rt_kprintf("SPI2 device attach failed! ret = %d\n", ret);

        return -RT_ERROR;
    }

    // 2. 查找挂载后的SPI设备
    bmp280_spi_dev = (struct rt_spi_device *)rt_device_find(BMP280_SPI_DEV_NAME);
    if (bmp280_spi_dev == RT_NULL)
    {
        rt_kprintf("Can't find %s device!\n", BMP280_SPI_DEV_NAME);
        return -RT_ERROR;
    }

    // 3. 配置SPI参数（适配BMP280）
    struct rt_spi_configuration spi_cfg = {0};
    spi_cfg.mode                        = RT_SPI_MODE_0 | RT_SPI_MSB; // CPOL=0, CPHA=0, 高位先行（BMP280默认）
    spi_cfg.data_width                  = 8;                          // 8位数据宽度
    spi_cfg.max_hz                      = 9 * 1000 * 1000;            // SPI时钟9MHz
    rt_spi_configure(bmp280_spi_dev, &spi_cfg);

    rt_kprintf("BMP280 device attach and init success!\n");
    return RT_EOK;
}

/**
 * @brief 读取转换3个连续寄存器（温度/压力专用）
 * @param addr: 起始寄存器地址（必须是 MSB 寄存器地址，如 BMP280_TEMPERATURE_MSB_REG）
 */
static int32_t bmp280_read_3regs(uint8_t addr)
{
    uint8_t data[3] = {0};
    int32_t adc_val = 0;

    // 读取3个寄存器：data[0] = MSB, data[1] = LSB, data[2] = XLSB
    spi_read_regs(bmp280_spi_dev, addr, data, 3);

    // 正确拼接20位数据：(MSB << 12) | (LSB << 4) | (XLSB >> 4)
    adc_val = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)data[2] >> 4);

    // 校验数据有效性（避免全0/全1的错误数据）
    if (adc_val == 0 || adc_val == 0xFFFFF)
    {
        rt_kprintf("BMP280 ADC data read error: 0x%08X\n", adc_val);
        return -1;
    }

    return adc_val;
}

/**
 * @brief 读取转换2个连续寄存器（校准参数专用）
 * @param addr: 起始寄存器地址（LSB地址）
 */
static int16_t bmp280_read_2regs(uint8_t addr)
{
    uint8_t data[2] = {0};
    int16_t cal_val = 0;

    // 读取2个寄存器：data[0] = LSB, data[1] = MSB
    spi_read_regs(bmp280_spi_dev, addr, data, 2);

    // 正确拼接16位校准参数：MSB << 8 | LSB
    cal_val = (int16_t)((data[1] << 8) | data[0]);

    return cal_val;
}

/************************ 核心初始化与数据读取 ************************/

/**
 * @brief 气压计初始化
 * @param baro: 气压计设备结构体指针
 */
static bool bmp280_init(baroDev_t *baro)
{
    if (baro == NULL)
        return false;

    // 检测设备
    if (!bmp280_detect())
    {
        return false;
    }

    uint8_t Osrs_T   = 1; // Temperature oversampling x 1
    uint8_t Osrs_P   = 1; // Pressure oversampling x 1
    uint8_t Mode     = 3; // Normal mode
    uint8_t T_sb     = 4; // Tstandby 500ms
    uint8_t Filter   = 4; // Filter
    uint8_t Spi3w_en = 0; // 3-wire SPI Disable

    uint8_t Ctrl_meas_reg = (Osrs_T << 5) | (Osrs_P << 2) | Mode;
    uint8_t Config_reg    = (T_sb << 5) | (Filter << 2) | Spi3w_en;

    // 复位BMP280
    spi_write_reg(bmp280_spi_dev, BMP280_RESET_REG, BMP280_RESET_VALUE);
    rt_thread_mdelay(10);

    // 配置测量参数
    spi_write_reg(bmp280_spi_dev, BMP280_CTRLMEAS_REG, Ctrl_meas_reg);
    spi_write_reg(bmp280_spi_dev, BMP280_CONFIG_REG, Config_reg);
    rt_thread_mdelay(20);

    // 读取校准参数
    Dig_T1 = bmp280_read_2regs(BMP280_DIG_T1_LSB_REG);
    Dig_T2 = bmp280_read_2regs(BMP280_DIG_T2_LSB_REG);
    Dig_T3 = bmp280_read_2regs(BMP280_DIG_T3_LSB_REG);
    Dig_P1 = bmp280_read_2regs(BMP280_DIG_P1_LSB_REG);
    Dig_P2 = bmp280_read_2regs(BMP280_DIG_P2_LSB_REG);
    Dig_P3 = bmp280_read_2regs(BMP280_DIG_P3_LSB_REG);
    Dig_P4 = bmp280_read_2regs(BMP280_DIG_P4_LSB_REG);
    Dig_P5 = bmp280_read_2regs(BMP280_DIG_P5_LSB_REG);
    Dig_P6 = bmp280_read_2regs(BMP280_DIG_P6_LSB_REG);
    Dig_P7 = bmp280_read_2regs(BMP280_DIG_P7_LSB_REG);
    Dig_P8 = bmp280_read_2regs(BMP280_DIG_P8_LSB_REG);
    Dig_P9 = bmp280_read_2regs(BMP280_DIG_P9_LSB_REG);

    return true;
}

/**
 * @brief 探测bmp280的id是否正确
 * @return true-检测成功，false-检测失败
 */
static bool bmp280_detect(void)
{

    rt_thread_mdelay(1); // 上电延时
    // 多次尝试读取ID
    uint8_t attempts = 20;
    while (attempts--)
    {
        rt_thread_mdelay(1);
        uint8_t ID = spi_read_reg(bmp280_spi_dev, BMP280_CHIPID_REG);
        if (ID == BMP280_CHIP_ID)
        {
            return true;
        }
    }
    return false;
}
/**
 * @brief 获取BMP280温度值（官方校准算法）
 * @return 温度值（单位：0.01℃）
 */
bool BMP280_GetTemp(baroDev_t *baro)
{
    if (baro == NULL)
        return false;

    // 在读取数据前检查测量状态
    uint8_t status = spi_read_reg(bmp280_spi_dev, BMP280_STATUS_REG);
    if (status & 0x08)
    { // 检查测量是否完成
        rt_kprintf("Measurement not ready\r\n");
        return false;
    }

    int32_t adc_T = bmp280_read_3regs(BMP280_TEMPERATURE_MSB_REG);
    if (adc_T < 0)
        return false; // 读取失败

    // 官方温度校准公式（保持不变）
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)Dig_T1 << 1))) * ((int32_t)Dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)Dig_T1)) * ((adc_T >> 4) - ((int32_t)Dig_T1))) >> 12) *
            ((int32_t)Dig_T3)) >>
           14;

    t_fine = var1 + var2;
    T      = (t_fine * 5 + 128) >> 8; // 转换为0.01℃单位

    baro->temp = T;
    return true;
}

/**
 * @brief 获取BMP280压力值（官方校准算法）
 * @return 压力值（单位：Pa）
 */
bool BMP280_GetPress(baroDev_t *baro)
{
    if (baro == NULL)
        return false;

    // 等待测量完成
    uint8_t timeout = 10;
    while (timeout--)
    {
        uint8_t status = spi_read_reg(bmp280_spi_dev, BMP280_STATUS_REG);
        if ((status & 0x08) == 0) // measuring=0表示完成
            break;
        rt_thread_mdelay(1);
    }
    if (timeout == 0)
    {
        rt_kprintf("BMP280 measure timeout\r\n");
        return false;
    }

    // 必须先读取温度，保证t_fine有效
    if (!BMP280_GetTemp(baro) || t_fine == 0)
    {
        rt_kprintf("Read temperature first failed!\n");
        return false;
    }

    int32_t adc_P = bmp280_read_3regs(BMP280_PRESSURE_MSB_REG);
    if (adc_P < 0)
        return false; // 读取失败

    // 官方压力校准公式（保持不变）
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)Dig_P6;
    var2 = var2 + ((var1 * (int64_t)Dig_P5) << 17);
    var2 = var2 + (((int64_t)Dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)Dig_P3) >> 8) + ((var1 * (int64_t)Dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)Dig_P1) >> 33;

    // 避免除零错误
    if (var1 == 0)
    {
        rt_kprintf("var1 is zero, pressure calculate failed!\n");
        return false;
    }

    p    = 1048576 - adc_P;
    p    = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)Dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)Dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)Dig_P7) << 4);

    // BMP280输出是Q24.8格式，需要除以256得到Pa
    baro->pressure = (uint32_t)(p >> 8);
    return true;
}
/**
 * @brief 根据气压计算海拔高度
 * @param baro: 气压计设备结构体
 * @param seaLevelPressure: 海平面气压(默认101325Pa)
 * @return 海拔高度 (cm)
 */
int32_t baro_get_altitude(baroDev_t *baro, uint32_t seaLevelPressure)
{
    if (baro == NULL)
        return 0;

    if (seaLevelPressure == 0)
    {
        seaLevelPressure = 101325;
    }

    // 气压高度公式：h = 44330 * (1 - (P/P0)^(1/5.255))
    float pressureRatio = (float)baro->pressure / (float)seaLevelPressure;
    float altitude      = 44330.0f * (1.0f - pow(pressureRatio, 1.0f / 5.255f));

    baro->altitude = (int32_t)(altitude * 100.0f); // 转换为cm
    return baro->altitude;
}

bool bmp280_read_press(baroDev_t *baro)
{
    if (BMP280_GetPress(baro) == false)
    {
        return false;
    }
    return true;
}

/************************ 对外接口 ************************/
/**
 * @brief 初始化气压计
 * @param baro: 气压计设备结构体
 * @return TRUE-初始化成功，FALSE-失败
 */
bool baro_init(baroDev_t *baro)
{
    if (baro == NULL)
        return false;

    // 挂载并初始化SPI设备（含DMA）
    if (baro_device_attach() != RT_EOK)
    {
        rt_kprintf("SPI device init failed!\n");
        return false;
    }

    // 初始化气压计
    baro->init         = bmp280_init;
    baro->read_press   = bmp280_read_press;
    baro->get_altitude = baro_get_altitude;
    baro->init(baro);

    return true;
}
