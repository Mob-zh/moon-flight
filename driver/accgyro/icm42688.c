#include "icm42688.h"
#include "at32f435_437_gpio.h"
#include "drv_spi.h"
#include "flight_init.h"
#include "spi_dev.h"
#include <rtdevice.h>
#include <rtthread.h>
// -------------------------- SPI硬件配置 --------------------------
#define ICM42688_SPI_BUS_NAME "spi1"      // SPI1总线名称（对应menuconfig初始化的SPI1）
#define ICM42688_SPI_DEV_NAME "spi10"     // 挂载后的SPI设备名称
#define SPI1_CS_GPIO_PORT     GPIOC       // CS引脚端口
#define SPI1_CS_GPIO_PIN      GPIO_PINS_4 // CS引脚号（PC4）

// -------------------------- 测试线程配置 --------------------------
#define ICM42688_TEST_THREAD_STACK_SIZE 1024 // 线程栈大小
#define ICM42688_TEST_THREAD_PRIORITY   20   // 线程优先级（数值越大优先级越低）
#define ICM42688_TEST_THREAD_TICK       10   // 线程调度周期（100ms）

static struct rt_spi_device *icm_spi_dev = RT_NULL;

// ICM42688P的AAF滤波器参数（258Hz默认）
static const aafConfig_t aafConfig = {6, 36, 10};
// ODR配置表
static const uint8_t odrLUT[] = {3, 4, 5, 6}; // 8K/4K/2K/1K对应的寄存器值

accgyroDev_t g_icm_accgyro; // 惯导设备实例
/************************ SPI设备挂载+初始化（RT-Thread DMA模式） ************************/
/**
 * @brief 挂载SPI1设备（绑定CS引脚）+ 初始化SPI配置
 */
static int accgyro_device_attach(void)
{
    rt_err_t ret = RT_EOK;

    // 1. 挂载SPI设备：将CS引脚绑定到SPI1总线，生成spi10设备
    ret = rt_hw_spi_device_attach(ICM42688_SPI_BUS_NAME,
                                  ICM42688_SPI_DEV_NAME,
                                  SPI1_CS_GPIO_PORT,
                                  SPI1_CS_GPIO_PIN);
    if (ret != RT_EOK)
    {
        rt_kprintf("SPI1 device attach failed! ret = %d\n", ret);

        return -RT_ERROR;
    }

    // 2. 查找挂载后的SPI设备
    icm_spi_dev = (struct rt_spi_device *)rt_device_find(ICM42688_SPI_DEV_NAME);
    if (icm_spi_dev == RT_NULL)
    {
        rt_kprintf("Can't find %s device!\n", ICM42688_SPI_DEV_NAME);
        return -RT_ERROR;
    }

    // 3. 配置SPI参数（适配ICM42688，DMA模式自动生效）
    struct rt_spi_configuration spi_cfg = {0};
    spi_cfg.mode                        = RT_SPI_MODE_0 | RT_SPI_MSB; // CPOL=0, CPHA=0, 高位先行（ICM42688默认）
    spi_cfg.data_width                  = 8;                          // 8位数据宽度
    spi_cfg.max_hz                      = 9 * 1000 * 1000;            // SPI时钟9MHz
    rt_spi_configure(icm_spi_dev, &spi_cfg);

    rt_kprintf("SPI1 device attach and init success!\n");
    return RT_EOK;
}

/************************ 核心功能函数 ************************/
/**
 * @brief 切换寄存器Bank
 * @param bank: Bank编号(0/1/2)
 */
static void set_bank(uint8_t bank)
{
    spi_write_reg(icm_spi_dev, ICM42688P_RA_REG_BANK_SEL, bank & 0x07);
}

/**
 * @brief ICM42688P软复位
 */
static void icm42688p_soft_reset(void)
{
    set_bank(ICM42688P_BANK_SELECT0);
    spi_write_reg(icm_spi_dev, ICM42688P_RA_DEVICE_CONFIG, DEVICE_CONFIG_SOFT_RESET_BIT);
    rt_thread_mdelay(1);
}

/**
 * @brief 检测ICM42688P设备
 * @return TRUE-检测成功，FALSE-检测失败
 */
static bool icm42688p_detect(void)
{
    icm42688p_soft_reset();
    rt_thread_mdelay(10); // 复位后稳定延时

    // 关闭传感器，准备检测
    set_bank(ICM42688P_BANK_SELECT0);
    spi_write_reg(icm_spi_dev, ICM42688P_RA_PWR_MGMT0, 0x2F);

    // 多次尝试读取WHO AM I
    uint8_t attempts = 20;
    while (attempts--)
    {
        rt_thread_mdelay(1);
        uint8_t whoami = spi_read_reg(icm_spi_dev, ICM42688P_RA_WHO_AM_I);
        if (whoami == ICM42688P_WHO_AM_I_CONST)
        {
            rt_kprintf("ICM42688P detect success! WHO AM I = 0x%02X\n", whoami);
            return true;
        }
    }
    rt_kprintf("ICM42688P detect failed!\n");
    return false;
}

/**
 * @brief 关闭陀螺仪和加速度计
 */
static void sensor_power_off(void)
{
    set_bank(ICM42688P_BANK_SELECT0);
    spi_write_reg(icm_spi_dev, ICM42688P_RA_PWR_MGMT0, PWR_MGMT0_GYRO_ACCEL_OFF);
}

/**
 * @brief 开启陀螺仪和加速度计（低噪声模式）
 */
static void sensor_power_on(void)
{
    set_bank(ICM42688P_BANK_SELECT0);
    spi_write_reg(icm_spi_dev, ICM42688P_RA_PWR_MGMT0,
                  PWR_MGMT0_ACCEL_MODE_LN | PWR_MGMT0_GYRO_MODE_LN);
    rt_thread_mdelay(1);
}

/************************ 陀螺仪/加速度计初始化/读取 ************************/
/**
 * @brief 惯性传感器初始化
 * @param accgyro: 惯性传感器设备结构体指针
 */
static bool icm42688_init(accgyroDev_t *accgyro)
{
    if (accgyro == NULL)
        return false;

    // 基础参数初始化
    accgyro->gyroRateKHz     = 1;                  // 默认1KHz采样率
    accgyro->mpuDividerDrops = 0;                  // 不分频
    accgyro->gyroScale       = 2000.0f / 32768.0f; // 陀螺仪换算系数：±2000 deg/s → 2000 / 32768 ≈ 0.061035
    accgyro->scale           = 2000.0f;            // 2000DPS量程
    accgyro->tempScale       = 1.0f / 132.48f;     // 温度校准系数
    accgyro->tempZero        = 25.0f;              // 温度零点25°C
    accgyro->acc_1G          = 2048.0f;
    accgyro->accScale        = 1.0f / 2048.0f; // 加速度计换算系数：LSB → g

    // 寄存器地址赋值
    accgyro->gyroDataReg = ICM42688P_RA_GYRO_DATA_X1;
    accgyro->accDataReg  = ICM42688P_RA_ACCEL_DATA_X1;
    accgyro->tempDataReg = ICM42688P_RA_TEMP_DATA1;

    // 检测设备
    if (!icm42688p_detect())
    {
        return false;
    }

    // 关闭传感器，准备配置
    sensor_power_off();

    // 配置陀螺仪抗混叠滤波器(AAF)
    set_bank(ICM42688P_BANK_SELECT1);
    spi_write_reg(icm_spi_dev, ICM42688P_RA_GYRO_CONFIG_STATIC3, aafConfig.delt);
    spi_write_reg(icm_spi_dev, ICM42688P_RA_GYRO_CONFIG_STATIC4, aafConfig.deltSqr & 0xFF);
    spi_write_reg(icm_spi_dev, ICM42688P_RA_GYRO_CONFIG_STATIC5, (aafConfig.deltSqr >> 8) | (aafConfig.bitshift << 4));

    // 配置加速度计抗混叠滤波器(AAF)
    set_bank(ICM42688P_BANK_SELECT2);
    spi_write_reg(icm_spi_dev, ICM42688P_RA_ACCEL_CONFIG_STATIC2, aafConfig.delt << 1);
    spi_write_reg(icm_spi_dev, ICM42688P_RA_ACCEL_CONFIG_STATIC3, aafConfig.deltSqr & 0xFF);
    spi_write_reg(icm_spi_dev, ICM42688P_RA_ACCEL_CONFIG_STATIC4, (aafConfig.deltSqr >> 8) | (aafConfig.bitshift << 4));

    // 配置UI滤波器（低延迟模式）
    set_bank(ICM42688P_BANK_SELECT0);
    spi_write_reg(icm_spi_dev, ICM42688P_RA_GYRO_ACCEL_CONFIG0,
                  ACCEL_UI_FILT_BW_LOW_LATENCY | GYRO_UI_FILT_BW_LOW_LATENCY);

    // 禁用AFSR，修复输出卡顿
    uint8_t intf_cfg = spi_read_reg(icm_spi_dev, ICM42688P_INTF_CONFIG1);
    intf_cfg &= ~INTF_CONFIG1_AFSR_MASK;
    intf_cfg |= INTF_CONFIG1_AFSR_DISABLE;
    spi_write_reg(icm_spi_dev, ICM42688P_INTF_CONFIG1, intf_cfg);

    // 开启传感器
    sensor_power_on();

    // 配置采样率(ODR)和量程（默认1KHz，2000DPS/16G）
    uint8_t odr_config = odrLUT[ODR_1K];
    spi_write_reg(icm_spi_dev, ICM42688P_RA_GYRO_CONFIG0, (0 << 5) | odr_config);
    rt_thread_mdelay(15);
    spi_write_reg(icm_spi_dev, ICM42688P_RA_ACCEL_CONFIG0, (0 << 5) | odr_config);
    rt_thread_mdelay(15);

    // 记录零偏
    accgyro->clear_offset(accgyro);

    return true;
}

/**
 * @brief 读取陀螺仪数据
 * @param gyro: 陀螺仪设备结构体指针
 * @return TRUE-读取成功，FALSE-失败
 */
static bool icm42688_read_gyro(accgyroDev_t *accgyro)
{
    if (accgyro == NULL)
        return false;

    uint8_t buf[6] = {0};
    // 读取陀螺仪X/Y/Z轴数据（6字节：X1/X0/Y1/Y0/Z1/Z0）
    spi_read_regs(icm_spi_dev, accgyro->gyroDataReg, buf, 6);

    // 拼接16位数据
    accgyro->gyroData[0] = (int16_t)((buf[0] << 8) | buf[1]); // X轴
    accgyro->gyroData[1] = (int16_t)((buf[2] << 8) | buf[3]); // Y轴
    accgyro->gyroData[2] = (int16_t)((buf[4] << 8) | buf[5]); // Z轴

    // // 读取温度数据
    // uint8_t temp_buf[2] = {0};
    // spi_read_regs(icm_spi_dev, accgyro->tempDataReg, temp_buf, 2);
    // int16_t temp_raw  = (int16_t)((temp_buf[1] << 8) | temp_buf[0]);
    // accgyro->tempData = (float)temp_raw * accgyro->tempScale + accgyro->tempZero;

    return true;
}

/**
 * @brief 读取加速度计数据
 * @param acc: 加速度计设备结构体指针
 * @return TRUE-读取成功，FALSE-失败
 */
static bool icm42688_read_acc(accgyroDev_t *accgyro)
{
    if (accgyro == NULL)
        return false;

    uint8_t buf[6] = {0};
    // 读取加速度计X/Y/Z轴数据（6字节）
    spi_read_regs(icm_spi_dev, accgyro->accDataReg, buf, 6);

    // 拼接16位原始数据
    accgyro->accData[0] = (int16_t)((buf[0] << 8) | buf[1]); // X轴
    accgyro->accData[1] = (int16_t)((buf[2] << 8) | buf[3]); // Y轴
    accgyro->accData[2] = (int16_t)((buf[4] << 8) | buf[5]); // Z轴

    return true;
}

uint8_t ICM42688_offset(accgyroDev_t *dev, uint16_t sensivity)
{
    static int32_t  tempgx = 0, tempgy = 0, tempgz = 0;
    static int32_t  tempax = 0, tempay = 0, tempaz = 0;
    static uint16_t cnt_a = 0; // 使用static修饰的局部变量，表明次变量具有静态存储周期，也就是说该函数执行完后不释放内存
    if (cnt_a == 0)
    {
        dev->accData[0]         = 0;
        dev->accData[1]         = 0;
        dev->accData[2]         = 0;
        dev->gyroData[0]        = 0;
        dev->gyroData[1]        = 0;
        dev->gyroData[2]        = 0;
        dev->accData_offset[0]  = 0;
        dev->accData_offset[1]  = 0;
        dev->accData_offset[2]  = 0;
        dev->gyroData_offset[0] = 0;
        dev->gyroData_offset[1] = 0;
        dev->gyroData_offset[2] = 0;
        tempgx                  = 0;
        tempgy                  = 0;
        tempgz                  = 0;
        cnt_a                   = 1;
        sensivity               = 0;
    }

    while (cnt_a < 200)
    {
        dev->readAcc(dev);
        dev->readGyro(dev);

        tempax += dev->accData[0];
        tempay += dev->accData[1];
        tempaz += dev->accData[2];
        tempgx += dev->gyroData[0];
        tempgy += dev->gyroData[1];
        tempgz += dev->gyroData[2];

        cnt_a++;
    }

    if (cnt_a == 200) // 200个数值求平均
    {
        dev->accData_offset[0] = tempax / cnt_a;
        dev->accData_offset[1] = tempay / cnt_a;
        dev->accData_offset[2] = tempaz / cnt_a;
        cnt_a                  = 0;

        return 1;
    }

    return 0;
}

/******************************************************************************
* 函  数：void MPU6050_DataProcess(void)
* 功  能：对MPU6050进行去零偏处理
* 参  数：无
* 返回值：无
* 备  注：无

*******************************************************************************/
bool ICM42688_clear_offset(accgyroDev_t *dev)
{
    // 查找标志位IMU_OFFSET是否设置，若置位则进行校准
    if (get_init_check_flag(IMU_CLEAR_OFFSET_CHECK)) // 陀螺仪进行零偏校准
    {
        if (ICM42688_offset(dev, 0))
        {
            // 校准完成，清除标志位，待完成实际代码
            clear_init_check_bit(IMU_CLEAR_OFFSET_CHECK);
        }
    }
}

/************************ 对外接口 ************************/
/**
 * @brief 初始化ICM42688P（陀螺仪+加速度计）
 * @param gyro: 陀螺仪设备结构体
 * @param acc: 加速度计设备结构体
 * @return TRUE-初始化成功，FALSE-失败
 */
bool accgyro_init(accgyroDev_t *dev)
{
    if (dev == NULL)
        return false;

    // 挂载并初始化SPI设备（含DMA）
    if (accgyro_device_attach() != RT_EOK)
    {
        rt_kprintf("SPI device init failed!\n");
        return false;
    }

    // 初始化陀螺仪
    dev->init         = icm42688_init;
    dev->readGyro     = icm42688_read_gyro;
    dev->readAcc      = icm42688_read_acc;
    dev->clear_offset = ICM42688_clear_offset;
    if (!(dev->init(dev)))
    {
        rt_kprintf("ICM42688P initialization failed!\n");
        return false;
    }

    return true;
}