/**
 * @file fatfs_test.c
 * @brief FatFS 测试代码
 */

#include "diskio.h"
#include "ff.h"
#include <rtthread.h>
#include <string.h>

#define DBG_TAG "fatfs_test"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static FATFS FatFS;

/**
 * @brief 测试SD卡读写
 */
int fatfs_test(void)
{
    FRESULT res;
    FIL     fil;
    UINT    bw;
    uint8_t write_buf[64] = "Hello from FatFS on SD Card!\n";
    uint8_t read_buf[64];

    LOG_I("FatFS test start...");

    /* 0. 初始化SD卡 */
    DSTATUS st = disk_initialize(0);
    if (st & STA_NOINIT)
    {
        LOG_E("disk_initialize failed: 0x%02X", st);
        return -1;
    }
    LOG_I("disk_initialize success");

    /* 1. 挂载文件系统 */
    res = f_mount(&FatFS, "", 1);
    if (res != FR_OK)
    {
        LOG_E("f_mount failed: %d", res);

        /* 🔴 自动格式化 */
        if (res == FR_NO_FILESYSTEM)
        {
            LOG_W("Formatting SD card to FAT32...");
            MKFS_PARM mkfs_opt = {0};
            mkfs_opt.fmt       = FM_ANY | FM_SFD;
            res                = f_mkfs("", &mkfs_opt, 0, 4096);
            if (res == FR_OK)
            {
                LOG_I("Format done! Remounting...");
                res = f_mount(&FatFS, "", 1);
            }
        }

        if (res != FR_OK)
            return -1;
    }
    LOG_I("f_mount success");

    /* 2. 尝试打开文件测试 */
    res = f_open(&fil, "test.txt", FA_READ);
    if (res == FR_OK)
    {
        /* 文件已存在，读取测试 */
        f_read(&fil, read_buf, sizeof(read_buf), &bw);
        f_close(&fil);
        LOG_I("Read existing file: %s", read_buf);
        return 0;
    }
    else if (res != FR_NO_FILE)
    {
        LOG_E("f_open failed: %d", res);
        return -1;
    }

    /* 3. 文件不存在，创建并写入 */
    res = f_open(&fil, "test.txt", FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK)
    {
        LOG_E("f_open for write failed: %d", res);
        return -1;
    }

    /* 4. 写入数据 */
    res = f_write(&fil, write_buf, strlen((char *)write_buf), &bw);
    if (res != FR_OK)
    {
        LOG_E("f_write failed: %d", res);
        f_close(&fil);
        return -1;
    }
    LOG_I("f_write success, wrote %d bytes", bw);
    f_close(&fil);

    /* 5. 重新打开读取验证 */
    res = f_open(&fil, "test.txt", FA_READ);
    if (res != FR_OK)
    {
        LOG_E("f_open for read failed: %d", res);
        return -1;
    }

    rt_memset(read_buf, 0, sizeof(read_buf));
    res = f_read(&fil, read_buf, sizeof(read_buf) - 1, &bw);
    f_close(&fil);

    if (res != FR_OK)
    {
        LOG_E("f_read failed: %d", res);
        return -1;
    }

    LOG_I("f_read success: %s", read_buf);
    LOG_I("FatFS test passed!");
    return 0;
}

/* 自动初始化，在系统启动后执行 */
// INIT_APP_EXPORT(fatfs_test);
MSH_CMD_EXPORT(fatfs_test, FatFS SD card test);
