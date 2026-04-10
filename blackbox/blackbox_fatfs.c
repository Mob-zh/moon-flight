/**
 * @file blackbox_fatfs.c
 * @brief Blackbox FatFS 文件系统存储实现
 */
#include "blackbox_fatfs.h"
#include "blackbox.h"
#include "blackbox_sdio.h"
#include "diskio.h"
#include "ff.h"
#include <rtdbg.h>
#include <rtthread.h>
#include <stdarg.h>
#include <string.h>

/* ==================== 私有变量 ==================== */

// 写入缓冲区
static uint8_t  write_buffer[BB_BUFFER_SIZE];
static uint32_t buffer_pos = 0;
static uint32_t file_size  = 0;

// FatFS 文件对象
static FATFS fat_fs;
static FIL   fil;

// 头部预算
static int32_t blackboxHeaderBudget = 0;

// 日志编号
static int32_t g_log_number = 0;

/* ==================== FatFS 存储操作实现 ==================== */

/**
 * @brief 初始化 FatFS 存储
 */
bool bb_fatfs_init(void)
{
    DSTATUS st;

    LOG_I("BB FatFS initializing...");

    /* 初始化 SD 卡 */
    st = disk_initialize(0);
    if (st & STA_NOINIT)
    {
        LOG_E("BB FatFS disk init failed: 0x%02X", st);
        return false;
    }

    /* 挂载文件系统 */
    FRESULT res = f_mount(&fat_fs, "", 1);
    if (res != FR_OK)
    {
        LOG_E("BB FatFS f_mount failed: %d", res);

        /* 检查是否需要格式化 */
        if (res == FR_NO_FILESYSTEM)
        {
            LOG_W("BB FatFS No filesystem, formatting...");
            uint8_t   workbuf[4096];
            MKFS_PARM mkfs_opt = {0};
            mkfs_opt.fmt       = FM_ANY | FM_SFD;
            res                = f_mkfs("", &mkfs_opt, workbuf, sizeof(workbuf));
            if (res != FR_OK)
            {
                LOG_E("BB FatFS format failed: %d", res);
                return false;
            }
            LOG_I("BB FatFS format done");

            /* 重新挂载 */
            res = f_mount(&fat_fs, "", 1);
            if (res != FR_OK)
            {
                LOG_E("BB FatFS remount failed: %d", res);
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    LOG_I("BB FatFS init OK");
    return true;
}

/**
 * @brief 打开日志文件
 */
bool bb_fatfs_open(void)
{
    FRESULT res;
    char    filename[32];

    /* 递增日志编号 */
    g_log_number++;

    /* 生成文件名: BBL000.bbl */
    snprintf(filename, sizeof(filename), "BBL%03u.bbl", g_log_number);

    /* 创建/打开文件 */
    res = f_open(&fil, filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK)
    {
        LOG_E("BB FatFS open file failed: %d, %s", res, filename);
        return false;
    }

    /* 重置状态 */
    buffer_pos = 0;
    file_size  = 0;

    /* 清空缓冲区 */
    memset(write_buffer, 0, BB_BUFFER_SIZE);

    g_bb_storage.logging = true;
    blackboxHeaderBudget = 64;

    LOG_I("BB FatFS opened: %s", filename);
    return true;
}

/**
 * @brief 关闭日志文件
 */
void bb_fatfs_close(void)
{
    if (!g_bb_storage.logging)
    {
        return;
    }

    /* 刷新缓冲区 */
    bb_fatfs_flush();

    /* 关闭文件 */
    f_close(&fil);

    LOG_I("BB FatFS closed: log_%03u, size=%u", g_log_number, file_size);

    g_bb_storage.logging = false;
}

/**
 * @brief 写入数据到缓冲区
 */
bool bb_fatfs_write(const uint8_t *data, uint32_t len)
{
    if (!g_bb_storage.logging || data == NULL || len == 0)
    {
        return false;
    }

    uint32_t written = 0;
    while (written < len)
    {
        uint32_t remain = len - written;
        uint32_t space  = BB_BUFFER_SIZE - buffer_pos;

        if (remain >= space)
        {
            /* 缓冲区满，先写入SD卡 */
            memcpy(write_buffer + buffer_pos, data + written, space);
            buffer_pos += space;
            written += space;

            /* 刷写缓冲区到文件 */
            if (!bb_fatfs_flush())
            {
                LOG_E("BB FatFS flush failed");
                return false;
            }
        }
        else
        {
            /* 数据可以全部放入缓冲区 */
            memcpy(write_buffer + buffer_pos, data + written, remain);
            buffer_pos += remain;
            written += remain;
        }
    }

    return true;
}

/**
 * @brief 刷新缓冲区到文件
 */
bool bb_fatfs_flush(void)
{
    UINT bw;

    if (!g_bb_storage.logging || buffer_pos == 0)
    {
        return true;
    }

    /* 写入文件 */
    FRESULT res = f_write(&fil, write_buffer, buffer_pos, &bw);
    if (res != FR_OK)
    {
        LOG_E("BB FatFS write failed: %d", res);
        return false;
    }

    /* 更新状态 */
    file_size += buffer_pos;
    buffer_pos = 0;

    return true;
}

/**
 * @brief 检查是否正在记录
 */
bool bb_fatfs_is_open(void)
{
    return g_bb_storage.logging;
}

/**
 * @brief 获取日志编号
 */
int32_t bb_fatfs_get_log_number(void)
{
    return g_log_number;
}

/* ==================== 操作函数集 ==================== */

// FatFS 存储操作函数集
static bbStorageOps_t bb_fatfs_ops = {
    .init           = bb_fatfs_init,
    .open           = bb_fatfs_open,
    .close          = bb_fatfs_close,
    .write          = bb_fatfs_write,
    .flush          = bb_fatfs_flush,
    .is_open        = bb_fatfs_is_open,
    .get_log_number = bb_fatfs_get_log_number,
};

/**
 * @brief 获取 FatFS 存储操作函数集
 */
void *bb_fatfs_get_ops(void)
{
    return &bb_fatfs_ops;
}
