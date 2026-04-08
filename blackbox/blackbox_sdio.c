/**
 * @file blackbox_sdio.c
 * @brief Blackbox SD卡存储实现
 */
#include "blackbox_sdio.h"
#include "blackbox.h"
#include <rtdbg.h>
#include <rtthread.h>
#include <stdarg.h>
#include <string.h>

// SD卡驱动头文件
#include "sdcard.h"

/* ==================== 私有变量 ==================== */

// 写入缓冲区
static uint8_t write_buffer[BB_BUFFER_SIZE];
static uint32_t buffer_pos = 0;
static uint32_t sector_offset = 0;  // 相对安全区的扇区偏移
static uint32_t file_size = 0;

// 头部预算
int32_t blackboxHeaderBudget = 0;

// 获取安全区的起始扇区
#define BB_SAFE_START_SECTOR  sd_get_safe_test_sector()

// ==================== SDIO存储操作实现 ====================

/**
 * @brief 初始化SD卡存储
 */
bool bb_sdio_init(void)
{
    LOG_I("BB SDIO initializing SD card...");

    // 初始化SD卡
    sd_error_status_type ret = sd_init();
    if (ret != SD_OK)
    {
        LOG_E("BB SDIO SD card init failed: %d", ret);
        return false;
    }

    // 设置为DMA模式
    ret = sd_device_mode_set(SD_TRANSFER_DMA_MODE);
    if (ret != SD_OK)
    {
        LOG_E("BB SDIO set DMA mode failed: %d", ret);
        return false;
    }

    // 打印卡信息
    LOG_I("BB SDIO: Card Capacity=%u bytes, BlockSize=%u",
          sd_card_info.card_capacity, sd_card_info.card_blk_size);
    LOG_I("BB SDIO: Safe sector range: %u - %u", BB_SAFE_START_SECTOR, MAX_ALLOW_SECTOR);

    LOG_I("BB SDIO init OK");
    return true;
}

/**
 * @brief 打开日志文件
 */
bool bb_sdio_open(void)
{
    // 记录新的日志
    g_bb_storage.log_number++;
    sector_offset = 0;
    file_size = 0;
    buffer_pos = 0;

    // 清空缓冲区
    memset(write_buffer, 0, BB_BUFFER_SIZE);

    g_bb_storage.logging = true;
    blackboxHeaderBudget = 64;

    LOG_I("BB SDIO opened: log_%03u, start sector: %u", g_bb_storage.log_number, BB_SAFE_START_SECTOR);
    return true;
}

/**
 * @brief 关闭日志文件
 */
void bb_sdio_close(void)
{
    if (!g_bb_storage.logging)
    {
        return;
    }

    // 刷新缓冲区
    bb_sdio_flush();

    LOG_I("BB SDIO closed: log_%03u, size=%u bytes, sectors used: %u",
          g_bb_storage.log_number, file_size, sector_offset);

    g_bb_storage.logging = false;
}

/**
 * @brief 写入数据到缓冲区
 */
bool bb_sdio_write(const uint8_t *data, uint32_t len)
{
    if (!g_bb_storage.logging || data == NULL || len == 0)
    {
        return false;
    }

    uint32_t written = 0;
    while (written < len)
    {
        uint32_t remain = len - written;
        uint32_t space = BB_BUFFER_SIZE - buffer_pos;

        if (remain >= space)
        {
            // 缓冲区满，先写入SD卡
            memcpy(write_buffer + buffer_pos, data + written, space);
            buffer_pos += space;
            written += space;

            // 刷写缓冲区到SD卡
            if (!bb_sdio_flush())
            {
                LOG_E("BB SDIO flush failed");
                return false;
            }
        }
        else
        {
            // 数据可以全部放入缓冲区
            memcpy(write_buffer + buffer_pos, data + written, remain);
            buffer_pos += remain;
            written += remain;
        }
    }

    return true;
}

/**
 * @brief 刷新缓冲区到SD卡（使用安全读写函数）
 */
bool bb_sdio_flush(void)
{
    if (!g_bb_storage.logging || buffer_pos == 0)
    {
        return true;
    }

    // 计算需要写入的扇区数
    uint32_t sectors = (buffer_pos + BB_SECTOR_SIZE - 1) / BB_SECTOR_SIZE;

    // 计算绝对扇区号（安全区起始 + 偏移）
    uint32_t start_sector = BB_SAFE_START_SECTOR + sector_offset;

    // 检查安全区域
    if (!sd_is_sector_safe(start_sector) || !sd_is_sector_safe(start_sector + sectors - 1))
    {
        LOG_E("BB SDIO: sector out of safe range! start=%u, count=%u", start_sector, sectors);
        return false;
    }

    // 填充剩余空间为0xFF
    if (buffer_pos < sectors * BB_SECTOR_SIZE)
    {
        memset(write_buffer + buffer_pos, 0xFF,
               sectors * BB_SECTOR_SIZE - buffer_pos);
    }

    // 使用安全读写函数写入SD卡
    sd_error_status_type ret;
    if (sectors == 1)
    {
        ret = sd_safe_block_write(write_buffer, start_sector);
    }
    else
    {
        ret = sd_safe_multi_write(write_buffer, start_sector, sectors);
    }

    if (ret != SD_OK)
    {
        LOG_E("BB SDIO write failed: %d, sector=%u", ret, start_sector);
        return false;
    }

    // 更新状态
    sector_offset += sectors;
    file_size += buffer_pos;
    buffer_pos = 0;

    return true;
}

/**
 * @brief 检查是否正在记录
 */
bool bb_sdio_is_open(void)
{
    return g_bb_storage.logging;
}

/**
 * @brief 获取日志编号
 */
int32_t bb_sdio_get_log_number(void)
{
    return g_bb_storage.log_number;
}

/* ==================== 操作函数集 ==================== */

// SDIO存储操作函数集
static bbStorageOps_t bb_sdio_ops = {
    .init = bb_sdio_init,
    .open = bb_sdio_open,
    .close = bb_sdio_close,
    .write = bb_sdio_write,
    .flush = bb_sdio_flush,
    .is_open = bb_sdio_is_open,
    .get_log_number = bb_sdio_get_log_number,
};

/**
 * @brief 获取SDIO存储操作函数集
 */
void* bb_sdio_get_ops(void)
{
    return &bb_sdio_ops;
}