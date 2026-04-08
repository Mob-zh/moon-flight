/**
 * @file sdcard_test.c
 * @brief SD卡测试程序 - 使用安全读写API
 *        参考官方示例 at32_sdio_test.c
 */
#include "sdcard.h"
#include <rtdbg.h>
#include <rtthread.h>
#include <stdbool.h>
#include <string.h>

#define BLOCK_SIZE        512
#define BLOCKS_NUMBER     64      // 多块测试扇区数
#define MULTI_BUFFER_SIZE (BLOCK_SIZE * BLOCKS_NUMBER)

#define TEST_THREAD_PRIORITY   20
#define TEST_THREAD_STACK_SIZE 8192

/* 测试结果统计 */
static struct
{
    uint32_t init_ok;
    uint32_t single_block_ok;
    uint32_t multi_blocks_ok;
    uint32_t total_tests;
    uint32_t passed;
} test_result = {0};

/* 测试缓冲区 */
static uint8_t sblock_tbuffer[BLOCK_SIZE], sblock_rbuffer[BLOCK_SIZE];
static uint8_t mblock_tbuffer[MULTI_BUFFER_SIZE], mblock_rbuffer[MULTI_BUFFER_SIZE];

/**
 * @brief 比较两个缓冲区
 */
static uint8_t buffer_compare(uint8_t *pbuffer1, uint8_t *pbuffer2, uint16_t buffer_length)
{
    while (buffer_length--)
    {
        if (*pbuffer1 != *pbuffer2)
        {
            return 0;
        }
        pbuffer1++;
        pbuffer2++;
    }
    return 1;
}

/**
 * @brief 初始化测试
 */
static int test_init(void)
{
    test_result.total_tests++;

    LOG_I("========== SD Card Init Test ==========");

    // 打印安全扇区范围
    LOG_I("Safe sector range: %u - %u", SAFE_START_SECTOR, MAX_ALLOW_SECTOR);

    sd_error_status_type ret = sd_init();
    if (ret == SD_OK)
    {
        LOG_I("[PASS] SD Card init OK");

        // 显示卡信息
        switch (sd_card_info.card_type)
        {
        case SDIO_STD_CAPACITY_SD_CARD_V1_1:
            LOG_I("  Card Type: SDSC V1.1");
            break;
        case SDIO_STD_CAPACITY_SD_CARD_V2_0:
            LOG_I("  Card Type: SDSC V2.0");
            break;
        case SDIO_HIGH_CAPACITY_SD_CARD:
            LOG_I("  Card Type: SDHC V2.0");
            break;
        case SDIO_MULTIMEDIA_CARD:
            LOG_I("  Card Type: MMC");
            break;
        case SDIO_HIGH_SPEED_MULTIMEDIA_CARD:
            LOG_I("  Card Type: MMC V4.2");
            break;
        case SDIO_HIGH_CAPACITY_MMC_CARD:
            LOG_I("  Card Type: eMMC");
            break;
        default:
            LOG_I("  Card Type: Unknown(%d)", sd_card_info.card_type);
            break;
        }

        LOG_I("  Card Capacity: %d bytes", sd_card_info.card_capacity);
        LOG_I("  Card Block Size: %d", sd_card_info.card_blk_size);
        LOG_I("  RCA: 0x%04X", sd_card_info.rca);

        // 设置为DMA模式
        ret = sd_device_mode_set(SD_TRANSFER_DMA_MODE);
        if (ret != SD_OK)
        {
            LOG_E("Failed to set DMA mode, error: %d", ret);
        }
        else
        {
            LOG_I("  Transfer Mode: DMA");
        }

        test_result.init_ok++;
        test_result.passed++;
        return 0;
    }
    else
    {
        LOG_E("[FAIL] SD Card init failed, error code: %d", ret);
        return -1;
    }
}

/**
 * @brief 单块读写测试（测试1-bit和4-bit宽度）
 */
static int test_single_block(void)
{
    test_result.total_tests++;

    LOG_I("========== Single Block Test ==========");

    sd_error_status_type status = SD_OK;
    uint8_t              bus_width;

    // 获取安全测试扇区
    uint32_t test_sector = sd_get_safe_test_sector();
    LOG_I("Using safe test sector: %u", test_sector);

    // 验证扇区安全检查
    if (!sd_is_sector_safe(test_sector))
    {
        LOG_E("Test sector not safe!");
        return -1;
    }
    if (sd_is_sector_safe(0))
    {
        LOG_E("Sector 0 should NOT be safe!");
        return -1;
    }

    // 测试1-bit和4-bit宽度
    for (bus_width = 0; bus_width < 2; bus_width++)
    {
        LOG_I("Testing bus width: %d-bit", (bus_width == 0) ? 1 : 4);

        // 填充测试数据
        memset(sblock_tbuffer, bus_width + 0xAB, BLOCK_SIZE);
        memset(sblock_rbuffer, 0, BLOCK_SIZE);

        // 设置总线宽度
        status = sd_wide_bus_operation_config((sdio_bus_width_type)bus_width);
        if (status != SD_OK)
        {
            LOG_E("  Set bus width failed: %d", status);
            return -1;
        }

        // 安全写入512字节
        status = sd_safe_block_write(sblock_tbuffer, test_sector);
        if (status != SD_OK)
        {
            LOG_E("  Safe write failed: %d", status);
            return -1;
        }
        LOG_I("  Safe Write OK");

        // 安全读取512字节
        status = sd_safe_block_read(sblock_rbuffer, test_sector);
        if (status != SD_OK)
        {
            LOG_E("  Safe read failed: %d", status);
            return -1;
        }
        LOG_I("  Safe Read OK");

        // 比较数据
        if (!buffer_compare(sblock_tbuffer, sblock_rbuffer, BLOCK_SIZE))
        {
            LOG_E("  Data compare failed!");
            return -1;
        }
        LOG_I("  Data verify OK");
    }

    LOG_I("[PASS] Single block test OK");
    test_result.single_block_ok++;
    test_result.passed++;
    return 0;
}

/**
 * @brief 多块读写测试（测试1-bit和4-bit宽度）
 */
static int test_multiple_blocks(void)
{
    test_result.total_tests++;

    LOG_I("========== Multiple Blocks Test ==========");

    sd_error_status_type status = SD_OK;
    uint8_t              bus_width;

    // 获取安全测试扇区（从安全区域开始）
    uint32_t test_sector = sd_get_safe_test_sector() + 100; // 偏移避免覆盖单块测试数据

    LOG_I("Using safe test sector: %u, count: %u", test_sector, BLOCKS_NUMBER);

    // 测试1-bit和4-bit宽度
    for (bus_width = 0; bus_width < 2; bus_width++)
    {
        LOG_I("Testing bus width: %d-bit, %d blocks", (bus_width == 0) ? 1 : 4, BLOCKS_NUMBER);

        // 填充测试数据
        memset(mblock_tbuffer, bus_width + 0x3C, MULTI_BUFFER_SIZE);
        memset(mblock_rbuffer, 0, MULTI_BUFFER_SIZE);

        // 设置总线宽度
        status = sd_wide_bus_operation_config((sdio_bus_width_type)bus_width);
        if (status != SD_OK)
        {
            LOG_E("  Set bus width failed: %d", status);
            return -1;
        }

        // 安全写入多块
        status = sd_safe_multi_write(mblock_tbuffer, test_sector, BLOCKS_NUMBER);
        if (status != SD_OK)
        {
            LOG_E("  Safe write multiple blocks failed: %d", status);
            return -1;
        }
        LOG_I("  Safe Write %d blocks OK", BLOCKS_NUMBER);

        // 安全读取多块
        status = sd_safe_multi_read(mblock_rbuffer, test_sector, BLOCKS_NUMBER);
        if (status != SD_OK)
        {
            LOG_E("  Safe read multiple blocks failed: %d", status);
            return -1;
        }
        LOG_I("  Safe Read %d blocks OK", BLOCKS_NUMBER);

        // 验证数据
        if (!buffer_compare(mblock_tbuffer, mblock_rbuffer, MULTI_BUFFER_SIZE))
        {
            LOG_E("  Data compare failed!");
            return -1;
        }
        LOG_I("  Data verify OK");
    }

    LOG_I("[PASS] Multiple blocks test OK");
    test_result.multi_blocks_ok++;
    test_result.passed++;
    return 0;
}

/**
 * @brief 打印测试结果
 */
static void print_result(void)
{
    LOG_I("========== Test Summary ==========");
    LOG_I("Total tests: %u", test_result.total_tests);
    LOG_I("Passed: %u", test_result.passed);
    LOG_I("Failed: %u", test_result.total_tests - test_result.passed);
    LOG_I("==================================");

    if (test_result.passed == test_result.total_tests)
    {
        LOG_I(">>> ALL TESTS PASSED <<<");
    }
    else
    {
        LOG_E(">>> SOME TESTS FAILED <<<");
    }
}

/**
 * @brief SD卡测试线程
 */
static void sdcard_test_thread(void *parameter)
{
    LOG_I("Starting SD Card Test (Safe Read/Write)...");

    // 1. 初始化测试
    if (test_init() != 0)
    {
        LOG_E("Init failed, abort tests");
        print_result();
        return;
    }

    rt_thread_mdelay(100);

    // 2. 单块读写测试
    if (test_single_block() != 0)
    {
        LOG_E("Single block test failed!");
    }
    rt_thread_mdelay(50);

    // 3. 多块读写测试
    if (test_multiple_blocks() != 0)
    {
        LOG_E("Multiple blocks test failed!");
    }
    rt_thread_mdelay(50);

    // 打印结果
    print_result();
}

/**
 * @brief 启动SD卡测试
 * @return 0 启动成功
 */
int sdcard_test_start(void)
{
    rt_thread_t tid = rt_thread_create("sd_test",
                                       sdcard_test_thread,
                                       RT_NULL,
                                       TEST_THREAD_STACK_SIZE,
                                       TEST_THREAD_PRIORITY,
                                       10);

    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
        LOG_I("SD test thread started");
        return 0;
    }
    else
    {
        LOG_E("Create SD test thread failed");
        return -1;
    }
}

MSH_CMD_EXPORT(sdcard_test_start, "Start SD card test");

// 自动启动测试（取消注释以自动运行）
// INIT_APP_EXPORT(sdcard_test_start);