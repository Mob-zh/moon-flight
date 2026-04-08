/**
 * @file blackbox_test.c
 * @brief Blackbox SD卡存储测试程序
 *        模拟飞行数据写入SD卡
 */
#include "blackbox.h"
#include "blackbox_sdio.h"
#include "blackbox_encode.h"
#include <rtdbg.h>
#include <rtthread.h>
#include <string.h>
#include <stdlib.h>

#define BB_TEST_THREAD_PRIORITY   20
#define BB_TEST_THREAD_STACK_SIZE 8192
#define TEST_FRAME_COUNT          100     // 测试帧数

/* 测试统计 */
static struct {
    uint32_t init_ok;
    uint32_t open_ok;
    uint32_t frame_count;
    uint32_t close_ok;
    uint32_t total_tests;
    uint32_t passed;
} bb_test_result = {0};

/**
 * @brief 测试初始化
 */
static int bb_test_init(void)
{
    bb_test_result.total_tests++;

    LOG_I("========== Blackbox Init Test ==========");

    // 初始化Blackbox（会自动注册并初始化SDIO存储）
    bb_init();

    if (!g_bb_storage.initialized) {
        LOG_E("[FAIL] BB storage not initialized");
        return -1;
    }

    LOG_I("[PASS] BB init OK");
    LOG_I("  Storage ops: %s", g_bb_storage.ops ? "registered" : "none");
    bb_test_result.init_ok++;
    bb_test_result.passed++;
    return 0;
}

/**
 * @brief 测试打开日志
 */
static int bb_test_open(void)
{
    bb_test_result.total_tests++;

    LOG_I("========== Blackbox Open Test ==========");

    bb_open();

    if (!bb_is_logging()) {
        LOG_E("[FAIL] BB should be logging");
        return -1;
    }

    LOG_I("[PASS] BB open OK, log number: %d", bb_get_log_number());
    bb_test_result.open_ok++;
    bb_test_result.passed++;
    return 0;
}

/**
 * @brief 测试写入飞行数据
 */
static int bb_test_write_data(void)
{
    bb_test_result.total_tests++;

    LOG_I("========== Blackbox Write Data Test ==========");

    // 创建主状态数据
    bb_main_state_t state;
    memset(&state, 0, sizeof(state));

    // 写入测试帧
    LOG_I("Writing %d test frames...", TEST_FRAME_COUNT);

    for (uint32_t i = 0; i < TEST_FRAME_COUNT; i++) {
        // 生成模拟飞行数据
        state.time = i * 4000; // 4ms周期

        // 模拟RC输入
        state.rcCommand[0] = 1500 + 300 * sin(i * 0.1);  // roll
        state.rcCommand[1] = 1500 + 200 * cos(i * 0.1);  // pitch
        state.rcCommand[2] = 1500;                        // yaw
        state.rcCommand[3] = 1000 + (i % 500);            // throttle

        // 模拟陀螺仪
        state.gyroADC[0] = (i * 10) % 2000 - 1000;
        state.gyroADC[1] = (i * 8) % 2000 - 1000;
        state.gyroADC[2] = (i * 5) % 1000 - 500;

        // 模拟加速度计
        state.accSmooth[0] = (rand() % 100) - 50;
        state.accSmooth[1] = (rand() % 100) - 50;
        state.accSmooth[2] = 400 + (rand() % 50);

        // 模拟电机
        for (int m = 0; m < 4; m++) {
            state.motor[m] = state.rcCommand[3] - 1000 + (rand() % 20 - 10);
        }

        // 模拟PID数据
        for (int j = 0; j < 3; j++) {
            state.axisPID_P[j] = (rand() % 2000) - 1000;
            state.axisPID_I[j] = (rand() % 1000) - 500;
            state.axisPID_D[j] = (rand() % 500) - 250;
            state.axisPID_F[j] = (rand() % 300) - 150;
        }

        // 模拟电池
        state.vbatLatest = 124 + (rand() % 10);
        state.amperageLatest = 5000 + rand() * 1000;

        // 模拟气压高度
        state.baroAlt = i * 2;

        // 模拟RSSI
        state.rssi = 100;

        // 调用bb_update写入帧
        bb_update(&state);
        bb_test_result.frame_count++;

        // 每20帧刷新一次缓冲区
        if (i % 20 == 0) {
            bb_flush();
        }

        rt_thread_mdelay(5); // 模拟4ms周期
    }

    bb_flush();

    LOG_I("[PASS] BB write %u frames OK", bb_test_result.frame_count);
    bb_test_result.passed++;
    return 0;
}

/**
 * @brief 测试关闭日志
 */
static int bb_test_close(void)
{
    bb_test_result.total_tests++;

    LOG_I("========== Blackbox Close Test ==========");

    bb_close();

    if (bb_is_logging()) {
        LOG_E("[FAIL] BB should not be logging");
        return -1;
    }

    LOG_I("[PASS] BB close OK");
    bb_test_result.close_ok++;
    bb_test_result.passed++;
    return 0;
}

/**
 * @brief 打印测试结果
 */
static void bb_test_print_result(void)
{
    LOG_I("========== Blackbox Test Summary ==========");
    LOG_I("Total tests: %u", bb_test_result.total_tests);
    LOG_I("Passed: %u", bb_test_result.passed);
    LOG_I("Failed: %u", bb_test_result.total_tests - bb_test_result.passed);
    LOG_I("Frames written: %u", bb_test_result.frame_count);
    LOG_I("==========================================");

    if (bb_test_result.passed == bb_test_result.total_tests) {
        LOG_I(">>> BLACKBOX TEST PASSED <<<");
    } else {
        LOG_E(">>> BLACKBOX TEST FAILED <<<");
    }
}

/**
 * @brief Blackbox测试线程
 */
static void blackbox_test_thread(void *parameter)
{
    LOG_I("Starting Blackbox SD Card Test...");

    // 1. 初始化测试
    if (bb_test_init() != 0) {
        LOG_E("Init test failed");
        bb_test_print_result();
        return;
    }

    rt_thread_mdelay(100);

    // 2. 打开日志
    if (bb_test_open() != 0) {
        LOG_E("Open test failed");
        bb_test_print_result();
        return;
    }

    rt_thread_mdelay(50);

    // 3. 写入飞行数据
    if (bb_test_write_data() != 0) {
        LOG_E("Write data test failed");
        bb_test_print_result();
        return;
    }

    rt_thread_mdelay(50);

    // 4. 关闭日志
    if (bb_test_close() != 0) {
        LOG_E("Close test failed");
        bb_test_print_result();
        return;
    }

    // 打印结果
    bb_test_print_result();
}

/**
 * @brief 启动Blackbox测试
 */
int blackbox_test_start(void)
{
    rt_thread_t tid = rt_thread_create("bb_test",
                                        blackbox_test_thread,
                                        RT_NULL,
                                        BB_TEST_THREAD_STACK_SIZE,
                                        BB_TEST_THREAD_PRIORITY,
                                        10);

    if (tid != RT_NULL) {
        rt_thread_startup(tid);
        LOG_I("Blackbox test thread started");
        return 0;
    } else {
        LOG_E("Create BB test thread failed");
        return -1;
    }
}

MSH_CMD_EXPORT(blackbox_test_start, "Start blackbox SD card test");

// 自动启动测试（取消注释以自动运行）
// INIT_APP_EXPORT(blackbox_test_start);