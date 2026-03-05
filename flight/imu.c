#include "icm42688.h"
#include <math.h>
#include <stdint.h>

// ==================== 基础数学 ====================
#define M_PI                    3.14159265358979323846
#define DEGREES_TO_RADIANS(deg) ((deg) * M_PI / 180.0f)
#define RADIANS_TO_DEGREES(rad) ((rad) * 180.0f / M_PI)

typedef struct
{
    float w, x, y, z;
} quaternion_t;

typedef struct
{
    float m[3][3];
} matrix33_t;

typedef struct
{
    int16_t roll;  // 横滚
    int16_t pitch; // 俯仰
    int16_t yaw;   // 偏航（会飘）
} attitudeEulerAngles_t;

// 全局变量
quaternion_t          q = {1, 0, 0, 0}; // 四元数
matrix33_t            rMat;             // 旋转矩阵
attitudeEulerAngles_t attitude;         // 姿态角

// 快速平方根倒数
static float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

// 计算旋转矩阵
static void imuComputeRotationMatrix(void)
{
    float qw = q.w, qx = q.x, qy = q.y, qz = q.z;

    rMat.m[0][0] = 1 - 2 * qy * qy - 2 * qz * qz;
    rMat.m[0][1] = 2 * qx * qy - 2 * qz * qw;
    rMat.m[0][2] = 2 * qx * qz + 2 * qy * qw;

    rMat.m[1][0] = 2 * qx * qy + 2 * qz * qw;
    rMat.m[1][1] = 1 - 2 * qx * qx - 2 * qz * qz;
    rMat.m[1][2] = 2 * qy * qz - 2 * qx * qw;

    rMat.m[2][0] = 2 * qx * qz - 2 * qy * qw;
    rMat.m[2][1] = 2 * qy * qz + 2 * qx * qw;
    rMat.m[2][2] = 1 - 2 * qx * qx - 2 * qy * qy;
}

// ==================== 核心：Mahony 滤波 ====================
// 只使用：陀螺仪 + 加速度计
static void IMU_Update(float dt,                     // 时间间隔，单位秒
                       float gx, float gy, float gz, // 陀螺仪，单位：deg/s
                       float ax, float ay, float az) // 加速度计，任意单位
{
    // 1. 转为弧度制
    gx = DEGREES_TO_RADIANS(gx);
    gy = DEGREES_TO_RADIANS(gy);
    gz = DEGREES_TO_RADIANS(gz);

    // 2. 归一化加速度计
    float norm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    // 3. 求误差（重力方向偏差）
    float ex = ay * rMat.m[2][2] - az * rMat.m[2][1];
    float ey = az * rMat.m[2][0] - ax * rMat.m[2][2];
    float ez = ax * rMat.m[2][1] - ay * rMat.m[2][0];

    // 4. 融合修正（比例控制）
    float Kp = 2.0f; // 调这个！越大加速度计修正越强
    float Ki = 0.0f; // 无磁力计，积分关掉

    // 积分项（这里关掉，防止漂移）
    static float ix = 0, iy = 0, iz = 0;
    ix += ex * Ki * dt;
    iy += ey * Ki * dt;
    iz += ez * Ki * dt;

    gx += Kp * ex + ix;
    gy += Kp * ey + iy;
    gz += Kp * ez + iz;

    // 5. 四元数积分
    float qw = q.w, qx = q.x, qy = q.y, qz = q.z;

    q.w += (-qx * gx - qy * gy - qz * gz) * 0.5f * dt;
    q.x += (+qw * gx + qy * gz - qz * gy) * 0.5f * dt;
    q.y += (+qw * gy - qx * gz + qz * gx) * 0.5f * dt;
    q.z += (+qw * gz + qx * gy - qy * gx) * 0.5f * dt;

    // 6. 归一化四元数
    norm = invSqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    q.w *= norm;
    q.x *= norm;
    q.y *= norm;
    q.z *= norm;

    // 7. 更新旋转矩阵
    imuComputeRotationMatrix();

    // 8. 计算欧拉角
    attitude.roll  = RADIANS_TO_DEGREES(atan2f(rMat.m[2][1], rMat.m[2][2])) * 10;
    attitude.pitch = RADIANS_TO_DEGREES(asinf(-rMat.m[2][0])) * 10;
    // yaw 只能靠陀螺仪积分，会漂移！
    attitude.yaw += RADIANS_TO_DEGREES(gz) * dt * 10;
}

static void IMU_update_thread_entry(void *parameter)
{
    // 校准偏移（先运行校准函数获取，示例值需实际校准）
    float accOffset[3]  = {0.0f, 0.0f, 0.0f}; // 静态偏移，需校准
    float gyroOffset[3] = {0.0f, 0.0f, 0.0f}; // 陀螺仪零偏，需校准

    while (1)
    {
        // 读取传感器数据
        g_icm_accgyro.readGyro(&g_icm_accgyro);
        g_icm_accgyro.readAcc(&g_icm_accgyro);

        // 1. 陀螺仪数据处理：LSB → deg/s（减去零偏）
        float gx = (float)g_icm_accgyro.gyroData[0] * g_icm_accgyro.gyroScale - gyroOffset[0];
        float gy = (float)g_icm_accgyro.gyroData[1] * g_icm_accgyro.gyroScale - gyroOffset[1];
        float gz = (float)g_icm_accgyro.gyroData[2] * g_icm_accgyro.gyroScale - gyroOffset[2];

        // 2. 加速度计数据处理：LSB → g（减去零偏）
        float ax = (float)g_icm_accgyro.accData[0] * g_icm_accgyro.accScale - accOffset[0];
        float ay = (float)g_icm_accgyro.accData[1] * g_icm_accgyro.accScale - accOffset[1];
        float az = (float)g_icm_accgyro.accData[2] * g_icm_accgyro.accScale - accOffset[2];

        // 3. 调用IMU更新（dt=0.001f 对应1KHz采样率）
        IMU_Update(0.001f, gx, gy, gz, ax, ay, az);

        // 4. 匹配1KHz采样率（1ms延时）
        rt_thread_mdelay(1);
    }
}

void IMU_init(void)
{
    accgyro_init(&g_icm_accgyro);

    // 创建IMU更新线程
    rt_thread_t imu_thread = rt_thread_create("imu_update", IMU_update_thread_entry, RT_NULL, 1024, 20, 10);
    if (imu_thread != RT_NULL)
        rt_thread_startup(imu_thread);
}
