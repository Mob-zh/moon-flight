#include "flight_control.h"
#include "at32f435_437_wk_config.h"
#include "dshot600.h"
#include "elrs.h"
#include "imu.h"
#include <math.h>
#include <rtthread.h>
#include <string.h>

rt_sem_t control_sem;
// 全局飞行控制实例
flight_control_t g_flight_control;

static void flight_control_thread_entry(void *parameter);

// ==================== 常量 ====================
// #define MAX_ANGLE    45.0f // Angle模式最大角度 (deg)
#define THROTTLE_MIN 0.1f // 最小油门
#define THROTTLE_MAX 1.0f // 最大油门
#define PID_SEND_DIV 2    // PID参数分频（每2次发送一次，即4Hz）
// 可调的角速度上限
float   rate_limit_max  = 667.0f;
float   roll_threshold  = 0.002f;
float   pitch_threshold = 0.002f;
float   yaw_threshold   = 0.004f;
uint8_t MAX_ANGLE       = 45;

// ==================== 工具函数 ====================

// PID计算
static float pid_calculate(pid_controller_t *pid, float target, float current, float dt)
{
    // 计算误差
    float error = target - current;

    // 积分项
    pid->sum += error * dt;

    // 微分项
    float derivative = (error - pid->prev) / dt;
    pid->prev        = error;

    // PID输出
    pid->output = pid->kp * error + pid->ki * pid->sum + pid->kd * derivative;

    // 输出限幅
    if (pid->output > pid->limit)
        pid->output = pid->limit;
    if (pid->output < -pid->limit)
        pid->output = -pid->limit;

    return pid->output;
}

// PID重置
static void pid_reset(pid_controller_t *pid)
{
    pid->sum    = 0.0f;
    pid->prev   = 0.0f;
    pid->output = 0.0f;
}

// PID初始化
static void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float limit)
{
    pid->kp    = kp;
    pid->ki    = ki;
    pid->kd    = kd;
    pid->limit = limit;
    pid_reset(pid);
}

// 限幅
static float constrain(float value, float min_val, float max_val)
{
    if (value > max_val)
        return max_val;
    if (value < min_val)
        return min_val;
    return value;
}

// ==================== API实现 ====================

void flight_control_init(flight_control_t *fc)
{
    // 清零结构体
    memset(fc, 0, sizeof(flight_control_t));

    // 默认模式
    fc->mode      = FLIGHT_MODE_ACRO;
    fc->prev_mode = FLIGHT_MODE_ACRO;

    // 初始化PID参数
    // 角度环
    pid_init(&fc->pid_angle_roll, 0.00f, 0.0f, 0.0f, MAX_ANGLE);
    pid_init(&fc->pid_angle_pitch, 0.00f, 0.0f, 0.0f, MAX_ANGLE);
    pid_init(&fc->pid_angle_yaw, 0.00f, 0.0f, 0.0f, MAX_ANGLE);

    // 角速度环
    pid_init(&fc->pid_rate_roll, 0.005f, 0.0f, 0.0f, rate_limit_max);
    pid_init(&fc->pid_rate_pitch, 0.005f, 0.0f, 0.0f, rate_limit_max);
    pid_init(&fc->pid_rate_yaw, 0.0000f, 0.00000f, 0.0000000f, rate_limit_max);

    // 位置环
    pid_init(&fc->pid_pos_n, 0.0f, 0.0f, 0.0f, 0.0);
    pid_init(&fc->pid_pos_e, 0.0f, 0.0f, 0.0f, 0.0);
    pid_init(&fc->pid_alt, 0.0f, 0.0f, 0.0f, 0.0);

    // 电机输出初始化
    for (int i = 0; i < 4; i++)
    {
        fc->motor[i] = 0.0f;
    }

    control_sem = rt_sem_create("control_sem", 0, RT_IPC_FLAG_PRIO);

    // 初始化控制线程
    // 创建飞行控制线程
    rt_thread_t ctrl_thread = rt_thread_create("flight_ctrl",
                                               flight_control_thread_entry,
                                               RT_NULL,
                                               2048,
                                               11,
                                               10);
    if (ctrl_thread != RT_NULL)
        rt_thread_startup(ctrl_thread);

    rt_kprintf("[FC] Flight control initialized\n");
}

void flight_control_update(flight_control_t *fc)
{
    // 未解锁时只更新期望值，不计算PID
    if (!fc->armed)
    {
        for (int i = 0; i < 4; i++)
        {
            fc->motor[i] = 0.0f;
        }
        return;
    }

    float dt = 0.000125f; // 8kHz = 0.000125s

    // 根据模式处理
    switch (fc->mode)
    {
    case FLIGHT_MODE_ACRO:
        // Acro模式：直接使用遥控器输入作为角速度目标
        fc->desired_rate_roll  = fc->rc_roll * rate_limit_max;
        fc->desired_rate_pitch = fc->rc_pitch * rate_limit_max;
        fc->desired_rate_yaw   = fc->rc_yaw * rate_limit_max;
        break;

    case FLIGHT_MODE_ANGLE:
    case FLIGHT_MODE_HOLD:
    {
        // 角度限幅
        float cmd_roll  = constrain(fc->rc_roll, -1.0f, 1.0f) * MAX_ANGLE;
        float cmd_pitch = constrain(fc->rc_pitch, -1.0f, 1.0f) * MAX_ANGLE;

        // 角度环PID（目标角度 -> 期望角速度）
        fc->desired_rate_roll  = pid_calculate(&fc->pid_angle_roll, cmd_roll, fc->actual_roll, dt);
        fc->desired_rate_pitch = pid_calculate(&fc->pid_angle_pitch, cmd_pitch, fc->actual_pitch, dt);
        fc->desired_rate_yaw   = fc->rc_yaw * rate_limit_max; // Yaw保持Rate模式
    }
    break;
    }

    // GPS Hold模式：添加位置环
    if (fc->mode == FLIGHT_MODE_HOLD && fc->ekf_ready)
    {
        // 位置环PID
        float pos_error_n = fc->desired_pos_n - fc->pos_n;
        float pos_error_e = fc->desired_pos_e - fc->pos_e;

        fc->desired_rate_roll += pid_calculate(&fc->pid_pos_n, 0, pos_error_n, dt);
        fc->desired_rate_pitch += pid_calculate(&fc->pid_pos_e, 0, pos_error_e, dt);
    }

    // 角速度环PID（期望角速度 -> 电机混合）
    float pid_roll  = pid_calculate(&fc->pid_rate_roll, fc->desired_rate_roll, fc->gyro_roll, dt);
    float pid_pitch = pid_calculate(&fc->pid_rate_pitch, fc->desired_rate_pitch, fc->gyro_pitch, dt);
    float pid_yaw   = pid_calculate(&fc->pid_rate_yaw, fc->desired_rate_yaw, fc->gyro_yaw, dt);

    // 电机混合（X型布局）
    float throttle = constrain(fc->rc_throttle, THROTTLE_MIN, THROTTLE_MAX);

    // X型布局:
    //   FL(0)   FR(1)
    //       ^
    //       |
    //   RL(2)   RR(3)
    fc->motor[0] = throttle + pid_roll + pid_pitch - pid_yaw; // FL
    fc->motor[1] = throttle - pid_roll + pid_pitch + pid_yaw; // FR
    fc->motor[2] = throttle + pid_roll - pid_pitch + pid_yaw; // RL
    fc->motor[3] = throttle - pid_roll - pid_pitch - pid_yaw; // RR

    // 电机输出限幅 (0-1)
    for (int i = 0; i < 4; i++)
    {
        fc->motor[i] = constrain(fc->motor[i], 0.0f, 1.0f);
    }
}

void flight_control_set_mode(flight_control_t *fc, flight_mode_t mode)
{
    if (mode >= FLIGHT_MODE_MAX)
        return;

    fc->prev_mode = fc->mode;
    fc->mode      = mode;

    // 切换模式时重置PID
    if (fc->mode != fc->prev_mode)
    {
        pid_reset(&fc->pid_angle_roll);
        pid_reset(&fc->pid_angle_pitch);
        pid_reset(&fc->pid_angle_yaw);
        pid_reset(&fc->pid_rate_roll);
        pid_reset(&fc->pid_rate_pitch);
        pid_reset(&fc->pid_rate_yaw);
    }
}

void flight_control_set_rc(flight_control_t *fc,
                           float roll, float pitch,
                           float yaw, float throttle)
{
    fc->rc_roll     = roll;
    fc->rc_pitch    = pitch;
    fc->rc_yaw      = yaw;
    fc->rc_throttle = throttle;
}

void flight_control_set_attitude(flight_control_t *fc,
                                 float roll, float pitch, float yaw)
{
    fc->actual_roll  = roll;
    fc->actual_pitch = pitch;
    fc->actual_yaw   = yaw;
}

void flight_control_set_gyro(flight_control_t *fc,
                             float roll, float pitch, float yaw)
{
    fc->gyro_roll  = roll;
    fc->gyro_pitch = pitch;
    fc->gyro_yaw   = yaw;
}

void flight_control_set_position(flight_control_t *fc,
                                 float north, float east, float alt)
{
    fc->pos_n   = north;
    fc->pos_e   = east;
    fc->pos_alt = alt;
}

void flight_control_set_armed(flight_control_t *fc, uint8_t armed)
{
    fc->armed = armed;

    if (!armed)
    {
        // 解锁时重置所有PID
        pid_reset(&fc->pid_angle_roll);
        pid_reset(&fc->pid_angle_pitch);
        pid_reset(&fc->pid_angle_yaw);
        pid_reset(&fc->pid_rate_roll);
        pid_reset(&fc->pid_rate_pitch);
        pid_reset(&fc->pid_rate_yaw);
        pid_reset(&fc->pid_pos_n);
        pid_reset(&fc->pid_pos_e);
        pid_reset(&fc->pid_alt);
    }
}

void flight_control_get_motor(flight_control_t *fc, float *motor)
{
    for (int i = 0; i < 4; i++)
    {
        motor[i] = fc->motor[i];
    }
}

void flight_control_set_pid(flight_control_t *fc, int type, int axis,
                            float kp, float ki, float kd)
{
    pid_controller_t *pid = NULL;

    // 选择PID组
    if (type == 0)
    { // angle
        if (axis == 0)
            pid = &fc->pid_angle_roll;
        else if (axis == 1)
            pid = &fc->pid_angle_pitch;
        else if (axis == 2)
            pid = &fc->pid_angle_yaw;
    }
    else if (type == 1)
    { // rate
        if (axis == 0)
            pid = &fc->pid_rate_roll;
        else if (axis == 1)
            pid = &fc->pid_rate_pitch;
        else if (axis == 2)
            pid = &fc->pid_rate_yaw;
    }
    else if (type == 2)
    { // pos
        if (axis == 0)
            pid = &fc->pid_pos_n;
        else if (axis == 1)
            pid = &fc->pid_pos_e;
    }

    if (pid)
    {
        pid->kp = kp;
        pid->ki = ki;
        pid->kd = kd;
    }
}

// ==================== 控制线程 ====================

// 电机输出转换：将0-1映射到DSHOT throttle值
// armed: 1 = 解锁, 0 = 锁定
static uint16_t motor_to_dshot(float motor_value, uint8_t armed)
{
    // 锁定状态：发送0（电机停止）
    if (!armed)
    {
        return 0; // DSHOT_CMD_MOTOR_STOP
    }

    // 解锁状态
    // 5%油门怠速 = 0.05 * (2047-48) + 48 ≈ 144
    if (motor_value < 0.05f)
    {
        motor_value = 0.05f;
    }

    // DSHOT范围: 48 = min throttle, 2047 = max throttle
    // 映射 0-1 -> 48-2047
    uint16_t dshot_value = (uint16_t)(motor_value * (2047 - 48) + 48);
    if (dshot_value > 2047)
        dshot_value = 2047;

    return dshot_value;
}

// 发送DSHOT命令5次（确保电调接收）
static void dshot600_send_cmd_5times(uint8_t channel, uint16_t cmd)
{
    for (int i = 0; i < 5; i++)
    {
        dshot600_send_packet(channel, cmd);
    }
}

extern uint16_t imu_cnt;
extern uint16_t pid_cnt;

static void flight_control_thread_entry(void *parameter)
{
    flight_control_t *fc = &g_flight_control;
    extern elrsDev_t  g_elrs_receiver;
    int8_t            imu_cnt = 0;

    while (1)
    {
        rt_sem_take(control_sem, RT_WAITING_FOREVER);

        // 准备传感器数据
        IMU_Prepare_Data();

        // 执行姿态解算
        IMUupdate(&Gyr_filt, &Acc_filt, &Att_Angle);
        imu_cnt++;

        if (imu_cnt == PID_SEND_DIV)
        {

            // 获取遥控器输入（标准化到 -1 ~ 1）
            float roll     = g_elrs_receiver.ch1_roll / 1000.0f;
            float pitch    = g_elrs_receiver.ch2_pitch / 1000.0f;
            float yaw      = g_elrs_receiver.ch4_yaw / 1000.0f;
            float throttle = (g_elrs_receiver.ch3_throttle + 1000) / 2000.0f; // 0 ~ 1
            // 设置遥控器输入
            flight_control_set_rc(fc, roll, pitch, yaw, throttle);

            // 解锁检测
            if (g_elrs_receiver.ch5_arm > 500)
            {
                flight_control_set_armed(fc, 1);
            }
            else
            {
                flight_control_set_armed(fc, 0);
            }

            // 获取当前姿态和角速度
            extern FLOAT_ANGLE Att_Angle;
            extern FLOAT_XYZ   Gyr_filt;
            flight_control_set_attitude(fc, Att_Angle.rol, Att_Angle.pit, Att_Angle.yaw);

            // 弧度小于limit/秒，认为是静止
            if ((Gyr_filt.X < pitch_threshold) && (Gyr_filt.X > -pitch_threshold))
            {
                Gyr_filt.X = 0;
            }
            if ((Gyr_filt.Y < roll_threshold) && (Gyr_filt.Y > -roll_threshold))
            {
                Gyr_filt.Y = 0;
            }
            if ((Gyr_filt.Z < yaw_threshold) && (Gyr_filt.Z > -yaw_threshold))
            {
                Gyr_filt.Z = 0;
            }

            flight_control_set_gyro(fc, Gyr_filt.X * 57.3f, -(Gyr_filt.Y * 57.3f), Gyr_filt.Z * 57.3f);

            // 执行PID控制运算
            flight_control_update(fc);

            // 获取电机输出并发送到DSHOT
            float motor[4];
            flight_control_get_motor(fc, motor);

            dshot600_send_packet(0, motor_to_dshot(motor[3], fc->armed)); // M4
            dshot600_send_packet(1, motor_to_dshot(motor[2], fc->armed)); // M3
            dshot600_send_packet(2, motor_to_dshot(motor[1], fc->armed)); // M2
            dshot600_send_packet(3, motor_to_dshot(motor[0], fc->armed)); // M1
            imu_cnt = 0;
        }

        // 发送测试电调顺序
        // dshot600_send_packet(0, 0x00); // M4
        // dshot600_send_packet(1, 0x01); // M3
        // dshot600_send_packet(2, 0x02); // M2
        // dshot600_send_packet(3, 0x03); // M1
    }
}

// ==================== MSH命令 ====================

// 武装/解缆命令
static void fc_arm(int argc, char *argv[])
{
    if (argc < 2)
    {
        rt_kprintf("Usage: fc_arm <0|1>\n");
        rt_kprintf("  0 = disarmed (锁定)\n");
        rt_kprintf("  1 = armed (解锁)\n");
        return;
    }

    uint8_t arm = atoi(argv[1]);
    flight_control_set_armed(&g_flight_control, arm);
    rt_kprintf("Flight control %s\n", arm ? "ARMED" : "DISARMED");
}

// 蜂鸣器测试命令
static void fc_beep(int argc, char *argv[])
{
    if (argc < 2)
    {
        rt_kprintf("Usage: fc_beep <1-5>\n");
        rt_kprintf("  1-5 = beep type\n");
        return;
    }

    uint8_t beep_type = atoi(argv[1]);
    if (beep_type < 1 || beep_type > 5)
    {
        rt_kprintf("Invalid beep type (1-5)\n");
        return;
    }

    // 发送5次确保电调接收
    for (int ch = 0; ch < 4; ch++)
    {
        dshot600_send_cmd_5times(ch, DSHOT_CMD_BEEP_1 - 1 + beep_type);
    }
    rt_kprintf("Beep %d sent\n", beep_type);
}

// 电机换向命令
static void fc_dir(int argc, char *argv[])
{
    if (argc < 2)
    {
        rt_kprintf("Usage: fc_dir <1|2>\n");
        rt_kprintf("  1 = direction 1\n");
        rt_kprintf("  2 = direction 2\n");
        return;
    }

    uint8_t direction = atoi(argv[1]);
    if (direction < 1 || direction > 2)
    {
        rt_kprintf("Invalid direction (1-2)\n");
        return;
    }

    // 发送5次确保电调接收
    for (int ch = 0; ch < 4; ch++)
    {
        if (direction == 1)
        {
            dshot600_send_cmd_5times(ch, DSHOT_CMD_ROTATION_DIR_1);
        }
        else
        {
            dshot600_send_cmd_5times(ch, DSHOT_CMD_ROTATION_DIR_2);
        }
    }
    rt_kprintf("Rotation direction %d sent\n", direction);
}

// 电机测试命令（按通道）
static void fc_motor(int argc, char *argv[])
{
    if (argc < 3)
    {
        rt_kprintf("Usage: fc_motor <channel> <throttle>\n");
        rt_kprintf("  channel: 0-3\n");
        rt_kprintf("  throttle: 0-100 (%%)\n");
        return;
    }

    uint8_t channel  = atoi(argv[1]);
    uint8_t throttle = atoi(argv[2]);

    if (channel > 3 || throttle > 100)
    {
        rt_kprintf("Invalid parameter\n");
        return;
    }

    // 转换为DSHOT值
    uint16_t dshot_val = (throttle * (2047 - 48) / 100) + 48;
    dshot600_send_packet(channel, dshot_val);
    rt_kprintf("Motor %d: throttle %d%% (DSHOT=%d)\n", channel, throttle, dshot_val);
}

MSH_CMD_EXPORT(fc_arm, "arm/disarm flight control");
MSH_CMD_EXPORT(fc_beep, "send beep command to ESC");
MSH_CMD_EXPORT(fc_dir, "set motor rotation direction");
MSH_CMD_EXPORT(fc_motor, "test motor with throttle");

// ==================== 角速度环参数设置命令 ====================

// 设置角速度环Kp值
static void fc_set_rate_kp(int argc, char *argv[])
{
    if (argc < 4)
    {
        rt_kprintf("Usage: fc_set_rate_kp <roll> <pitch> <yaw>\n");
        rt_kprintf("  Example: fc_set_rate_kp 2.0 -2.0 2.5\n");
        return;
    }

    g_flight_control.pid_rate_roll.kp  = atof(argv[1]);
    g_flight_control.pid_rate_pitch.kp = atof(argv[2]);
    g_flight_control.pid_rate_yaw.kp   = atof(argv[3]);

    char str[64];
    snprintf(str, sizeof(str), "Rate Kp set: Roll=%.2f, Pitch=%.2f, Yaw=%.2f\n",
             g_flight_control.pid_rate_roll.kp,
             g_flight_control.pid_rate_pitch.kp,
             g_flight_control.pid_rate_yaw.kp);
    rt_kprintf(str);
}

// 设置角速度上限
static void fc_set_rate_limit(int argc, char *argv[])
{
    if (argc < 2)
    {
        rt_kprintf("Usage: fc_set_rate_limit <deg/s>\n");
        rt_kprintf("  Current: %.1f deg/s\n", rate_limit_max);
        rt_kprintf("  Example: fc_set_rate_limit 500\n");
        return;
    }

    rate_limit_max = atof(argv[1]);

    char str[64];
    snprintf(str, sizeof(str), "Rate limit set to: %.1f deg/s\n", rate_limit_max);
    rt_kprintf(str);
}

// 设置角速度环Ki值
static void fc_set_rate_ki(int argc, char *argv[])
{
    if (argc < 4)
    {
        rt_kprintf("Usage: fc_set_rate_ki <roll> <pitch> <yaw>\n");
        rt_kprintf("  Example: fc_set_rate_ki 0.05 0.05 0.1\n");
        return;
    }

    g_flight_control.pid_rate_roll.ki  = atof(argv[1]);
    g_flight_control.pid_rate_pitch.ki = atof(argv[2]);
    g_flight_control.pid_rate_yaw.ki   = atof(argv[3]);

    char str[64];
    snprintf(str, sizeof(str), "Rate Ki set: Roll=%.3f, Pitch=%.3f, Yaw=%.3f\n",
             g_flight_control.pid_rate_roll.ki,
             g_flight_control.pid_rate_pitch.ki,
             g_flight_control.pid_rate_yaw.ki);
    rt_kprintf(str);
}

// 设置角速度环Kd值
static void fc_set_rate_kd(int argc, char *argv[])
{
    if (argc < 4)
    {
        rt_kprintf("Usage: fc_set_rate_kd <roll> <pitch> <yaw>\n");
        rt_kprintf("  Example: fc_set_rate_kd 0.1 0.1 0.15\n");
        return;
    }

    g_flight_control.pid_rate_roll.kd  = atof(argv[1]);
    g_flight_control.pid_rate_pitch.kd = atof(argv[2]);
    g_flight_control.pid_rate_yaw.kd   = atof(argv[3]);

    char str[64];
    snprintf(str, sizeof(str), "Rate Kd set: Roll=%.3f, Pitch=%.3f, Yaw=%.3f\n",
             g_flight_control.pid_rate_roll.kd,
             g_flight_control.pid_rate_pitch.kd,
             g_flight_control.pid_rate_yaw.kd);
    rt_kprintf(str);
}

MSH_CMD_EXPORT(fc_set_rate_kp, "set rate PID Kp");
MSH_CMD_EXPORT(fc_set_rate_limit, "set rate limit");
MSH_CMD_EXPORT(fc_set_rate_ki, "set rate PID Ki");
MSH_CMD_EXPORT(fc_set_rate_kd, "set rate PID Kd");
