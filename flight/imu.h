#ifndef _IMU_H_
#define _IMU_H_

#include <rtthread.h>

#define M_PI 3.1415926f

// IMU PI参数 (可调试修改)
extern float imu_Kp;
extern float imu_Ki;

// 初始化惯导系统
void IMU_init(void);

// 调试命令
void imu_debug_cmd(char *cmd);

#endif
