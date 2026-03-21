#ifndef _IMU_H_
#define _IMU_H_

#include <rtthread.h>

#define M_PI 3.1415926f

// 初始化惯导系统
void IMU_init(void);

// 外部信号量声明 (供中断处理函数使用)
extern rt_sem_t imu_sem;

#endif
