#ifndef _POSITION_INIT_H_
#define _POSITION_INIT_H_

#include <stdbool.h>

/**
 * @brief 位置传感器初始化（气压计 + GPS + EKF + 读取线程）
 */
void position_sensor_init(void);

#endif /* _POSITION_INIT_H_ */