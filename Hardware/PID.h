#ifndef __PID_H
#define __PID_H

#include "stm32f1xx_hal.h"

#define TURN_MAX 150
#define SPEED_MAX 20

extern float Pitch, Roll, Yaw; // 姿态角
extern volatile float Vertical_Kp;      // 直立环P参数 0 ~ 1000
extern volatile float Vertical_Kd;      // 直立环D参数 0 ~ 10
extern volatile float Velocity_Kp;      // 速度环P参数 0 ~ 1
extern volatile float Velocity_Ki;      // 速度环I参数 根据工程经验 Kp/200
extern volatile float Turn_Kp;          // 转向环P参数
extern volatile float Turn_Kd;          // 转向环D参数
extern volatile float Med_Angle;        // 直立环机械中值角度

void PID_Control(void);

#endif // __PID_H