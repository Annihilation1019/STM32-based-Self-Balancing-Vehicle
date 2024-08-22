#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "main.h"

enum CarState
{
    ControlMode = 'A',
    SetParam,
    Switch_Resolution,
    Save,
    UP = 'G',
    DOWN,
    LEFT,
    RIGHT,
    STOP
};

extern volatile uint8_t Car_State;                                            // 小车状态
extern volatile uint8_t UP_Flag, DOWN_Flag, LEFT_Flag, RIGHT_Flag, STOP_Flag; // 方向控制/调参标志位
extern volatile uint8_t Switch_Resolution_Flag;                               // 分辨率切换标志位
extern volatile uint8_t STOP_Flag;                                            // 立即停止标志位
extern volatile uint8_t Save_Flag;                                            // 保存参数标志位

void Bluetooth_Init(void);

#endif // __BLUETOOTH_H