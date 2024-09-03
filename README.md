# 基于 STM32F103C8T6 的两轮自平衡小车

## 功能简介

- 两轮自平衡，通过 MPU6050 陀螺仪传感器检测姿态，通过 PID 控制算法控制电机转速，实现自平衡功能
- 通过 HC-05 蓝牙模块与手机连接，通过串口通信控制小车前进、后退、左转、右转等动作
- 通过 HC-SR04 超声波传感器检测前方障碍物距离，附带简易均值滤波，可防撞规避
- 通过 OLED 显示屏显示小车状态信息
- 可通过上位机切换控制模式与调参模式，实现实时调参（上电默认控制模式）