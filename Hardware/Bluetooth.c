#include "Bluetooth.h"
#include "usart.h"
#include "PID.h"
#include "stdio.h"

uint8_t rx_buf[2]; // 串口接收缓存

volatile uint8_t Car_State = ControlMode;                                                  // 小车状态，上电默认为遥控模式
volatile uint8_t UP_Flag = 0, DOWN_Flag = 0, LEFT_Flag = 0, RIGHT_Flag = 0; // 方向控制/调参标志位
volatile int8_t STOP_Flag = 0;                                                             // 立即停止标志位
volatile uint8_t Switch_Resolution_Flag = 0;                                               // 分辨率切换标志位
volatile uint8_t Save_Flag = 0;                                                            // 保存参数标志位

/**
 * @brief  初始化蓝牙串口
 */
void Bluetooth_Init(void)
{
    HAL_UART_Receive_IT(&huart3, rx_buf, 1);
}

/**
 * @brief  串口接收中断回调函数
 * @param  huart 串口句柄
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        HAL_UART_Transmit(&huart3, rx_buf, 1, 0xffff); // 回显
        switch (rx_buf[0])
        {
        case ControlMode: // 控制模式
            Car_State = ControlMode;
            break;
        case SetParam: // 调参模式
            Car_State = SetParam;
            break;
        case Switch_Resolution: // 分辨率切换
            Switch_Resolution_Flag = 1;
            break;
        case Save: // 保存参数
            Save_Flag = 1;
        case UP:
            UP_Flag = 1;
            break;
        case DOWN:
            DOWN_Flag = 1;
            break;
        case LEFT:
            LEFT_Flag = 1;
            break;
        case RIGHT:
            RIGHT_Flag = 1;
            break;
        case STOP:
            STOP_Flag = 1;
            break;
        default:
            break;
        }

        HAL_UART_Receive_IT(&huart3, rx_buf, 1); // 重新开启串口接收中断
    }
}