#include "HC_SR04.h"

volatile float distance;                      // 距离
float distance_buffer[MEASUREMENTS] = {0.0f}; // 距离滤波缓存
uint8_t dis_index = 0;                        // 距离滤波索引
__IO uint32_t sys_tick = 0;                   // 系统滴答
extern __IO uint32_t uwTick;

#define CPU_FREQUENCY_MHZ 72 // STM32时钟主频

float abs_float(float a)
{
    return a >= 0 ? a : -a;
}

void delay_us(__IO uint32_t delay)
{
    int last, curr, val;
    int temp;

    while (delay != 0)
    {
        temp = delay > 900 ? 900 : delay;
        last = SysTick->VAL;
        curr = last - CPU_FREQUENCY_MHZ * temp;
        if (curr >= 0)
        {
            do
            {
                val = SysTick->VAL;
            } while ((val < last) && (val >= curr));
        }
        else
        {
            curr += CPU_FREQUENCY_MHZ * 1000;
            do
            {
                val = SysTick->VAL;
            } while ((val <= last) || (val > curr));
        }
        delay -= temp;
    }
}

/**
 * @brief HC-SR04测距
 * @note 防止发射信号对回波信号的影响，测距间隔应大于60ms
 */
void HC_SR04_GetDistance(void)
{
    if (uwTick - sys_tick < 80) // 80ms执行一次
        return;
    sys_tick = uwTick;
    HC_SR04_TRIG_HIGH();
    delay_us(12);
    HC_SR04_TRIG_LOW();
}

/* 不好用，动态响应差，静态响应一般 */
// /**
//  * @brief HC-SR04测距滤波函数
//  * @note 取连续3次测距的平均值，丢弃误差较大的值
//  */
// void HC_SR04_Filter(void)
// {
//     static float distance_sum = 0.0f;
//     static uint8_t flag = 0;
//     if (dis_index == MEASUREMENTS) // 3次测距完成
//     {
//         distance = distance_sum / MEASUREMENTS;
//         dis_index = 0;
//         distance_sum = 0.0f;
//         flag = 1;
//     }
//     else
//     {
//         distance_buffer[dis_index] = (float)__HAL_TIM_GetCounter(&htim3) * 0.017;
//         if (dis_index > 0 && abs_float(distance_buffer[dis_index] - distance_buffer[dis_index - 1]) > ERR_MAX) // 丢弃误差较大的值
//         {
//             distance_buffer[dis_index] = distance_buffer[dis_index - 1]; // 使用上一次的值
//         }
//         else if (dis_index == 0 && abs_float(distance_buffer[0] - distance_buffer[MEASUREMENTS - 1]) > ERR_MAX && flag == 1) // 丢弃误差较大的值
//         {
//             distance_buffer[0] = distance_buffer[MEASUREMENTS - 1]; // 使用上一次的值
//         }

//         distance_sum += distance_buffer[dis_index];
//         dis_index++;
//     }
// }

/**
 * @brief HC-SR04中断回调函数
 * @param GPIO_Pin 触发中断的引脚
 * @note 上升沿触发开始计时，下降沿触发结束计时
 */
// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
// {
//     if(HC_SR04_ECHO_isHIGH()) // 上升沿中断
//     {
//         __HAL_TIM_SetCounter(&htim3, 0);
//         HAL_TIM_Base_Start(&htim3);
//     }
//     else if(HC_SR04_ECHO_isLOW()) // 下降沿中断
//     {
//         HAL_TIM_Base_Stop(&htim3);
//         distance = (float)__HAL_TIM_GetCounter(&htim3) / 1e6 * 340 * 100 / 2;
//     }
// }