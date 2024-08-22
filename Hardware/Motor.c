#include "Motor.h"
#include "tim.h"

int abs(int x)
{
    return x > 0 ? x : -x;
}

/**
 * @brief 电机初始化
 */
void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

/**
 * @brief 电机控制
 * @param speed 速度 -7200 ~ 7200
 */
void Motor_Control(int16_t SpeedA, int16_t SpeedB)
{
    if (SpeedA >= 0)
    {
        AIN1_SET();
        AIN2_RESET();
    }
    else
    {
        AIN1_RESET();
        AIN2_SET();
    }
    if (SpeedB >= 0)
    {
        BIN1_SET();
        BIN2_RESET();
    }
    else
    {
        BIN1_RESET();
        BIN2_SET();
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, abs(SpeedA));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, abs(SpeedB));
}

void Motor_Limit(int *MotorA, int *MotorB)
{
    if (*MotorA > PWM_MAX)
        *MotorA = PWM_MAX;
    else if (*MotorA < PWM_MIN)
        *MotorA = PWM_MIN;
    if (*MotorB > PWM_MAX)
        *MotorB = PWM_MAX;
    else if (*MotorB < PWM_MIN)
        *MotorB = PWM_MIN;
}