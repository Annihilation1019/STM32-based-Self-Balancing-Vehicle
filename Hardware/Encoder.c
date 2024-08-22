#include "Encoder.h"
#include "tim.h"

int Encoder_Left, Encoder_Right; // 编码器左右轮的值

/**
 * @brief 编码器初始化
 */
void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
}

/**
 * @brief 获取编码器的值
 * @param TIMx 编码器的句柄
 * @return 编码器的值
 */
int GetEncoderVal(TIM_HandleTypeDef *TIMx)
{
    int temp;
    temp = (short)__HAL_TIM_GET_COUNTER(TIMx);
    __HAL_TIM_SET_COUNTER(TIMx, 0);
    return temp;
}
