#ifndef __HC_SR04_H
#define __HC_SR04_H

#include "stm32f1xx_hal.h"
#include "main.h"

#define HC_SR04_TRIG_HIGH() HAL_GPIO_WritePin(HC_SR04_Trig_GPIO_Port, HC_SR04_Trig_Pin, GPIO_PIN_SET)
#define HC_SR04_TRIG_LOW() HAL_GPIO_WritePin(HC_SR04_Trig_GPIO_Port, HC_SR04_Trig_Pin, GPIO_PIN_RESET)
#define HC_SR04_ECHO_isHIGH() HAL_GPIO_ReadPin(HC_SR04_Echo_GPIO_Port, HC_SR04_Echo_Pin) == GPIO_PIN_SET
#define HC_SR04_ECHO_isLOW() HAL_GPIO_ReadPin(HC_SR04_Echo_GPIO_Port, HC_SR04_Echo_Pin) == GPIO_PIN_RESET

#define ERR_MAX 20 // 误差阈值
#define MEASUREMENTS 3 // 测量次数

extern TIM_HandleTypeDef htim3;
extern volatile float distance;

void HC_SR04_GetDistance(void);
void HC_SR04_Filter(void);

#endif // __HC_SR04_H