#ifndef __FLASH_H
#define __FLASH_H

#define FLASH_USER_START_ADDR ((uint32_t)0x0801FC00) /* 最后一页的起始地址 */
#define FLASH_USER_END_ADDR ((uint32_t)0x0801FFFF)   /* 最后一页的结束地址 */

#include "stm32f1xx_hal.h"

void Flash_Read_PID_Params(float *params, uint32_t length);
void Flash_Write_PID_Params(float *params, uint32_t length);

#endif // __FLASH_H