#include "flash.h"
#include "stm32f1xx_hal.h"

/**
 * @brief 将PID参数写入Flash
 *
 * @param params PID参数缓存数组
 * @param length 数组长度
 */
void Flash_Write_PID_Params(float *params, uint32_t length)
{
    if (length > 9)
    {
        // 参数长度超过9，返回错误
        return;
    }
    HAL_FLASH_Unlock();

    /* 擦除指定页 */
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
    EraseInitStruct.NbPages = 1;
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
    {
        // 错误处理
        HAL_FLASH_Lock();
        return;
    }

    /* 写入数据 */
    for (uint8_t i = 0; i < length; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_USER_START_ADDR + i * 4, *(uint32_t *)&params[i]) != HAL_OK)
        {
            // 错误处理
            HAL_FLASH_Lock();
            return;
        }
    }
    HAL_FLASH_Lock();
}

/**
 * @brief 从Flash读取PID参数
 *
 * @param params 用于存储读取数据的数组
 * @param length 数组长度
 */
void Flash_Read_PID_Params(float *params, uint32_t length)
{
    if (length > 9)
    {
        // 参数长度超过9，返回错误
        return;
    }

    for (uint8_t i = 0; i < length; i++)
    {
        params[i] = *(float *)(FLASH_USER_START_ADDR + i * 4);
    }
}