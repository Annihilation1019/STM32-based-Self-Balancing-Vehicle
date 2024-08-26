/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "stdio.h"
#include "HC_SR04.h"
#include "Motor.h"
#include "Encoder.h"
#include "Bluetooth.h"
#include "PID.h"
#include "flash.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PID_buf_length 7
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char display_buffer[30];             // 显示缓存
uint8_t PID_Flag = 0;                // PID控制触发标志
float PID_Param_buf[PID_buf_length]; // PID参数缓存
int8_t PID_buf_index = 0;            // PID参数索引
float variation = 100.0f;            // PID参数变化量
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  MPU_Init();     // 初始化MPU6050
  mpu_dmp_init(); // 初始化DMP姿态解算
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  Motor_Init();                                         // 初始化电机
  Encoder_Init();                                       // 初始化编码器
  Bluetooth_Init();                                     // 初始化蓝牙
  Flash_Read_PID_Params(PID_Param_buf, PID_buf_length); // 从Flash读取PID参数

  OLED_ShowString(1, 1, "Initial success!", OLED_8x16);
  HAL_Delay(500);
  OLED_Clear();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* MPU6050 */
    sprintf(display_buffer, "Pit:%.2f  ", Pitch);    // 将Pitch转换为字符串
    OLED_ShowString(1, 1, display_buffer, OLED_8x6); // 显示Pitch
    sprintf(display_buffer, "Rol:%.2f  ", Roll);     // 将Roll转换为字符串
    OLED_ShowString(2, 1, display_buffer, OLED_8x6); // 显示Roll
    sprintf(display_buffer, "Yaw:%.2f  ", Yaw);      // 将Yaw转换为字符串
    OLED_ShowString(3, 1, display_buffer, OLED_8x6); // 显示Yaw

    /* HC-SR04 */
    // HC_SR04_GetDistance();                             // 获取HC-SR04距离
    sprintf(display_buffer, "Dis:%.2fcm  ", distance); // 将距离转换为字符串
    OLED_ShowString(4, 1, display_buffer, OLED_8x6);   // 显示距离

    /* 霍尔编码器 */
    sprintf(display_buffer, "L:%d R:%d   ", Encoder_Left, Encoder_Right); // 将编码器的值转换为字符串
    OLED_ShowString(5, 1, display_buffer, OLED_8x6);                      // 显示编码器的值

    /* 显示PID参数 */
    Vertical_Kp = PID_Param_buf[0];
    Vertical_Kd = PID_Param_buf[1];
    Velocity_Kp = PID_Param_buf[2];
    Velocity_Ki = PID_Param_buf[3];
    Turn_Kp = PID_Param_buf[4];
    Turn_Kd = PID_Param_buf[5];
    Med_Angle = PID_Param_buf[6];

    sprintf(display_buffer, "Kp:%.2f Kd:%.2f  ", Vertical_Kp, Vertical_Kd); // 将PID参数转换为字符串
    OLED_ShowString(6, 1, display_buffer, OLED_8x6);                        // 显示PID参数
    sprintf(display_buffer, "Kp:%.2f Kd:%.3f   ", Velocity_Kp, Velocity_Ki);
    OLED_ShowString(7, 1, display_buffer, OLED_8x6);
    sprintf(display_buffer, "Kp:%.2f Kd:%.2f  ", Turn_Kp, Turn_Kd);
    OLED_ShowString(8, 1, display_buffer, OLED_8x6);
    sprintf(display_buffer, "MA:%.2f", Med_Angle);
    OLED_ShowString(1, 14, display_buffer, OLED_8x6);
    sprintf(display_buffer, "Index:%d", PID_buf_index);
    OLED_ShowString(2, 14, display_buffer, OLED_8x6);
    sprintf(display_buffer, "V%.3f", variation);
    OLED_ShowString(3, 14, display_buffer, OLED_8x6);

    /* 为确保PID控制函数中读取传感器数据周期的准确性，将PID_Control();放置在MPU6050_INT引脚触发的外部中断回调函数中，10ms执行一次 */

    /* 蓝牙控制器 */
    /* 调参模式放置在主函数，控制模式要求及时响应，放置在中断回调函数 */
    if (Car_State == SetParam) // 调参模式
    {
      if (UP_Flag)
      {
        PID_buf_index--;
        if (PID_buf_index < 0)
        {
          PID_buf_index = 6;
        }
        sprintf(display_buffer, "%d", PID_buf_index);
        HAL_UART_Transmit(&huart3, (uint8_t *)display_buffer, 1, 0xffff);
        UP_Flag = 0;
      }
      else if (DOWN_Flag)
      {
        PID_buf_index++;
        PID_buf_index %= 7; // 循环索引
        sprintf(display_buffer, "%d", PID_buf_index);
        HAL_UART_Transmit(&huart3, (uint8_t *)PID_buf_index, 1, 0xffff);
        DOWN_Flag = 0;
      }
      else if (LEFT_Flag)
      {
        PID_Param_buf[PID_buf_index] -= variation;
        LEFT_Flag = 0;
      }
      else if (RIGHT_Flag)
      {
        PID_Param_buf[PID_buf_index] += variation;
        RIGHT_Flag = 0;
      }
      else if (Switch_Resolution_Flag)
      {
        variation /= 10.0;
        if (variation <= 0.0001)
        {
          variation = 100.0;
        }
        Switch_Resolution_Flag = 0;
      }
    }
    /* Flash写入PID控制参数 */
    if (Save_Flag)
    {
      Flash_Write_PID_Params(PID_Param_buf, PID_buf_length);
      OLED_ShowString(5, 20, "!", OLED_8x6);
      Save_Flag = 0;
    }
    else
    {
      OLED_ShowString(5, 20, " ", OLED_8x6);
    }
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == HC_SR04_Echo_Pin)
  {
    if (HC_SR04_ECHO_isHIGH()) // 上升沿中断
    {
      __HAL_TIM_SetCounter(&htim3, 0);
      HAL_TIM_Base_Start(&htim3);
    }
    else if (HC_SR04_ECHO_isLOW()) // 下降沿中断
    {
      HAL_TIM_Base_Stop(&htim3);
      distance = (float)__HAL_TIM_GetCounter(&htim3) / 1e6 * 340 * 100 / 2;
    }
  }
  if (GPIO_Pin == MPU6050_INT_Pin)
  {
    PID_Control();         // PID主控制器
    HC_SR04_GetDistance(); // 获取HC-SR04距离
  }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
