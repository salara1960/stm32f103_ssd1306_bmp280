/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>

#include "stm32f1xx_hal_uart.h"
//#include "stm32f1xx_hal_rtc.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
	ZERO_DOWN = 0,
	ZERO_UP,
	COLOR_ALL_UP,
	COLOR_ALL_DOWN,
	COLOR_DOWN,
	COLOR_UP,
	ZERO_ALL_UP,
	ZERO_ALL_DOWN
} dir_mode_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

I2C_HandleTypeDef hi2c2;
UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim1;

HAL_StatusTypeDef i2cError;

const uint32_t min_wait_ms;
const uint32_t max_wait_ms;

uint8_t HalfsecCounter;

uint8_t GoTxDMA;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define wait_sensor_def 10 // 1 * 10 = 10 sec
#define MAX_UART_BUF 256
#define DEF_COLOR 64

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

int sec_to_str_time(uint32_t sec, char *stx);

void inc_secCounter();
uint32_t get_secCounter();
uint32_t get_tmr(uint32_t sec);
bool check_tmr(uint32_t sec);

void Report(const char *txt, bool addCRLF, bool addTime);
void errLedOn(const char *from);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWM_Pin GPIO_PIN_1
#define PWM_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_10
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_11
#define SDA_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_12
#define LED2_GPIO_Port GPIOB
#define TXD_Pin GPIO_PIN_9
#define TXD_GPIO_Port GPIOA
#define RXD_Pin GPIO_PIN_10
#define RXD_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

#undef SET_SSD1306_INVERT

#define LED_ERROR GPIO_PIN_12

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
