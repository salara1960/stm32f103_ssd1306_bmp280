#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif


//----------------------------------------------------------------------------

#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_uart.h"

//----------------------------------------------------------------------------

#define DISPLAY
#define SET_BMP
#define SET_UART

/*
#include "stm32f1xx_hal_def.h"
typedef enum
{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;


#ifndef HAL_OK
	#define HAL_OK      0x00U
#endif
#ifndef HAL_ERROR
	#define HAL_ERROR   0X01U
#endif
#ifndef HAL_BUSY
	#define HAL_BUSY    0x02U
#endif
#ifndef HAL_TIMEOUT
	#define HAL_TIMEOUT 0x03U
#endif

*/

#define wait_tick_def 1000
#ifdef SET_BMP
	#define wait_sensor_def 5000
#endif


//LEDs
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_12//1
#define LED2_GPIO_Port GPIOB
#define LED_ERROR LED2_Pin
//I2C
#define SCL_Pin GPIO_PIN_10
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_11
#define SDA_GPIO_Port GPIOB

#ifdef SET_UART
	//UART1
	#define TXD_Pin GPIO_PIN_9
	#define TXD_GPIO_Port GPIOA
	#define RXD_Pin GPIO_PIN_10
	#define RXD_GPIO_Port GPIOA
#endif

//----------------------------------------------------------------------------

#pragma pack(push,1)
    typedef struct {
        double temp;// DegC
        double pres;// mmHg
        double humi;// %rH
    } result_t;
#pragma pack(pop)

//----------------------------------------------------------------------------

extern I2C_HandleTypeDef hi2c2;
extern HAL_StatusTypeDef i2cError;
extern UART_HandleTypeDef huart1;

extern result_t sensors;

extern const uint32_t min_wait_ms;
extern const uint32_t max_wait_ms;

//----------------------------------------------------------------------------

extern uint32_t get_tmr(uint32_t msec);
extern bool check_tmr(uint32_t msec);
extern void Report(const char *txt, bool addCRLF, bool addTime);
extern void errLedOn(const char *from);


void Error_Handler(void);


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

