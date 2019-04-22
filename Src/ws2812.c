/*
 * ws2812.c
 *
 *  Created on: Apr 11, 2019
 *      Author: alarm
 */
#include "ws2812.h"

//----------------------------------------------------------------------------

extern TIM_HandleTypeDef htim2;

extern uint8_t GoTxDMA;

static uint16_t rgb_BUF[ARRAY_LEN] = {0};

//------------------------------------------------------------------
void ws2812_init(void)
{
	for (uint16_t i = DELAY_LEN; i < ARRAY_LEN; i++) rgb_BUF[i] = LOW;
}
//------------------------------------------------------------------
void ws2812_pixel_to_buf(rgb_t colors, uint16_t posX)
{
uint16_t word;

	for (uint16_t i = 0; i < LED_COUNT; i++) {
		if (BitIsSet(colors.red, (7 - i)) == 1) word = HIGH; else word = LOW;
		rgb_BUF[DELAY_LEN + posX * 24 + i + 8] = word;

		if (BitIsSet(colors.green, (7 - i)) == 1) word = HIGH; else word = LOW;
		rgb_BUF[DELAY_LEN + posX * 24 + i + 0] = word;

		if (BitIsSet(colors.blue, (7 - i)) == 1) word = HIGH; else word = LOW;
		rgb_BUF[DELAY_LEN + posX * 24 + i + 16] = word;
	}
}
//------------------------------------------------------------------
void ws2812_setData(void *adr)
{
	for (int i = 0; i < LED_COUNT; i++) ws2812_pixel_to_buf(*(rgb_t *)(adr + i * sizeof(rgb_t)), i);
}
//------------------------------------------------------------------
void ws2812_start(void)
{
	GoTxDMA = 1;
	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, (uint32_t *)&rgb_BUF, ARRAY_LEN);
}
//------------------------------------------------------------------





