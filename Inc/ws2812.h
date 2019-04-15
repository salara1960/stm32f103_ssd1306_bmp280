/*
 * ws2812.h
 *
 *  Created on: Apr 11, 2019
 *      Author: alarm
 */
#ifndef WS2812_H_
#define WS2812_H_

//--------------------------------------------------

#include "stm32f1xx_hal.h"

//--------------------------------------------------
#pragma pack(push,1)
typedef struct {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
} rgb_t;
#pragma pack(pop)
//--------------------------------------------------

#define DELAY_LEN 0//48
#define LED_COUNT 8//144
#define ARRAY_LEN DELAY_LEN + (LED_COUNT * 24)// 0 + 8 * 24 = 0 + 192 = 192
#define HIGH 65
#define LOW 26

#define L32 32
#define L64 L32 << 1
#define L128 L32 << 2

#define RGB_SET(r,g,b) ((rgb_t){.red=r, .green=g, .blue=b})

#define BitIsSet(reg, bit) ((reg & (1 << bit)) != 0)

//--------------------------------------------------

//--------------------------------------------------

void ws2812_init(void);
void ws2812_pixel_to_buf(rgb_t colors, uint16_t posX);
void ws2812_setData(void *adr);
void ws2812_start(void);


#endif /* WS2812_H_ */
