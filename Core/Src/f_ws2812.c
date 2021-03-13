/*
 * functions.c
 *
 *  Created on: Nov 15, 2020
 *      Author: nathan
 */

#include "stm32f7xx_hal.h"

#include "main.h"
#include "functions.h"

static __WS2812_DMA_MEM_ALIGNED_DECL(__ws2812_signal, WS2812_MAX_LEDS);
void F_WS2812_Init() {
	// HAL_TIM_OC_Start(&htim1, WS2812_TIMER_CHANNEL);
}

void F_WS2812_Drive(const uint8_t *grb_data_seq, const uint8_t num_leds, const uint8_t* gamma) {
	// prepare
	uint8_t *sa = __ws2812_signal;
	// zero line at start
	*sa++ = 0;
	*sa++ = 0;
	*sa++ = 0;
	*sa++ = 0;
	*sa++ = 0;
	*sa++ = 0;
	*sa++ = 0;
	*sa++ = 0;
	int pnum = num_leds * WS2812_STRIPES;
	if (gamma) {
		while (pnum--) {
			uint8_t byte = gamma[*grb_data_seq++];
			*sa++ = (byte & 0x80) ? WS2812_SIGNAL_DATA_1 : WS2812_SIGNAL_DATA_0;
			*sa++ = (byte & 0x40) ? WS2812_SIGNAL_DATA_1 : WS2812_SIGNAL_DATA_0;
			*sa++ = (byte & 0x20) ? WS2812_SIGNAL_DATA_1 : WS2812_SIGNAL_DATA_0;
			*sa++ = (byte & 0x10) ? WS2812_SIGNAL_DATA_1 : WS2812_SIGNAL_DATA_0;
			*sa++ = (byte & 0x08) ? WS2812_SIGNAL_DATA_1 : WS2812_SIGNAL_DATA_0;
			*sa++ = (byte & 0x04) ? WS2812_SIGNAL_DATA_1 : WS2812_SIGNAL_DATA_0;
			*sa++ = (byte & 0x02) ? WS2812_SIGNAL_DATA_1 : WS2812_SIGNAL_DATA_0;
			*sa++ = (byte & 0x01) ? WS2812_SIGNAL_DATA_1 : WS2812_SIGNAL_DATA_0;
		}
	} else {
		while (pnum--) {
			uint8_t byte = *grb_data_seq++;
			*sa++ = (byte & 0x80) ? WS2812_SIGNAL_DATA_1 : WS2812_SIGNAL_DATA_0;
			*sa++ = (byte & 0x40) ? WS2812_SIGNAL_DATA_1 : WS2812_SIGNAL_DATA_0;
			*sa++ = (byte & 0x20) ? WS2812_SIGNAL_DATA_1 : WS2812_SIGNAL_DATA_0;
			*sa++ = (byte & 0x10) ? WS2812_SIGNAL_DATA_1 : WS2812_SIGNAL_DATA_0;
			*sa++ = (byte & 0x08) ? WS2812_SIGNAL_DATA_1 : WS2812_SIGNAL_DATA_0;
			*sa++ = (byte & 0x04) ? WS2812_SIGNAL_DATA_1 : WS2812_SIGNAL_DATA_0;
			*sa++ = (byte & 0x02) ? WS2812_SIGNAL_DATA_1 : WS2812_SIGNAL_DATA_0;
			*sa++ = (byte & 0x01) ? WS2812_SIGNAL_DATA_1 : WS2812_SIGNAL_DATA_0;
		}
	}
	// zero line at end
	*sa++ = 0;
	*sa++ = 0;
	*sa++ = 0;
	*sa++ = 0;
	*sa++ = 0;
	*sa++ = 0;
	*sa++ = 0;
	*sa++ = 0;
	// clean cache
	SCB_CleanDCache_by_Addr((uint32_t*) __ws2812_signal,
			16 + num_leds * WS2812_STRIPES * WS2812_SINGLE_COLOR_BITLEN);
	// output
	HAL_SPI_Transmit_DMA(&hspi1, __ws2812_signal,
			16 + num_leds * WS2812_STRIPES * WS2812_SINGLE_COLOR_BITLEN);
}
