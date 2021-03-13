/*
 * sb_generic.cpp
 *
 *  Created on: Nov 29, 2020
 *      Author: marci
 */


#include <stdbool.h>
#include <string.h>
#include "stm32f7xx_hal.h"

#include "main.h"
#include "sbus.h"

char * rt0 = "C";
char * rt1 = "H";
char * rt2 = "E";

uint16_t sbus_uart_circ_buff[256];

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart7) {
		HAL_UART_Transmit(&huart3, (uint8_t*)rt0, 1, 1);
	}
}

extern "C" void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart7) {
		HAL_UART_Transmit(&huart3, (uint8_t*)rt1, 1, 1);
	}
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart7) {
		HAL_UART_Transmit(&huart3, (uint8_t*)rt2, 1, 1);
	}
}

void SBUS_InitRCV() {
	// SET_BIT(huart7.Instance->CR1, USART_CR1_IDLEIE);
	HAL_UART_Receive_DMA(&huart7, (uint8_t*) sbus_uart_circ_buff, 256);
}

SBUS_Unpacked& SBUS_GetLast() {
	SBUS_Data k;
	memcpy(&k.raw, sbus_uart_circ_buff, 25);
	SBUS_Unpacked w(k.parsed);
	return w;
}
