/*
 * f_iris.c
 *
 *  Created on: Nov 18, 2020
 *      Author: marci
 */

#include "stm32f7xx_hal.h"

#include "main.h"
#include "functions.h"

void F_SignalOperational(bool operational) {
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin,
			operational ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void F_SignalUSB(bool connected) {
	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin,
			connected ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void F_InitializeIrisControl() {
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

void F_SetIrisPercent(uint16_t pwmc, uint16_t pwms) {
	switch (pwmc) {
	case F_CAM1:
		htim3.Instance->CCR1 = pwms;
		break;
	case F_CAM2:
		htim3.Instance->CCR2 = pwms;
		break;
	}
}
