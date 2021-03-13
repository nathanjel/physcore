/*
 * f_servo.c
 *
 *  Created on: Nov 22, 2020
 *      Author: marci
 */

#include <main.h>
#include <functions.h>

void F_InitializeServoControl() {
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

void F_Set_ServoRefreshRate(uint16_t hz) {
	uint32_t period = 1000000 / hz;
	htim2.Instance->ARR = period;
}

void F_Set_Servo(uint8_t sid, uint32_t promilage) {
	if (promilage > 1000) {
		promilage = 1000;
	}
	promilage += 1000;
	switch (sid) {
	case F_SERVO_1:
		htim2.Instance->CCR1 = promilage;
		break;
	case F_SERVO_2:
		htim2.Instance->CCR3 = promilage;
		break;
	case F_SERVO_3:
		htim2.Instance->CCR4 = promilage;
		break;
	}
}
