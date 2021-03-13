/*
 * impl.cpp
 *
 *  Created on: 28 lis 2020
 *      Author: marci
 */

#include "main.h"

#include "functions.h"
#include "sensors.h"
#include "csm.h"
#include "fast_hsv2rgb.h"
#include "Madgwick.h"
#include "usbd_cdc_if.h"
#include "sbus.h"

uint32_t ltick = 0;
Madgwick mfilter;

DSHOT_ALIGNED_MEM_DECL(dshot_bsrr_data);
DSHOT_EDT_MEM_DECL(dshot_edt);
DSHOT_EDT_COUNT_DECL(dshot_edt_count);

extern "C" void pre_while() {
	// init Servo control
	F_InitializeServoControl();
	F_Set_ServoRefreshRate(60);
	F_Set_Servo(F_SERVO_1, 500);
	F_Set_Servo(F_SERVO_2, 500);
	F_Set_Servo(F_SERVO_3, 500);

	// init WS output
	F_WS2812_Init();

	// startup camera PWM timers and init closed iris
	F_InitializeIrisControl();
	F_SetIrisPercent(F_CAM1, 0);
	F_SetIrisPercent(F_CAM2, 0);

	// signal startup LED
	F_SignalOperational(true);

	uint16_t dshot_values[8] = { 1024, 1023, 0, 0, 0, 0, 0, 0 };
	uint8_t dshot_stats[8] = { 1, 0, 0, 0, 0, 0, 0, 0 };

	uint16_t dshot_flags = OUT_DRIVE_1_Pin | OUT_DRIVE_2_Pin | OUT_DRIVE_3_Pin
			| OUT_DRIVE_4_Pin | OUT_DRIVE_5_Pin | OUT_DRIVE_6_Pin
			| OUT_DRIVE_7_Pin | OUT_DRIVE_8_Pin;

	F_DSHOT_Prepare(dshot_flags, dshot_edt, &dshot_edt_count);
	F_DSHOT_Generate(dshot_bsrr_data, dshot_values, dshot_stats, dshot_edt,
			dshot_edt_count);
	F_DSHOT_OutputStart(dshot_bsrr_data, OUT_DRIVE_1_GPIO_Port);

	bool is_ls25h = S_Presence(&S_LPS25H);
	if (is_ls25h) {
		S_Configure(&S_LPS25H);
	}
	bool is_lis3mdl = S_Presence(&S_LIS3MDL);
	if (is_lis3mdl) {
		S_Configure(&S_LIS3MDL);
	}
	bool is_lsm6ds33 = S_Presence(&S_LSM6DS33);
	if (is_lsm6ds33) {
		S_Configure(&S_LSM6DS33);
	}

	mfilter.begin(1000.0);
	ltick = HAL_GetTick();
	SBUS_InitRCV();
}

uint32_t last_tick = 0;
WS2812_LED_MEM_DECL(leds, 8);
uint16_t h = 640;
uint8_t buff_press[16];
uint8_t buff_mag[16];
uint8_t buff_ga[16];

extern "C" void in_while() {
	uint8_t r, g, b;
	uint32_t curr_tick = HAL_GetTick();
	if (ltick < curr_tick) {
		ltick += 50;
		uint16_t localh = h;
		h += 16;
		if (h > HSV_HUE_MAX) {
			h -= HSV_HUE_MAX;
		}
		for (int i = 0; i < 8; i++) {
			fast_hsv2rgb_32bit(localh, 240, 64, &r, &g, &b);
			leds[3 * i + 0] = g;
			leds[3 * i + 1] = r;
			leds[3 * i + 2] = b;
			localh += 32;
			if (localh > HSV_HUE_MAX) {
				localh -= HSV_HUE_MAX;
			}
		}
		F_WS2812_Drive(leds, 8, F_CIE_GAMMA_MAP);
		F_DSHOT_OutputStart(dshot_bsrr_data, OUT_DRIVE_1_GPIO_Port);
		F_Set_Servo(F_SERVO_1, (h * 2) / 3);
		F_Set_Servo(F_SERVO_2, (h * 2) / 3);
		F_Set_Servo(F_SERVO_3, (h * 2) / 3);
		char pbuf[128];
		sprintf(pbuf, "P%7.2f R%7.2f Y%7.2f\r\n", mfilter.getPitch(),
				mfilter.getRoll(), mfilter.getYaw());
		CDC_Transmit_FS((uint8_t*) pbuf, strlen(pbuf));
	}
	if (curr_tick != last_tick) {
		last_tick = curr_tick;
		S_GetMeasurement(&S_LPS25H, buff_press);
		S_GetMeasurement(&S_LIS3MDL, buff_mag);
		S_GetMeasurement(&S_LSM6DS33, buff_ga);
		mfilter.update(
				S_Scale_float(S_Parse_i16le(buff_ga + S_LSM6DS33_OFFSET_GYRO_X),
				S_LSM6DS33_GYRO_CONV_1000DPS_RANGE_DEG),
				S_Scale_float(S_Parse_i16le(buff_ga + S_LSM6DS33_OFFSET_GYRO_Y),
				S_LSM6DS33_GYRO_CONV_1000DPS_RANGE_DEG),
				S_Scale_float(S_Parse_i16le(buff_ga + S_LSM6DS33_OFFSET_GYRO_Z),
				S_LSM6DS33_GYRO_CONV_1000DPS_RANGE_DEG),
				S_Scale_float(S_Parse_i16le(buff_ga + S_LSM6DS33_OFFSET_ACC_X),
						1),
				S_Scale_float(S_Parse_i16le(buff_ga + S_LSM6DS33_OFFSET_ACC_Y),
						1),
				S_Scale_float(S_Parse_i16le(buff_ga + S_LSM6DS33_OFFSET_ACC_Z),
						1),
				S_Scale_float(S_Parse_i16le(buff_mag + S_LIS3MDL_OFFSET_X), 1),
				S_Scale_float(S_Parse_i16le(buff_mag + S_LIS3MDL_OFFSET_Y), 1),
				S_Scale_float(S_Parse_i16le(buff_mag + S_LIS3MDL_OFFSET_Z), 1));
	}

	csm_message_list_t *msg = CSM_GetMessage(&csmc);
	if (msg) {
		switch (msg->data[0]) {
		case F_MSG_SET_IRIS:
			if (msg->data[1] <= 100)
				F_SetIrisPercent(F_CAM1, msg->data[1]);
			if (msg->data[2] <= 100)
				F_SetIrisPercent(F_CAM2, msg->data[2]);
		}
		CSM_CleanupMessage(msg);
	}
}
