/*
 * s_generic.c
 *
 *  Created on: 23 lis 2020
 *      Author: marci
 */

#include <stdbool.h>
#include "stm32f7xx_hal.h"

#include "main.h"
#include "sensors.h"

//struct i2c_st_sensor_def {
//	uint8_t bus_address;
//	uint8_t who_am_i_reg_addr;
//	uint8_t who_am_i_reg_val;
//	uint8_t status_reg_addr;
//	uint8_t measurement_address;
//	uint8_t measurement_len;
//	uint8_t * config_tuples;
//	uint8_t private_params;
//};

i2c_st_sensor_def_t S_LPS25H = {
	.bus_address = 0x5d,
	.who_am_i_reg_addr = 0x0f,
	.who_am_i_reg_val = 0xbd,
	.status_reg_addr = 0x27,
	.measurement_address = 0x28 | 0x80,	// assert most significant register bit to enable auto increment
	.measurement_len = 0x03,
	.config_tuples = {
			0x20, 0xc4,		// PD, ODR = 25Hz, BDU
			0x21, 0x00,
			0x22, 0x00,
			0x23, 0x00,
			0x10, 0x0f,		// Pressure N. internal average 512, Temp N. internal average 64
			0
		}
};

i2c_st_sensor_def_t S_LIS3MDL = {
	.bus_address = 0x1e,
	.who_am_i_reg_addr = 0x0f,
	.who_am_i_reg_val = 0x3d,
	.status_reg_addr = 0x27,
	.measurement_address = 0x28 | 0x80,	// assert most significant register bit to enable auto increment
	.measurement_len = 0x06,
	.config_tuples = {
			0x21, 0x0c,		// reboot
			0x22, 0x03,		// MD = 11 (power down)
			0x20, 0x62,		// OM = 11 (UHP, 155Hz), FAST_ODR
			0x21, 0x00,		// FS = 00 (+/-4gauss)
			0x23, 0x0c,		// OMZ = 11 (UHP), BLE = 0 (little endian)
			0x24, 0x40,		// BDU
			0x30, 0x08,		// Disable all interrupts
			0x22, 0x00,		// MD = 00 (continous)
			0
		}
};

i2c_st_sensor_def_t S_LSM6DS33 = {
	.bus_address = 0x6b,
	.who_am_i_reg_addr = 0x0f,
	.who_am_i_reg_val = 0x69,
	.status_reg_addr = 0x1e,
	.measurement_address = 0x22,	//
	.measurement_len = 0x0c,
	.config_tuples = {
			0x10, 0x88,		// ACCEL ODR = 1000 (1,6kHz), FS_XL = 10 (4g), BW_XL = 00, 400Hz
			0x11, 0x88,		// GYRO ODR = 1000 (1,6kHz), FS_G = 10 (1000 dps), FS_125 = 0
			0x12, 0x44,		// BDU, IF_INC, BLE = 0 (little endian)
			0x13, 0x80,		// ACCEL BANDWIDTH DIRECT, MASKS DISABLED
			0x14, 0x00,		// No rounding
			0x16, 0x40,		// Gyro High Performance, HPF enabled, 0.0081Hz cutoff
			0x17, 0x00,		// Accel filters off
			0x18, 0x38,		// Accel output enable all axis
			0x19, 0x38,		// Gyro output enable all axis
			0
		}
};

bool S_Presence(i2c_st_sensor_def_t * sdef) {
	uint8_t wmi = ~sdef->who_am_i_reg_val;
	HAL_I2C_Mem_Read(&hi2c1, sdef->bus_address << 1, sdef->who_am_i_reg_addr, 1, &wmi, 1, HAL_MAX_DELAY);
	return sdef->who_am_i_reg_val == wmi;
}

bool S_Configure(i2c_st_sensor_def_t * sdef) {
	HAL_StatusTypeDef vret = HAL_OK;
	uint8_t * reg = sdef->config_tuples;
	while(vret == HAL_OK && *reg != 0) {
		vret = HAL_I2C_Mem_Write(&hi2c1, sdef->bus_address << 1, *reg, 1, reg+1, 1, HAL_MAX_DELAY);
		reg += 2;
	}
	return vret;
}

uint8_t S_GetStatusReg(i2c_st_sensor_def_t * sdef) {
	uint8_t reg = 0x00;
	HAL_I2C_Mem_Read(&hi2c1, sdef->bus_address << 1, sdef->status_reg_addr, 1, &reg, 1, HAL_MAX_DELAY);
	return reg;
}

bool S_GetMeasurement(i2c_st_sensor_def_t * sdef, uint8_t * databuff) {
	return HAL_OK == HAL_I2C_Mem_Read(&hi2c1, sdef->bus_address << 1, sdef->measurement_address, 1, databuff, sdef->measurement_len, HAL_MAX_DELAY);
}

bool S_GetMeasurement_DMA(i2c_st_sensor_def_t * sdef, uint8_t * databuff) {
	return HAL_OK == HAL_I2C_Mem_Read_DMA(&hi2c1, sdef->bus_address << 1, sdef->measurement_address, 1, databuff, sdef->measurement_len);
}

bool S_Arbitraty_Read(i2c_st_sensor_def_t * sdef, uint8_t addr, uint8_t * databuff, uint8_t datalen) {
	return HAL_OK == HAL_I2C_Mem_Read(&hi2c1, sdef->bus_address << 1, addr, 1, databuff, datalen, HAL_MAX_DELAY);
}

bool S_Arbitraty_Write(i2c_st_sensor_def_t * sdef, uint8_t addr, uint8_t * databuff, uint8_t datalen) {
	return HAL_OK == HAL_I2C_Mem_Write(&hi2c1, sdef->bus_address << 1, addr, 1, databuff, datalen, HAL_MAX_DELAY);
}

int32_t S_Parse_u24le(uint8_t * databuff) {
	// below never gets negative, as we're parsing an usigned
	// and don't touch oldest bits of int
	return databuff[0] | (databuff[1] << 8) | (databuff[2] << 16);
}

int16_t S_Parse_i16le(uint8_t * databuff) {
	// luckily the processor endiannes matches
	return *((int16_t*)(databuff));
}

double S_Scale_double(int32_t raw, double scale) {
	double out = raw;
	out *= scale;
	return out;
}

float S_Scale_float(int32_t raw, float scale) {
	float out = raw;
	out *= scale;
	return out;
}
