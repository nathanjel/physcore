/*
 * sensors.h
 *
 *  Created on: 23 lis 2020
 *      Author: marci
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#ifdef __cplusplus
extern "C" {
#endif

struct i2c_st_sensor_def {
	uint8_t bus_address;
	uint8_t who_am_i_reg_addr;
	uint8_t who_am_i_reg_val;
	uint8_t status_reg_addr;
	uint8_t measurement_address;
	uint8_t measurement_len;
	uint8_t private_params;
	uint8_t config_tuples[];
};

typedef struct i2c_st_sensor_def i2c_st_sensor_def_t;

#define S_STANDARD_GRAVITY (9.80665)
#define S_LPS25H_OFFSET_PRESSURE (0x0)

#define S_LIS3MDL_OFFSET_X (0x0)
#define S_LIS3MDL_OFFSET_Y (0x2)
#define S_LIS3MDL_OFFSET_Z (0x4)

#define S_LSM6DS33_OFFSET_GYRO_X (0x0)
#define S_LSM6DS33_OFFSET_GYRO_Y (0x2)
#define S_LSM6DS33_OFFSET_GYRO_Z (0x4)
#define S_LSM6DS33_OFFSET_ACC_X (0x6)
#define S_LSM6DS33_OFFSET_ACC_Y (0x8)
#define S_LSM6DS33_OFFSET_ACC_Z (0xa)

#define S_LPS25H_CONV_HPA (1.0 / 4096.0)
#define S_LIS3MDL_CONV_4_GAUSS_RANGE (4.0 / 32768.0)
#define S_LSM6DS33_GYRO_CONV_1000DPS_RANGE_DEG (1000.0 / 32768.0)
#define S_LSM6DS33_ACC_CONV_4G_RANGE (S_STANDARD_GRAVITY * 4.0 / 32768.0)

extern i2c_st_sensor_def_t S_LPS25H;
extern i2c_st_sensor_def_t S_LIS3MDL;
extern i2c_st_sensor_def_t S_LSM6DS33;

bool S_Presence(i2c_st_sensor_def_t * sdef);
bool S_Configure(i2c_st_sensor_def_t * sdef);
uint8_t S_GetStatusReg(i2c_st_sensor_def_t * sdef);
bool S_GetMeasurement(i2c_st_sensor_def_t * sdef, uint8_t * databuff);
bool S_GetMeasurement_DMA(i2c_st_sensor_def_t * sdef, uint8_t * databuff);
bool S_Arbitraty_Read(i2c_st_sensor_def_t * sdef, uint8_t addr, uint8_t * databuff, uint8_t datalen);
bool S_Arbitraty_Write(i2c_st_sensor_def_t * sdef, uint8_t addr, uint8_t * databuff, uint8_t datalen);

int32_t S_Parse_u24le(uint8_t * databuff);
int16_t S_Parse_i16le(uint8_t * databuff);
double S_Scale_double(int32_t raw, double scale);
float S_Scale_float(int32_t raw, float scale);

#ifdef __cplusplus
}
#endif

#endif /* INC_SENSORS_H_ */
