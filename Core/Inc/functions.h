/*
 * functions.h
 *
 *  Created on: Nov 15, 2020
 *      Author: nathan
 */

//Sensor								Slave Address (default)	Slave Address (SA0 driven low)
//LSM6DS33 (gyro and accelerometer)		1101011b				1101010b
//LIS3MDL (magnetometer)				0011110b				0011100b
//LPS25H (barometer)					1011101b				1011100b

#ifndef INC_FUNCTIONS_H_
#define INC_FUNCTIONS_H_

#ifdef __cplusplus
extern "C" {
#endif

#define F_MSG_SET_IRIS 0x0a

#define F_CAM1	0
#define F_CAM2	1

#define DSHOT_ALIGNED_MEM_DECL(x) uint32_t x[DSHOT_BSRR_DATA_MEMORY_LEN_WORDS] __attribute__ ((aligned (32)))
#define DSHOT_EDT_MEM_DECL(x) uint16_t x[DSHOT_FUNCTION_MAX_CHANNELS+1]
#define DSHOT_EDT_COUNT_DECL(x) uint8_t x

#define DSHOT_BSRR_DATA_MEMORY_LEN_WORDS (128)
#define DSHOT_BSRR_DATA_MEMORY_LEN_BYTES (512)
#define DSHOT_BSRR_HALFWORD_SHIFT (16)
#define DSHOT_MESSAGE_BITLEN (16)
#define DSHOT_FUNCTION_MAX_CHANNELS (16)

#define WS2812_MAX_LEDS			(64)
#define WS2812_TIMER_CHANNEL	(TIM_CHANNEL_1)
#define WS2812_STRIPES			(3)
#define WS2812_SINGLE_COLOR_BITLEN (8)
#define WS2812_SIGNAL_DATA_0 (0x30)
#define WS2812_SIGNAL_DATA_1 (0x3e)

#define WS2812_LED_MEM_DECL(x, numleds)	uint8_t x[WS2812_STRIPES*numleds]
#define __WS2812_DMA_MEM_ALIGNED_DECL(x, maxleds) \
	uint8_t x[16 + WS2812_STRIPES*maxleds*WS2812_SINGLE_COLOR_BITLEN] __attribute__ ((aligned (32)))

#define F_SERVO_1 (0)
#define F_SERVO_2 (1)
#define F_SERVO_3 (2)

extern unsigned char F_CIE_GAMMA_MAP[];

void F_SignalUSB(bool connected);
void F_SignalOperational(bool operational);

void F_InitializeServoControl();
void F_Set_Servo(uint8_t sid, uint32_t promilage);
void F_Set_ServoRefreshRate(uint16_t hz);

void F_InitializeIrisControl();
void F_SetIrisPercent(uint16_t pwmc, uint16_t pwms);

void F_DSHOT_Prepare(const uint16_t igen_flags, uint16_t *edt,
		uint8_t *edt_count);
void F_DSHOT_Generate(uint32_t *dshot_bsrr_block_addr, const uint16_t *values,
		const uint8_t *stat_req, const uint16_t *edt, const uint8_t edtptr);
void F_DSHOT_OutputStart(const uint32_t *dshot_bsrr_block_addr, GPIO_TypeDef *GPIOx);

void F_WS2812_Init();
void F_WS2812_Drive(const uint8_t *grb_data_seq, const uint8_t num_leds, const uint8_t* gamma);

HAL_StatusTypeDef __flocal_HAL_DMA_Start(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);

#ifdef __cplusplus
}
#endif

#endif /* INC_FUNCTIONS_H_ */
