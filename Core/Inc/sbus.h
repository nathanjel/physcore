/*
 * sbus.h
 *
 *  Created on: Nov 26, 2020
 *      Author: marci
 */

#ifndef INC_SBUS_H_
#define INC_SBUS_H_

#ifdef __cplusplus
extern "C" {
#endif

struct SBUS_Packed {
	uint8_t header :8;
	unsigned int ch1 :11;
	unsigned int ch2 :11;
	unsigned int ch3 :11;
	unsigned int ch4 :11;
	unsigned int ch5 :11;
	unsigned int ch6 :11;
	unsigned int ch7 :11;
	unsigned int ch8 :11;
	unsigned int ch9 :11;
	unsigned int ch10 :11;
	unsigned int ch11 :11;
	unsigned int ch12 :11;
	unsigned int ch13 :11;
	unsigned int ch14 :11;
	unsigned int ch15 :11;
	unsigned int ch16 :11;
	bool dc17 :1;
	bool dc18 :1;
	bool frame_lost :1;
	bool failsafe_a :1;
	unsigned int na :4;
	uint8_t endbyte :8;
} __attribute__((packed));

union SBUS_Data {
	uint8_t raw[25];
	struct SBUS_Packed parsed;
};

struct SBUS_Unpacked {
	uint16_t ch1;
	uint16_t ch2;
	uint16_t ch3;
	uint16_t ch4;
	uint16_t ch5;
	uint16_t ch6;
	uint16_t ch7;
	uint16_t ch8;
	uint16_t ch9;
	uint16_t ch10;
	uint16_t ch11;
	uint16_t ch12;
	uint16_t ch13;
	uint16_t ch14;
	uint16_t ch15;
	uint16_t ch16;
	bool dc17;
	bool dc18;
	bool frame_lost;
	bool failsafe_a;
#ifdef __cplusplus
	SBUS_Unpacked(const SBUS_Packed &souce) {
		*this = souce;
	}
	SBUS_Unpacked& operator=(const SBUS_Packed &souce) {
		SBUS_Unpacked &r = *this;
		r.ch1 = souce.ch1;
		r.ch2 = souce.ch2;
		r.ch3 = souce.ch3;
		r.ch4 = souce.ch4;
		r.ch5 = souce.ch5;
		r.ch6 = souce.ch6;
		r.ch7 = souce.ch7;
		r.ch8 = souce.ch8;
		r.ch9 = souce.ch9;
		r.ch10 = souce.ch10;
		r.ch11 = souce.ch11;
		r.ch12 = souce.ch12;
		r.ch13 = souce.ch13;
		r.ch14 = souce.ch14;
		r.ch15 = souce.ch15;
		r.ch16 = souce.ch16;
		r.dc17 = souce.dc17;
		r.dc18 = souce.dc18;
		r.frame_lost = souce.frame_lost;
		r.failsafe_a = souce.failsafe_a;
		return r;
	}
#endif
};

void SBUS_InitRCV();
SBUS_Unpacked& SBUS_GetLast();

#ifdef __cplusplus
}
#endif

#endif /* INC_SBUS_H_ */
