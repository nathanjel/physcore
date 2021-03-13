/*
 * csm.h
 *
 *  Created on: Nov 15, 2020
 *      Author: nathan
 */

#ifndef INC_CSM_H_
#define INC_CSM_H_

#ifdef __cplusplus
extern "C" {
#endif

// Python code for message generation:
//
//__crc16f = crcmod.mkCrcFun(0x11021, initCrc=0xffff, xorOut=0x0, rev=False)
//
//def create_message(dbytes):
//    msg = [0xa5]
//    msg.append(len(dbytes))
//    msg.extend(dbytes)
//    crc = __crc16f(dbytes)
//    msg.append((crc >> 8) & 0xff)
//    msg.append(crc & 0xff)
//    return bytearray(msg)
//

#include <stdint.h>

#define CSM_SYNC_BYTE (0xA5)
#define CSM_REC_TIMEOUT (0xfff)

#define CSM_MAX_MSG_SIZE (0x7F)
#define CSM_MAX_MSGS_IN_QUEUE (0x40)

enum csm_state {
	PendingSync,
	PendingLen,
	ReceivingData,
	ReceivingCRChi,
	ReceivingCRClo
};

typedef enum csm_state csm_state_t;

struct csm_message_list;
typedef struct csm_message_list csm_message_list_t;

struct csm_message_list {
	uint8_t * data;
	uint8_t datalen;
	csm_message_list_t * next;
};

struct csm_core {
	csm_message_list_t * messages;
	csm_state_t state;
	uint32_t t0, t1;
	uint16_t crc;
	uint8_t datalen;
	uint8_t * data;
};

typedef volatile struct csm_core csm_core_t;

void CSM_Memory_Manager_Init();
void CSM_Init(csm_core_t * cc);
void CSM_Process(csm_core_t * cc, uint8_t const * bytes, const uint32_t len);
csm_message_list_t * CSM_GetMessage(csm_core_t * cc);
void CSM_CleanupMessage(csm_message_list_t * msg);

#ifdef __cplusplus
}
#endif

#endif /* INC_CSM_H_ */
