/*
 * csm.c
 *
 *  Created on: Nov 15, 2020
 *      Author: nathan
 */

#include "main.h"
#include "csm.h"
#include "cs.h"
#include <string.h>

static uint8_t csm_tmp_datablock[CSM_MAX_MSG_SIZE];
static uint8_t csm_mempool_message_payload[CSM_MAX_MSG_SIZE
		* CSM_MAX_MSGS_IN_QUEUE];
static csm_message_list_t csm_mempool_message_list[CSM_MAX_MSGS_IN_QUEUE];

volatile static uint8_t csm_mempool_allocation_map[CSM_MAX_MSGS_IN_QUEUE];
volatile static uint8_t csm_mempool_next_allocation_index;

static csm_message_list_t* CSM_Memory_Manager_Alloc_Message() {
	csm_mempool_allocation_map[csm_mempool_next_allocation_index] = 1;
	csm_message_list_t *rv = csm_mempool_message_list
			+ csm_mempool_next_allocation_index;
	rv->data = csm_mempool_message_payload
			+ (csm_mempool_next_allocation_index * CSM_MAX_MSG_SIZE);
	while (csm_mempool_allocation_map[csm_mempool_next_allocation_index]) {
		csm_mempool_next_allocation_index++;
		csm_mempool_next_allocation_index %= CSM_MAX_MSGS_IN_QUEUE;
	}
	return rv;
}

void CSM_Memory_Manager_Init() {
	int i = CSM_MAX_MSGS_IN_QUEUE;
	while (i--) {
		csm_mempool_allocation_map[i] = 0;
	}
	csm_mempool_next_allocation_index = 0;
}

void CSM_Init(csm_core_t *cc) {
	cc->data = csm_tmp_datablock;
	cc->state = PendingSync;
	cc->messages = NULL;
}

void CSM_Process(csm_core_t *cc, uint8_t const *bytes, const uint32_t len) {
	for (int i = 0; i < len; i++) {
		uint8_t rbyte = bytes[i];
		switch (cc->state) {
		case PendingSync:
			if (rbyte == CSM_SYNC_BYTE) {
				cc->state = PendingLen;
			}
			break;
		case PendingLen:
			if (rbyte <= CSM_MAX_MSG_SIZE) {
				cc->t0 = rbyte;
				cc->t1 = 0;
				if (rbyte) {
					cc->state = ReceivingData;
				} else {
					cc->state = ReceivingCRChi;
				}
			} else {
				cc->state = PendingSync;
			}
			break;
		case ReceivingData:
			cc->data[cc->t1++] = rbyte;
			if (cc->t0 == cc->t1) {
				cc->state = ReceivingCRChi;
				cc->crc = 0;
			}
			break;
		case ReceivingCRChi:
			cc->datalen = cc->t1;
			cc->crc |= rbyte;
			cc->crc <<= 8;
			cc->state = ReceivingCRClo;
			break;
		case ReceivingCRClo:
			cc->crc |= rbyte;
			uint32_t mcrc = HAL_CRC_Calculate(&hcrc, (uint32_t*) cc->data,
					cc->datalen);
			if (cc->crc == mcrc) {
				csm_message_list_t *nmessage =
						CSM_Memory_Manager_Alloc_Message();
				nmessage->next = NULL;
				nmessage->datalen = cc->datalen;
				memcpy(nmessage->data, cc->data, cc->datalen);
				volatile csm_message_list_t **msgadr = (volatile csm_message_list_t **)&(cc->messages);
				while (*msgadr != NULL) {
					msgadr = (volatile csm_message_list_t **)&((*msgadr)->next);
				}
				*msgadr = nmessage;
			}
			cc->state = PendingSync;
			break;
		}
	}
}

csm_message_list_t* CSM_GetMessage(csm_core_t *cc) {
	if (!cc->messages)
		return NULL;
	__HAL_ENTER_CRITICAL_SECTION();
	csm_message_list_t *rv = cc->messages;
	if (cc->messages) {
		cc->messages = rv->next;
	}
	__HAL_EXIT_CRITICAL_SECTION();
	rv->next = NULL;
	return rv;
}

void CSM_CleanupMessage(csm_message_list_t *msg) {
	int midx = msg - csm_mempool_message_list;
	csm_mempool_allocation_map[midx] = 0;
}
