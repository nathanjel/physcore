/*
 * f_dshot.c
 *
 *  Created on: Nov 18, 2020
 *      Author: marci
 */

#include "stm32f7xx_hal.h"

#include "main.h"
#include "functions.h"

/**
 * @brief  Starts the TIM Base generation in DMA mode.
 * @param  htim TIM Base handle
 * @param  pData The source Buffer address.
 * @param  Length The length of data to be transferred from memory to peripheral.
 * @retval HAL status
 */
static HAL_StatusTypeDef __flocal_HAL_TIM_Base_Start_DMA(
		TIM_HandleTypeDef *htim, uint32_t *pData, uint32_t *targetRegisterAddr,
		uint16_t Length) {
	uint32_t tmpsmcr;

	/* Check the parameters */
	assert_param(IS_TIM_DMA_INSTANCE(htim->Instance));

//	if (htim->State == HAL_TIM_STATE_BUSY) {
//		return HAL_BUSY;
//	} else if (htim->State == HAL_TIM_STATE_READY) {
//		if ((pData == NULL) && (Length > 0U)) {
//			return HAL_ERROR;
//		} else {
//			htim->State = HAL_TIM_STATE_BUSY;
//		}
//	} else {
//		/* nothing to do */
//	}

//	/* Set the DMA Period elapsed callbacks */
//	htim->hdma[TIM_DMA_ID_UPDATE]->XferCpltCallback =
//			__flocal_TIM_DMAPeriodElapsedCplt;
//	htim->hdma[TIM_DMA_ID_UPDATE]->XferHalfCpltCallback =
//			__flocal_TIM_DMAPeriodElapsedHalfCplt;

	/* Set the DMA error callback */
	htim->hdma[TIM_DMA_ID_UPDATE]->XferErrorCallback = TIM_DMAError;

	/* Enable the DMA stream */
	// was ***_IT
	if (__flocal_HAL_DMA_Start(htim->hdma[TIM_DMA_ID_UPDATE], (uint32_t) pData,
			(uint32_t) targetRegisterAddr, Length) != HAL_OK) {
		return HAL_ERROR;
	}

	/* Enable the TIM Update DMA request */
	__HAL_TIM_ENABLE_DMA(htim, TIM_DMA_UPDATE);

	/* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
	tmpsmcr = htim->Instance->SMCR & TIM_SMCR_SMS;
	if (!IS_TIM_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr)) {
		__HAL_TIM_ENABLE(htim);
	}

	/* Return function status */
	return HAL_OK;
}

void F_DSHOT_Prepare(const uint16_t igen_flags, uint16_t *edt,
		uint8_t *edt_count) {
	int edtptr = 0;
	int shiftptr = 0;
	int gen_flags = igen_flags;
	while (gen_flags != 0 && edtptr < DSHOT_FUNCTION_MAX_CHANNELS) {
		// LSB first
		if (gen_flags & 0x1) {
			edt[edtptr++] = 1 << shiftptr;
		}
		gen_flags >>= 1;
		shiftptr++;
	}
	*edt_count = edtptr;
	edt[edtptr] = igen_flags;			// all ON
}

void F_DSHOT_Generate(uint32_t *dshot_bsrr_block_addr, const uint16_t *values,
		const uint8_t *stat_req, const uint16_t *edt, const uint8_t edtptr) {
	uint16_t pvalues[DSHOT_FUNCTION_MAX_CHANNELS];
	uint32_t set_bits = edt[edtptr];
	uint32_t reset_bits = set_bits << DSHOT_BSRR_HALFWORD_SHIFT;
	// add status request and calculate parity
	for (int i = 0; i < edtptr; i++) {
		pvalues[i] = (values[i] << 5) | (stat_req[i] ? 0x10 : 0x0);
		pvalues[i] |= __builtin_parity(pvalues[i] & 0x8880) << 3;
		pvalues[i] |= __builtin_parity(pvalues[i] & 0x4440) << 2;
		pvalues[i] |= __builtin_parity(pvalues[i] & 0x2220) << 1;
		pvalues[i] |= __builtin_parity(pvalues[i] & 0x1110);
	}
	// generate BSRR data for GPIO control
	for (int i = 0; i < DSHOT_MESSAGE_BITLEN; i++) {
		// current bits
		uint32_t nval = 0;
		for (int k = 0; k < edtptr; k++) {
			// MSB first
			nval |= edt[k] << ((pvalues[k] & 0x8000) ? 0 : DSHOT_BSRR_HALFWORD_SHIFT);
			pvalues[k] <<= 1;
		}
		// all high
		*dshot_bsrr_block_addr++ = set_bits;
		*dshot_bsrr_block_addr++ = set_bits;
		*dshot_bsrr_block_addr++ = set_bits;
		// data driven
		*dshot_bsrr_block_addr++ = nval;
		*dshot_bsrr_block_addr++ = nval;
		*dshot_bsrr_block_addr++ = nval;
		// all low
		*dshot_bsrr_block_addr++ = reset_bits;
		*dshot_bsrr_block_addr++ = reset_bits;
	}
}

void F_DSHOT_OutputStart(const uint32_t *dshot_bsrr_block_addr, GPIO_TypeDef *GPIOx) {
	SCB_CleanDCache_by_Addr((uint32_t *)dshot_bsrr_block_addr, DSHOT_BSRR_DATA_MEMORY_LEN_BYTES);
	__flocal_HAL_TIM_Base_Start_DMA(&htim8, (uint32_t *)dshot_bsrr_block_addr,
			(uint32_t*) &GPIOx->BSRR, DSHOT_BSRR_DATA_MEMORY_LEN_WORDS);
}
