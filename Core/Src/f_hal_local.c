/*
 * f_hal_local.c
 *
 *  Created on: Nov 18, 2020
 *      Author: marci
 */

#include "stm32f7xx_hal.h"

static void __flocal_DMA_SetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress,
		uint32_t DstAddress, uint32_t DataLength) {
	/* Clear DBM bit */
	hdma->Instance->CR &= (uint32_t) (~DMA_SxCR_DBM);

	/* Configure DMA Stream data length */
	hdma->Instance->NDTR = DataLength;

	/* Memory to Peripheral */
	if ((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH) {
		/* Configure DMA Stream destination address */
		hdma->Instance->PAR = DstAddress;

		/* Configure DMA Stream source address */
		hdma->Instance->M0AR = SrcAddress;
	}
	/* Peripheral to Memory */
	else {
		/* Configure DMA Stream source address */
		hdma->Instance->PAR = SrcAddress;

		/* Configure DMA Stream destination address */
		hdma->Instance->M0AR = DstAddress;
	}
}

HAL_StatusTypeDef __flocal_HAL_DMA_Start(DMA_HandleTypeDef *hdma,
		uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength) {
	HAL_StatusTypeDef status = HAL_OK;

	/* Check the parameters */
	assert_param(IS_DMA_BUFFER_SIZE(DataLength));

//  /* Process locked */
//  __HAL_LOCK(hdma);
//
//  if(HAL_DMA_STATE_READY == hdma->State)
//  {
//    /* Change DMA peripheral state */
//    hdma->State = HAL_DMA_STATE_BUSY;
//
//    /* Initialize the error code */
//    hdma->ErrorCode = HAL_DMA_ERROR_NONE;

	// reset DMA interrupts
	__HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));
	__HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_HT_FLAG_INDEX(hdma));
	__HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TE_FLAG_INDEX(hdma));
	__HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_DME_FLAG_INDEX(hdma));
	__HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_FE_FLAG_INDEX(hdma));

	/* Configure the source, destination address and the data length */
	__flocal_DMA_SetConfig(hdma, SrcAddress, DstAddress, DataLength);

	/* Enable the Peripheral */
	__HAL_DMA_ENABLE(hdma);
//  }
//  else
//  {
//    /* Process unlocked */
//    __HAL_UNLOCK(hdma);
//
//    /* Return error status */
//    status = HAL_BUSY;
//  }
	return status;
}

