/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "csm.h"
#include "functions.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_tim8_up;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;
extern CRC_HandleTypeDef hcrc;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart7;
extern csm_core_t csmc;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void SystemClock_Config();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OUT_DRIVE_2_Pin GPIO_PIN_2
#define OUT_DRIVE_2_GPIO_Port GPIOE
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define SERVO_DRIVER___Pin GPIO_PIN_0
#define SERVO_DRIVER___GPIO_Port GPIOA
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define SERVO_DRIVER_2_Pin GPIO_PIN_3
#define SERVO_DRIVER_2_GPIO_Port GPIOA
#define WS2812_SPI_CLK_IGNORE_Pin GPIO_PIN_5
#define WS2812_SPI_CLK_IGNORE_GPIO_Port GPIOA
#define APERTURE_CAM___Pin GPIO_PIN_6
#define APERTURE_CAM___GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define OUT_DRIVE_3_Pin GPIO_PIN_7
#define OUT_DRIVE_3_GPIO_Port GPIOE
#define SBUS_RX_Pin GPIO_PIN_8
#define SBUS_RX_GPIO_Port GPIOE
#define OUT_DRIVE_4_Pin GPIO_PIN_10
#define OUT_DRIVE_4_GPIO_Port GPIOE
#define OUT_DRIVE_5_Pin GPIO_PIN_11
#define OUT_DRIVE_5_GPIO_Port GPIOE
#define OUT_DRIVE_6_Pin GPIO_PIN_12
#define OUT_DRIVE_6_GPIO_Port GPIOE
#define OUT_DRIVE_7_Pin GPIO_PIN_14
#define OUT_DRIVE_7_GPIO_Port GPIOE
#define OUT_DRIVE_8_Pin GPIO_PIN_15
#define OUT_DRIVE_8_GPIO_Port GPIOE
#define SERVO_DRIVER_3_Pin GPIO_PIN_10
#define SERVO_DRIVER_3_GPIO_Port GPIOB
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define APERTURE_CAM_2_Pin GPIO_PIN_7
#define APERTURE_CAM_2_GPIO_Port GPIOC
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define WS2812_SPI_DATA_Pin GPIO_PIN_7
#define WS2812_SPI_DATA_GPIO_Port GPIOD
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
#define MEMS_I2C_SCL_Pin GPIO_PIN_8
#define MEMS_I2C_SCL_GPIO_Port GPIOB
#define MEMS_I2C_SDA_Pin GPIO_PIN_9
#define MEMS_I2C_SDA_GPIO_Port GPIOB
#define OUT_DRIVE_1_Pin GPIO_PIN_0
#define OUT_DRIVE_1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
