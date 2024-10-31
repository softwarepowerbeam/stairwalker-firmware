/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define VNH7XXXXAS_PWM_BIS_Pin GPIO_PIN_9
#define VNH7XXXXAS_PWM_BIS_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define VNH7XXXXAS_CURRENT_S_Pin GPIO_PIN_0
#define VNH7XXXXAS_CURRENT_S_GPIO_Port GPIOC
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define mcs1802_vout_Pin GPIO_PIN_3
#define mcs1802_vout_GPIO_Port GPIOC
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define VNH7XXXXAS_PWM_Pin GPIO_PIN_3
#define VNH7XXXXAS_PWM_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define LED_GREEN_Pin GPIO_PIN_0
#define LED_GREEN_GPIO_Port GPIOB
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_14
#define LED_RED_GPIO_Port GPIOB
#define STLK_VCP_RX_Pin GPIO_PIN_8
#define STLK_VCP_RX_GPIO_Port GPIOD
#define STLK_VCP_TX_Pin GPIO_PIN_9
#define STLK_VCP_TX_GPIO_Port GPIOD
#define USB_FS_PWR_EN_Pin GPIO_PIN_10
#define USB_FS_PWR_EN_GPIO_Port GPIOD
#define BNO085_PS0_Pin GPIO_PIN_2
#define BNO085_PS0_GPIO_Port GPIOG
#define BNO085_PS1_Pin GPIO_PIN_3
#define BNO085_PS1_GPIO_Port GPIOG
#define USB_FS_OVCR_Pin GPIO_PIN_7
#define USB_FS_OVCR_GPIO_Port GPIOG
#define BNO085_CS_Pin GPIO_PIN_8
#define BNO085_CS_GPIO_Port GPIOC
#define BNO085_INTN_Pin GPIO_PIN_9
#define BNO085_INTN_GPIO_Port GPIOC
#define USB_FS_VBUS_Pin GPIO_PIN_9
#define USB_FS_VBUS_GPIO_Port GPIOA
#define USB_FS_ID_Pin GPIO_PIN_10
#define USB_FS_ID_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define BNO085_TX_SCK_Pin GPIO_PIN_10
#define BNO085_TX_SCK_GPIO_Port GPIOC
#define BNO085_RX_MISO_Pin GPIO_PIN_11
#define BNO085_RX_MISO_GPIO_Port GPIOC
#define BNO085_NRST_Pin GPIO_PIN_2
#define BNO085_NRST_GPIO_Port GPIOD
#define VNH7XXXXAS_SEL0_Pin GPIO_PIN_3
#define VNH7XXXXAS_SEL0_GPIO_Port GPIOD
#define VNH7XXXXAS_INB_Pin GPIO_PIN_4
#define VNH7XXXXAS_INB_GPIO_Port GPIOD
#define VNH7XXXXAS_INA_Pin GPIO_PIN_5
#define VNH7XXXXAS_INA_GPIO_Port GPIOD
#define LIMIT_SW1_Pin GPIO_PIN_6
#define LIMIT_SW1_GPIO_Port GPIOD
#define LIMIT_SW2_Pin GPIO_PIN_7
#define LIMIT_SW2_GPIO_Port GPIOD
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LED_YELLOW_Pin GPIO_PIN_1
#define LED_YELLOW_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
#define ALIGN_BASE2_CEIL(nSize, nAlign)  ( ((nSize) + ((nAlign) - 1)) & ~((nAlign) - 1) )

typedef struct control_vars
{
	float state;
	float state_raw;
	float error;
	float output;
	float raw_output;

}control_vars_t;


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
