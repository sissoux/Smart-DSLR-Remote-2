/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include "stm32l4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VUSB_INTERNAL_EN_Pin GPIO_PIN_13
#define VUSB_INTERNAL_EN_GPIO_Port GPIOC
#define Touch_YP_Pin GPIO_PIN_0
#define Touch_YP_GPIO_Port GPIOC
#define Touch_XM_Pin GPIO_PIN_1
#define Touch_XM_GPIO_Port GPIOC
#define Touch_XP_Pin GPIO_PIN_2
#define Touch_XP_GPIO_Port GPIOC
#define Touch_YM_Pin GPIO_PIN_3
#define Touch_YM_GPIO_Port GPIOC
#define Charger_Current_Pin GPIO_PIN_0
#define Charger_Current_GPIO_Port GPIOA
#define Backlight_PWM_Pin GPIO_PIN_1
#define Backlight_PWM_GPIO_Port GPIOA
#define LCD_RST_Pin GPIO_PIN_2
#define LCD_RST_GPIO_Port GPIOA
#define LCD_DC_Pin GPIO_PIN_3
#define LCD_DC_GPIO_Port GPIOA
#define LCD_CS_Pin GPIO_PIN_4
#define LCD_CS_GPIO_Port GPIOA
#define LCD_CLK_Pin GPIO_PIN_5
#define LCD_CLK_GPIO_Port GPIOA
#define LCD_MISO_Pin GPIO_PIN_6
#define LCD_MISO_GPIO_Port GPIOA
#define LCD_MOSI_Pin GPIO_PIN_7
#define LCD_MOSI_GPIO_Port GPIOA
#define Aux_5V_EN_Pin GPIO_PIN_4
#define Aux_5V_EN_GPIO_Port GPIOC
#define Trigger_Analog_Comp_Pin GPIO_PIN_5
#define Trigger_Analog_Comp_GPIO_Port GPIOC
#define Trigger_Analog_Pin GPIO_PIN_0
#define Trigger_Analog_GPIO_Port GPIOB
#define Trigger_source_sel1_Pin GPIO_PIN_1
#define Trigger_source_sel1_GPIO_Port GPIOB
#define Trigger_source_sel2_Pin GPIO_PIN_2
#define Trigger_source_sel2_GPIO_Port GPIOB
#define Ext_I2C_SCL_Pin GPIO_PIN_10
#define Ext_I2C_SCL_GPIO_Port GPIOB
#define Ext_I2C_SDA_Pin GPIO_PIN_11
#define Ext_I2C_SDA_GPIO_Port GPIOB
#define DSLR_Shutter_EN_Pin GPIO_PIN_12
#define DSLR_Shutter_EN_GPIO_Port GPIOB
#define DSLR_Focus_EN_Pin GPIO_PIN_13
#define DSLR_Focus_EN_GPIO_Port GPIOB
#define Flash1_EN_Pin GPIO_PIN_14
#define Flash1_EN_GPIO_Port GPIOB
#define Flash2_EN_Pin GPIO_PIN_15
#define Flash2_EN_GPIO_Port GPIOB
#define Charger_status_Pin GPIO_PIN_6
#define Charger_status_GPIO_Port GPIOC
#define PwrMux_Stat_Pin GPIO_PIN_7
#define PwrMux_Stat_GPIO_Port GPIOC
#define Reg_5V_EN_Pin GPIO_PIN_8
#define Reg_5V_EN_GPIO_Port GPIOC
#define Reg_33_EN_Pin GPIO_PIN_9
#define Reg_33_EN_GPIO_Port GPIOC
#define USB_OTG_EN_Pin GPIO_PIN_8
#define USB_OTG_EN_GPIO_Port GPIOA
#define USB_VBUS_LV_Pin GPIO_PIN_9
#define USB_VBUS_LV_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define Pwr_Mux_EN_Pin GPIO_PIN_10
#define Pwr_Mux_EN_GPIO_Port GPIOC
#define BLE_CMD_Pin GPIO_PIN_12
#define BLE_CMD_GPIO_Port GPIOC
#define BLE_HW_Wake_Pin GPIO_PIN_2
#define BLE_HW_Wake_GPIO_Port GPIOD
#define BLE_CTS_Pin GPIO_PIN_3
#define BLE_CTS_GPIO_Port GPIOB
#define BLE_RTS_Pin GPIO_PIN_4
#define BLE_RTS_GPIO_Port GPIOB
#define BLE_SW_Wake_Pin GPIO_PIN_5
#define BLE_SW_Wake_GPIO_Port GPIOB
#define BLE_TX_Pin GPIO_PIN_6
#define BLE_TX_GPIO_Port GPIOB
#define BLE_RX_Pin GPIO_PIN_7
#define BLE_RX_GPIO_Port GPIOB
#define Status_LED_Pin GPIO_PIN_8
#define Status_LED_GPIO_Port GPIOB
#define IR_LED_Pin GPIO_PIN_9
#define IR_LED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
