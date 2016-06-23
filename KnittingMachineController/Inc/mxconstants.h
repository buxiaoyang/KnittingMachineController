/**
  ******************************************************************************
  * File Name          : mxconstants.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MXCONSTANT_H
#define __MXCONSTANT_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define Motor2_Limt_Start_Pin GPIO_PIN_2
#define Motor2_Limt_Start_GPIO_Port GPIOE
#define Motor2_Limt_End_Pin GPIO_PIN_3
#define Motor2_Limt_End_GPIO_Port GPIOE
#define Motor3_Limt_Start_Pin GPIO_PIN_4
#define Motor3_Limt_Start_GPIO_Port GPIOE
#define Motor3_Limt_End_Pin GPIO_PIN_5
#define Motor3_Limt_End_GPIO_Port GPIOE
#define Motor4_Limt_Start_Pin GPIO_PIN_6
#define Motor4_Limt_Start_GPIO_Port GPIOE
#define Motor4_PWM_Pin GPIO_PIN_9
#define Motor4_PWM_GPIO_Port GPIOI
#define Motor5_PWM_Pin GPIO_PIN_10
#define Motor5_PWM_GPIO_Port GPIOI
#define Motor6_PWM_Pin GPIO_PIN_11
#define Motor6_PWM_GPIO_Port GPIOI
#define Motor7_PWM_Pin GPIO_PIN_6
#define Motor7_PWM_GPIO_Port GPIOF
#define Motor8_PWM_Pin GPIO_PIN_7
#define Motor8_PWM_GPIO_Port GPIOF
#define Motor1_Dir_Pin GPIO_PIN_8
#define Motor1_Dir_GPIO_Port GPIOF
#define Motor2_Dir_Pin GPIO_PIN_9
#define Motor2_Dir_GPIO_Port GPIOF
#define Motor3_Dir_Pin GPIO_PIN_10
#define Motor3_Dir_GPIO_Port GPIOF
#define USART2_DEBUG_TX_Pin GPIO_PIN_2
#define USART2_DEBUG_TX_GPIO_Port GPIOA
#define USART2_DEBUG_RX_Pin GPIO_PIN_3
#define USART2_DEBUG_RX_GPIO_Port GPIOA
#define SPI1_SD2_SCK_Pin GPIO_PIN_5
#define SPI1_SD2_SCK_GPIO_Port GPIOA
#define SPI1_SD2_MISO_Pin GPIO_PIN_6
#define SPI1_SD2_MISO_GPIO_Port GPIOA
#define SPI1_SD2_MOSI_Pin GPIO_PIN_7
#define SPI1_SD2_MOSI_GPIO_Port GPIOA
#define Motor4_Dir_Pin GPIO_PIN_4
#define Motor4_Dir_GPIO_Port GPIOC
#define Motor5_Dir_Pin GPIO_PIN_5
#define Motor5_Dir_GPIO_Port GPIOC
#define Motor6_Dir_Pin GPIO_PIN_0
#define Motor6_Dir_GPIO_Port GPIOB
#define Motor7_Dir_Pin GPIO_PIN_1
#define Motor7_Dir_GPIO_Port GPIOB
#define Motor8_Dir_Pin GPIO_PIN_8
#define Motor8_Dir_GPIO_Port GPIOH
#define SDIO_SD1_D0_Pin GPIO_PIN_8
#define SDIO_SD1_D0_GPIO_Port GPIOC
#define SDIO_SD1_D1_Pin GPIO_PIN_9
#define SDIO_SD1_D1_GPIO_Port GPIOC
#define USART1_LCD_TX_Pin GPIO_PIN_9
#define USART1_LCD_TX_GPIO_Port GPIOA
#define USART1_LCD_RX_Pin GPIO_PIN_10
#define USART1_LCD_RX_GPIO_Port GPIOA
#define Motor1_PWM_Pin GPIO_PIN_0
#define Motor1_PWM_GPIO_Port GPIOI
#define Motor2_PWM_Pin GPIO_PIN_1
#define Motor2_PWM_GPIO_Port GPIOI
#define Motor3_PWM_Pin GPIO_PIN_2
#define Motor3_PWM_GPIO_Port GPIOI
#define SDIO_SD1_D2_Pin GPIO_PIN_10
#define SDIO_SD1_D2_GPIO_Port GPIOC
#define SDIO_SD1_D3_Pin GPIO_PIN_11
#define SDIO_SD1_D3_GPIO_Port GPIOC
#define SDIO_SD1_CK_Pin GPIO_PIN_12
#define SDIO_SD1_CK_GPIO_Port GPIOC
#define SDIO_SD1_CMD_Pin GPIO_PIN_2
#define SDIO_SD1_CMD_GPIO_Port GPIOD
#define Motor1_Limt_Start_Pin GPIO_PIN_3
#define Motor1_Limt_Start_GPIO_Port GPIOD
#define Motor1_Limt_End_Pin GPIO_PIN_4
#define Motor1_Limt_End_GPIO_Port GPIOD
#define Motor4_Limt_End_Pin GPIO_PIN_5
#define Motor4_Limt_End_GPIO_Port GPIOD
#define Motor5_Limt_Start_Pin GPIO_PIN_6
#define Motor5_Limt_Start_GPIO_Port GPIOD
#define Motor5_Limt_End_Pin GPIO_PIN_7
#define Motor5_Limt_End_GPIO_Port GPIOD
#define Motor6_Limt_Start_Pin GPIO_PIN_9
#define Motor6_Limt_Start_GPIO_Port GPIOG
#define Motor6_Limt_End_Pin GPIO_PIN_10
#define Motor6_Limt_End_GPIO_Port GPIOG
#define Motor7_Limt_Start_Pin GPIO_PIN_11
#define Motor7_Limt_Start_GPIO_Port GPIOG
#define Motor7_Limt_End_Pin GPIO_PIN_12
#define Motor7_Limt_End_GPIO_Port GPIOG
#define Motor8_Limt_Start_Pin GPIO_PIN_13
#define Motor8_Limt_Start_GPIO_Port GPIOG
#define Motor8_Limt_End_Pin GPIO_PIN_14
#define Motor8_Limt_End_GPIO_Port GPIOG
#define Relay_Data_Pin GPIO_PIN_3
#define Relay_Data_GPIO_Port GPIOB
#define Relay_CLK_Pin GPIO_PIN_4
#define Relay_CLK_GPIO_Port GPIOB
#define Relay_LAT_Pin GPIO_PIN_5
#define Relay_LAT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MXCONSTANT_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
