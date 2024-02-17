/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f4xx_hal.h"

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
#define RN_B_Pin GPIO_PIN_2
#define RN_B_GPIO_Port GPIOE
#define RN_A_Pin GPIO_PIN_3
#define RN_A_GPIO_Port GPIOE
#define SW_F2_Pin GPIO_PIN_4
#define SW_F2_GPIO_Port GPIOE
#define SW_Down_Pin GPIO_PIN_5
#define SW_Down_GPIO_Port GPIOE
#define LCD_D7_Pin GPIO_PIN_6
#define LCD_D7_GPIO_Port GPIOE
#define LCD_D6_Pin GPIO_PIN_13
#define LCD_D6_GPIO_Port GPIOC
#define LCD_D5_Pin GPIO_PIN_14
#define LCD_D5_GPIO_Port GPIOC
#define LCD_D4_Pin GPIO_PIN_15
#define LCD_D4_GPIO_Port GPIOC
#define LCD_EN_Pin GPIO_PIN_0
#define LCD_EN_GPIO_Port GPIOC
#define LCD_RW_Pin GPIO_PIN_1
#define LCD_RW_GPIO_Port GPIOC
#define LCD_RS_Pin GPIO_PIN_2
#define LCD_RS_GPIO_Port GPIOC
#define Sensor2_Pin GPIO_PIN_3
#define Sensor2_GPIO_Port GPIOC
#define Sensor2_EXTI_IRQn EXTI3_IRQn
#define Start_Pin GPIO_PIN_0
#define Start_GPIO_Port GPIOA
#define Sensor1_Pin GPIO_PIN_1
#define Sensor1_GPIO_Port GPIOA
#define Sensor1_EXTI_IRQn EXTI1_IRQn
#define SupplyFB_Pin GPIO_PIN_2
#define SupplyFB_GPIO_Port GPIOA
#define Watchdog_Pin GPIO_PIN_3
#define Watchdog_GPIO_Port GPIOA
#define CMND28_Pin GPIO_PIN_4
#define CMND28_GPIO_Port GPIOA
#define SPI_CS_Pin GPIO_PIN_4
#define SPI_CS_GPIO_Port GPIOC
#define SPI_WP_Pin GPIO_PIN_5
#define SPI_WP_GPIO_Port GPIOC
#define CMND27_Pin GPIO_PIN_0
#define CMND27_GPIO_Port GPIOB
#define CMND26_Pin GPIO_PIN_1
#define CMND26_GPIO_Port GPIOB
#define CMND25_Pin GPIO_PIN_2
#define CMND25_GPIO_Port GPIOB
#define CMND24_Pin GPIO_PIN_7
#define CMND24_GPIO_Port GPIOE
#define CMND23_Pin GPIO_PIN_8
#define CMND23_GPIO_Port GPIOE
#define CMND22_Pin GPIO_PIN_9
#define CMND22_GPIO_Port GPIOE
#define CMND21_Pin GPIO_PIN_10
#define CMND21_GPIO_Port GPIOE
#define CMND20_Pin GPIO_PIN_11
#define CMND20_GPIO_Port GPIOE
#define CMND19_Pin GPIO_PIN_12
#define CMND19_GPIO_Port GPIOE
#define CMND18_Pin GPIO_PIN_13
#define CMND18_GPIO_Port GPIOE
#define CMND17_Pin GPIO_PIN_14
#define CMND17_GPIO_Port GPIOE
#define ENSF4_Pin GPIO_PIN_15
#define ENSF4_GPIO_Port GPIOE
#define ENSF3_Pin GPIO_PIN_10
#define ENSF3_GPIO_Port GPIOB
#define ENSF3_EXTI_IRQn EXTI15_10_IRQn
#define ENSF2_Pin GPIO_PIN_11
#define ENSF2_GPIO_Port GPIOB
#define ENSF2_EXTI_IRQn EXTI15_10_IRQn
#define ENSF1_Pin GPIO_PIN_12
#define ENSF1_GPIO_Port GPIOB
#define ENSF1_EXTI_IRQn EXTI15_10_IRQn
#define ENSF8_Pin GPIO_PIN_13
#define ENSF8_GPIO_Port GPIOB
#define ENSF7_Pin GPIO_PIN_14
#define ENSF7_GPIO_Port GPIOB
#define ENSF6_Pin GPIO_PIN_15
#define ENSF6_GPIO_Port GPIOB
#define ENSF5_Pin GPIO_PIN_8
#define ENSF5_GPIO_Port GPIOD
#define S1CMND_Pin GPIO_PIN_9
#define S1CMND_GPIO_Port GPIOD
#define S2CMND_Pin GPIO_PIN_10
#define S2CMND_GPIO_Port GPIOD
#define S5T6CMND_Pin GPIO_PIN_11
#define S5T6CMND_GPIO_Port GPIOD
#define S3CMND_Pin GPIO_PIN_12
#define S3CMND_GPIO_Port GPIOD
#define S4T5CMND_Pin GPIO_PIN_13
#define S4T5CMND_GPIO_Port GPIOD
#define S7T8CMND_Pin GPIO_PIN_14
#define S7T8CMND_GPIO_Port GPIOD
#define T2CMND_Pin GPIO_PIN_15
#define T2CMND_GPIO_Port GPIOD
#define S6T7CMND_Pin GPIO_PIN_6
#define S6T7CMND_GPIO_Port GPIOC
#define T1CMND_Pin GPIO_PIN_7
#define T1CMND_GPIO_Port GPIOC
#define T3CMND_Pin GPIO_PIN_8
#define T3CMND_GPIO_Port GPIOC
#define EnableRS485_Pin GPIO_PIN_9
#define EnableRS485_GPIO_Port GPIOC
#define T4CMND_Pin GPIO_PIN_8
#define T4CMND_GPIO_Port GPIOA
#define Reverse_Pin GPIO_PIN_9
#define Reverse_GPIO_Port GPIOA
#define UART1RX_Pin GPIO_PIN_10
#define UART1RX_GPIO_Port GPIOA
#define CMND16_Pin GPIO_PIN_15
#define CMND16_GPIO_Port GPIOA
#define CMND15_Pin GPIO_PIN_10
#define CMND15_GPIO_Port GPIOC
#define CMND14_Pin GPIO_PIN_11
#define CMND14_GPIO_Port GPIOC
#define CMND13_Pin GPIO_PIN_12
#define CMND13_GPIO_Port GPIOC
#define CMND12_Pin GPIO_PIN_0
#define CMND12_GPIO_Port GPIOD
#define CMND11_Pin GPIO_PIN_1
#define CMND11_GPIO_Port GPIOD
#define CMND10_Pin GPIO_PIN_2
#define CMND10_GPIO_Port GPIOD
#define CMND9_Pin GPIO_PIN_3
#define CMND9_GPIO_Port GPIOD
#define CMND8_Pin GPIO_PIN_4
#define CMND8_GPIO_Port GPIOD
#define CMND7_Pin GPIO_PIN_5
#define CMND7_GPIO_Port GPIOD
#define CMND6_Pin GPIO_PIN_6
#define CMND6_GPIO_Port GPIOD
#define CMND5_Pin GPIO_PIN_7
#define CMND5_GPIO_Port GPIOD
#define CMND4_Pin GPIO_PIN_3
#define CMND4_GPIO_Port GPIOB
#define CMND3_Pin GPIO_PIN_4
#define CMND3_GPIO_Port GPIOB
#define CMND2_Pin GPIO_PIN_5
#define CMND2_GPIO_Port GPIOB
#define CMND1_Pin GPIO_PIN_6
#define CMND1_GPIO_Port GPIOB
#define SW_F1_Pin GPIO_PIN_7
#define SW_F1_GPIO_Port GPIOB
#define SW_Left_Pin GPIO_PIN_8
#define SW_Left_GPIO_Port GPIOB
#define SW_Right_Pin GPIO_PIN_9
#define SW_Right_GPIO_Port GPIOB
#define SW_Up_Pin GPIO_PIN_0
#define SW_Up_GPIO_Port GPIOE
#define RN_SW_Pin GPIO_PIN_1
#define RN_SW_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
