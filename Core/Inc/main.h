/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef union																	// module send to Lop
{	uint8_t data;
	struct
	{
		uint8_t B0				:1;
		uint8_t B1				:1;
		uint8_t B2				:1;
		uint8_t B3				:1;
		uint8_t B4				:1;
		uint8_t B5				:1;
		uint8_t B6				:1;
		uint8_t B7				:1;
	}bit;

}Flag8_t;

typedef enum
{
	MenuButtonStatus_oneClick = 0,
	MenuButtonStatus_doubleClick,
	MenuButtonStatus_heldPressed,
	MenuButtonStatus_notPressed
}MenuButtonStatus_e;


typedef enum
{
	IndicatorFlasher_normal = 0,
	IndicatorFlasher_siren,

}IndicatorFlasher_e;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define Acceptance_Level	3000
#define Restart_Level		500


#define AUDIO_LEVEL_STEP	10

#define PCF8574_ADDRESS		0b01000000
typedef struct
{
	uint32_t 			buttonTimer;
	bool 				buttonTimerEnable;
	MenuButtonStatus_e	buttonStatus;
	Flag8_t 			buttonFlag;
	uint8_t 			buttonHeldPressedCounter;
}MenuButton_t;

typedef struct
{
	uint32_t 			testTimer;
	bool 				testTimerEnable;
	uint8_t				testCounter;
	bool 				testDirection;
}SoundTest_t;

typedef struct
{
	uint32_t 			indicatorTimer;
	bool 				indicatorEnable;
	bool				indicatorLedSet;
	uint8_t				indicatorBuffer;
	IndicatorFlasher_e	indicatorMode;
	uint16_t			indicatorAddress;
	int16_t				indicatorCounter;
	uint8_t 			indicatorFlasher;
}Pcf7584Control_t;

extern MenuButton_t hMenuButton;
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
uint8_t Button_DeBounce		(uint16_t* ADC_Buffer);
uint8_t	Button1_DeBounce	(uint16_t* ADC_Buffer);
uint8_t	Button2_DeBounce	(uint16_t* ADC_Buffer);
uint8_t	Button3_DeBounce	(uint16_t* ADC_Buffer);
uint8_t	Button4_DeBounce	(uint16_t* ADC_Buffer);
uint8_t	Button5_DeBounce	(uint16_t* ADC_Buffer);
uint8_t Button6_DeBounce	(uint16_t* ADC_Buffer);
uint8_t MenuButton_Debounce	(void);

uint8_t checkTimer			(uint32_t* timer, uint32_t msTime);
void 	setTimer			(uint32_t* timer);

FunctionalState soundLevelLowerBoundryCheck	(uint16_t currentSoundLevel);
FunctionalState soundLevelUpperBoundryCheck	(uint16_t currentSoundLevel);

void 			indicatorBufferLoad			(Pcf7584Control_t* hLedIndicator);
bool 			indicatorBufferUpperCheck	(uint8_t indicatorCounter);
bool 			indicatorBufferLowerCheck	(uint8_t indicatorCounter);
void 			indicatorHandler			(Pcf7584Control_t* hLedIndicator);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define test_pin_Pin GPIO_PIN_1
#define test_pin_GPIO_Port GPIOA
#define menu_button_Pin GPIO_PIN_4
#define menu_button_GPIO_Port GPIOA
#define menu_button_EXTI_IRQn EXTI4_IRQn



/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
