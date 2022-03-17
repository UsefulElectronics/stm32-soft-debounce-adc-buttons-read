/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * Author: Ward Almasarani
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
MenuButton_t hMenuButton;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	hMenuButton.buttonFlag.data = RESET;			//Flag reset

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  uint16_t ADC_Buffer[1] = {0};										//Initialize ADC DMA buffer

  HAL_ADC_Start_DMA(&hadc1, ADC_Buffer, 1);							//Start ADC sampling over DMA

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(ADC_Buffer[0] < 3900)										//it is possible that one of the buttons has been pressed
	  {																//Every led will have a debounce function

		  if((ADC_Buffer[0] <= 500) && (ADC_Buffer[0] > 1)) //First region
		  {
			  if(Button_DeBounce(ADC_Buffer))
			  {
				  htim2.Instance->PSC 	= 5000;
				  htim2.Instance->CCR1  = 1;
			  }
		  }
		  else if((ADC_Buffer[0] <= 1250) && (ADC_Buffer[0] > 750)) //Second region
		  {
			  if(Button1_DeBounce(ADC_Buffer))
			  {
				  htim2.Instance->PSC 	= 10000;
				  htim2.Instance->CCR1  = 1;
			  }
		  }
		  else if((ADC_Buffer[0] <= 1800) && (ADC_Buffer[0] > 1300)) //third region
		  {
			  if(Button2_DeBounce(ADC_Buffer))
			  {
				  htim2.Instance->PSC 	= 15000;
				  htim2.Instance->CCR1  = 1;
			  }
		  }
		  else if((ADC_Buffer[0] <= 2300) && (ADC_Buffer[0] > 1800)) //Forth region
		  {
			  if(Button3_DeBounce(ADC_Buffer))
			  {
				  htim2.Instance->PSC 	= 20000;
				  htim2.Instance->CCR1  = 1;
			  }
		  }
		  else if((ADC_Buffer[0] <= 2900) && (ADC_Buffer[0] > 2400)) //Fifth region
		  {
			  if(Button4_DeBounce(ADC_Buffer))
			  {
				  htim2.Instance->PSC 	= 22500;
				  htim2.Instance->CCR1  = 1;
			  }
		  }
		  else if((ADC_Buffer[0] <= 3400) && (ADC_Buffer[0] > 3000)) //Sixth region
		  {
			  if(Button5_DeBounce(ADC_Buffer))
			  {
				  htim2.Instance->PSC 	= 25000;
				  htim2.Instance->CCR1  = 1;
			  }
		  }
		  else if((ADC_Buffer[0] <= 3900) && (ADC_Buffer[0] > 3500)) //Seventh region
		  {
			  if(Button6_DeBounce(ADC_Buffer))
			  {
				  htim2.Instance->PSC 	= 27500;
				  htim2.Instance->CCR1  = 1;
			  }
		  }
	  }
	  else
	  {
		  htim2.Instance->CCR1  = 0;									//Set duty cycle to be zero
		  Button_DeBounce(ADC_Buffer);									//Discharge software capacitors
		  Button1_DeBounce(ADC_Buffer);
		  Button2_DeBounce(ADC_Buffer);
		  Button3_DeBounce(ADC_Buffer);
		  Button4_DeBounce(ADC_Buffer);
		  Button5_DeBounce(ADC_Buffer);
		  Button6_DeBounce(ADC_Buffer);
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(hMenuButton.buttonFlag.bit.B0)
	  {
		  static uint8_t Toggle = RESET;
		  if(MenuButton_Debounce())
		  {
			  hMenuButton.buttonFlag.bit.B0 = RESET;
			  HAL_GPIO_WritePin(test_pin_GPIO_Port, test_pin_Pin, Toggle);
			  Toggle ^= 1;

			  if(hMenuButton.buttonTimerEnable != SET)
			  {
				  hMenuButton.buttonTimerEnable = SET;
				  hMenuButton.buttonStatus = MenuButtonStatus_oneClick;
				  setTimer(&hMenuButton.buttonTimer);
			  }
			  else
			  {
				  hMenuButton.buttonStatus = MenuButtonStatus_doubleClick;
			  }


		  }
	  }
	  if((checkTimer(&hMenuButton.buttonTimer, 10 * hMenuButton.buttonHeldPressedCounter)) && hMenuButton.buttonTimerEnable)
	  {
		  if(!HAL_GPIO_ReadPin(menu_button_GPIO_Port, menu_button_Pin))
		  {
			  ++hMenuButton.buttonHeldPressedCounter;
		  }
		  hMenuButton.buttonStatus = (hMenuButton.buttonHeldPressedCounter >= 30)  ? MenuButtonStatus_heldPressed : hMenuButton.buttonStatus;
	  }
	  if((checkTimer(&hMenuButton.buttonTimer, 300)) && hMenuButton.buttonTimerEnable)
	  {
		  switch(hMenuButton.buttonStatus)
		  {
		  	  case	MenuButtonStatus_oneClick:
		  		  break;
		  	  case 	MenuButtonStatus_doubleClick:
		  		  break;
		  	  case MenuButtonStatus_heldPressed:
		  		  break;

		  }
		  hMenuButton.buttonTimerEnable 		= RESET;
		  hMenuButton.buttonHeldPressedCounter 	= RESET;
	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 10000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(test_pin_GPIO_Port, test_pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : test_pin_Pin */
  GPIO_InitStruct.Pin = test_pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(test_pin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : menu_button_Pin */
  GPIO_InitStruct.Pin = menu_button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(menu_button_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

uint8_t Button_DeBounce(uint16_t* ADC_Buffer)
{
	uint8_t 		ret = DISABLE;
	static uint16_t	Level = Restart_Level;			//Software capacitor charged level
	if(ADC_Buffer[0] < 3900)						//The level is set to be static so the last level value is not gone after executing this function
	{
		++Level;									//Increment Software capacitor charged level
	}
	else
	{												//entered when no button is pressed
		--Level;									//Decrement Software capacitor charged level
		if(Level <= Restart_Level)					//keep the lowest level at Restart_Level. this will also prevent overflow
		{
			Level = Restart_Level;
		}
	}
	if(Level >= Acceptance_Level)					//limit the Software capacitor charged level at Acceptance_Level when a button is held pressed
	{
		Level = Acceptance_Level;
		ret = ENABLE;								//Enable taking action wherever this function is called
	}
	return ret;
}

uint8_t Button1_DeBounce(uint16_t* ADC_Buffer)
{
	uint8_t 		ret = DISABLE;
	static uint16_t	Level = Restart_Level;
	if(ADC_Buffer[0] < 3900)
	{
		++Level;
	}
	else
	{
		--Level;
		if(Level <= Restart_Level)
		{
			Level = Restart_Level;
		}
	}
	if(Level >= Acceptance_Level)
	{
		Level = Acceptance_Level;
		ret = ENABLE;
	}
	return ret;
}

uint8_t Button2_DeBounce(uint16_t* ADC_Buffer)
{
	uint8_t 		ret = DISABLE;
	static uint16_t	Level = Restart_Level;
	if(ADC_Buffer[0] < 3900)
	{
		++Level;
	}
	else
	{
		--Level;
		if(Level <= Restart_Level)
		{
			Level = Restart_Level;
		}
	}
	if(Level >= Acceptance_Level)
	{
		Level = Acceptance_Level;
		ret = ENABLE;
	}
	return ret;
}
uint8_t Button3_DeBounce(uint16_t* ADC_Buffer)
{
	uint8_t 		ret = DISABLE;
	static uint16_t	Level = Restart_Level;
	if(ADC_Buffer[0] < 3900)
	{
		++Level;
	}
	else
	{
		--Level;
		if(Level <= Restart_Level)
		{
			Level = Restart_Level;
		}
	}
	if(Level >= Acceptance_Level)
	{
		Level = Acceptance_Level;
		ret = ENABLE;
	}
	return ret;
}
uint8_t Button4_DeBounce(uint16_t* ADC_Buffer)
{
	uint8_t 		ret = DISABLE;
	static uint16_t	Level = Restart_Level;
	if(ADC_Buffer[0] < 3900)
	{
		++Level;
	}
	else
	{
		--Level;
		if(Level <= Restart_Level)
		{
			Level = Restart_Level;
		}
	}
	if(Level >= Acceptance_Level)
	{
		Level = Acceptance_Level;
		ret = ENABLE;
	}
	return ret;
}
uint8_t Button5_DeBounce(uint16_t* ADC_Buffer)
{
	uint8_t 		ret = DISABLE;
	static uint16_t	Level = Restart_Level;
	if(ADC_Buffer[0] < 3900)
	{
		++Level;
	}
	else
	{
		--Level;
		if(Level <= Restart_Level)
		{
			Level = Restart_Level;
		}
	}
	if(Level >= Acceptance_Level)
	{
		Level = Acceptance_Level;
		ret = ENABLE;
	}
	return ret;
}
uint8_t Button6_DeBounce(uint16_t* ADC_Buffer)
{
	uint8_t 		ret = DISABLE;
	static uint16_t	Level = Restart_Level;
	if(ADC_Buffer[0] < 3900)
	{
		++Level;
	}
	else
	{
		--Level;
		if(Level <= Restart_Level)
		{
			Level = Restart_Level;
		}
	}
	if(Level >= Acceptance_Level)
	{
		Level = Acceptance_Level;
		ret = ENABLE;
	}
	return ret;
}
uint8_t MenuButton_Debounce(void)									//Menu Button debounce function
{
	uint8_t 		ret = DISABLE;
	static uint16_t Level = Restart_Level;
	if(!HAL_GPIO_ReadPin(menu_button_GPIO_Port, menu_button_Pin))
	{
		++Level;
	}
	else
	{
		--Level;
		if(Level <= Restart_Level)
		{
			Level = Restart_Level;
		}
	}
	if(Level >= 4000)
	{
		Level = Acceptance_Level;
		ret = ENABLE;
	}
	return ret;
}

void setTimer(uint32_t* timer)
{
	*timer = HAL_GetTick();
}
uint8_t checkTimer(uint32_t* timer, uint32_t msTime)
{
	uint8_t ret = RESET;
	ret = ((HAL_GetTick() - *timer) > msTime)  ? ENABLE : DISABLE;
	return ret;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
