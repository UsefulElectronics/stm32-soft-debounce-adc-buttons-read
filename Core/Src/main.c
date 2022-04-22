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

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
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
	SoundTest_t			hSoundLevelTest;

	Pcf7584Control_t	hIndicator;

	hMenuButton.buttonFlag.data 		= RESET;			//Flag reset

	hSoundLevelTest.testSoundLevel 		= 10;






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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  uint16_t 	ADC_Buffer[1] 		= {0};										//Initialize ADC DMA buffer

  hIndicator.indicatorAddress 	= PCF8574_ADDRESS;
  hIndicator.indicatorEnable	= ENABLE;
  hIndicator.indicatorCounter	= ENABLE;
  hIndicator.indicatorLedSet 	= SET;

  hMenuButton.buttonStatus 		= MenuButtonStatus_notPressed;


  HAL_ADC_Start_DMA(&hadc1, ADC_Buffer, 1);							//Start ADC sampling over DMA

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  HAL_I2C_Init(&hi2c1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(adcKeyboardHandler	(ADC_Buffer, hSoundLevelTest.testSoundLevel) != ENABLE)
	  {
		  sirenHandler			(&hSoundLevelTest);
	  }

	  indicatorHandler		(&hIndicator);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(hMenuButton.buttonFlag.bit.B0)
	  {
		  if(MenuButton_Debounce())											//Apply de-bounce algorithm before taking any action
		  {
			  hMenuButton.buttonFlag.bit.B0 = RESET;						//Reset interrupt flag

			  if(hMenuButton.buttonTimerEnable != SET)						//Start timer when the menu button gets pressed for the first time
			  {
				  hMenuButton.buttonTimerEnable = SET;						//Enable software timer
				  hMenuButton.buttonStatus = MenuButtonStatus_oneClick;		//The menu button has been pressed for one time
				  setTimer(&hMenuButton.buttonTimer);						//Start software timer
			  }
			  else
			  {
				  hMenuButton.buttonStatus = MenuButtonStatus_doubleClick;	//If the timer is already started this means that the button is pressed for the second time
			  }
		  }
	  }
	  if((checkTimer(&hMenuButton.buttonTimer, 5 * hMenuButton.buttonHeldPressedCounter)) && ( hMenuButton.buttonStatus == MenuButtonStatus_oneClick))
	  {																		//If the menu button has been pressed, take logic sample every 10 ms from the button read pin
		  if(!HAL_GPIO_ReadPin(menu_button_GPIO_Port, menu_button_Pin))		//To check if the button is still pressed
		  {
			  ++hMenuButton.buttonHeldPressedCounter;						//Increment samples number
		  }
		  hMenuButton.buttonStatus = (hMenuButton.buttonHeldPressedCounter >= 65)  ? MenuButtonStatus_heldPressed : hMenuButton.buttonStatus;
	  }																		//If the button is held pressed change button status
	  if((checkTimer(&hMenuButton.buttonTimer, 350)) && hMenuButton.buttonTimerEnable)
	  {																		//Take decision after 300 ms whether one click, double click or held pressed event has occured
		  switch(hMenuButton.buttonStatus)
		  {
		  	  case	MenuButtonStatus_oneClick:
		  		  if(soundLevelUpperBoundryCheck(hSoundLevelTest.testSoundLevel))
		  		  {
		  			hSoundLevelTest.testSoundLevel += AUDIO_LEVEL_STEP; 						//Increment sound level on step
		  		  }
		  		  if(indicatorBufferUpperCheck(hIndicator.indicatorCounter))
		  		  {
		  			  hIndicator.indicatorEnable = ENABLE;					//Enable LED indicator handler
		  			++hIndicator.indicatorCounter;							//Increment number of LEDs to be turned on
		  			  hIndicator.indicatorLedSet = SET;						//Turn on one more LED
		  		  }
		  		  break;
		  	  case 	MenuButtonStatus_doubleClick:
		  		  if(soundLevelLowerBoundryCheck(hSoundLevelTest.testSoundLevel))
		  		  {
		  			  hSoundLevelTest.testSoundLevel -= AUDIO_LEVEL_STEP;						//Detriment sound level
		  		  }
		  		  if(indicatorBufferLowerCheck(hIndicator.indicatorCounter))
		  		  {
		  			  hIndicator.indicatorEnable = ENABLE;
		  			--hIndicator.indicatorCounter;							//Increment number of LEDs to be turned on
		  			  hIndicator.indicatorLedSet = RESET;					//Turn off one more LED
		  		  }
		  		 break;
		  	  case MenuButtonStatus_heldPressed:
		  		  hSoundLevelTest.testTimerEnable ^= 1;
		  		  setTimer(&hSoundLevelTest.testTimer);
		  		  hSoundLevelTest.testDirection = ENABLE;
		  		 break;
		  	case MenuButtonStatus_notPressed:
		  		 break;

		  }
		  hMenuButton.buttonTimerEnable 		= RESET;					//Disable software timer
		  hMenuButton.buttonHeldPressedCounter 	= RESET;					//Reset counter
		  hMenuButton.buttonStatus 				= MenuButtonStatus_notPressed;
		  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	//button status is set to no press to avoid misbehavior of the button functionality

	  }
//	  indicatorHandler(&hIndicator);
//	  indicatorBuffer = 0x55;
//
//	  HAL_I2C_Master_Transmit(&hi2c1, indicatorAddress, &indicatorBuffer, 1, 100);

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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim2.Init.Prescaler = 200;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
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
	return ret;										//Button_DeBounce functions are copies of this function for every single button added to the keyboard bus
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
FunctionalState soundLevelLowerBoundryCheck(uint16_t currentSoundLevel)
{
	uint8_t ret = ENABLE;
	if( (currentSoundLevel == 0))
	{
		ret = DISABLE;
	}
	return ret;
}
FunctionalState soundLevelUpperBoundryCheck(uint16_t currentSoundLevel)
{
	uint8_t ret = ENABLE;
	if(currentSoundLevel >= 90)
	{
		ret = DISABLE;
	}
	return ret;
}
void indicatorHandler(Pcf7584Control_t* hLedIndicator)
{
	uint8_t bit = 0;
	uint8_t  IndicatorDisplay = hLedIndicator->indicatorBuffer;
	if(hLedIndicator->indicatorEnable)
	{
		hLedIndicator->indicatorEnable = DISABLE;				//Disable the activation flag to prevent entering this function
																//when no change is happening.
		indicatorBufferLoad(hLedIndicator);

		IndicatorDisplay = hLedIndicator->indicatorBuffer;		//Copy to a temporary buffer

		IndicatorDisplay ^= 0xFF;								//Invert bits before transmission

		HAL_I2C_Master_Transmit(&hi2c1,							//Start transmission
				hLedIndicator->indicatorAddress,
				&IndicatorDisplay,
				ENABLE,
				100);

	}
}

bool indicatorBufferUpperCheck(uint8_t indicatorCounter)
{
	bool ret 	= DISABLE;
	ret 		= (indicatorCounter == 8) ? DISABLE : ENABLE;	//Check if the upper indicator boundary has been reached
	return 	ret;
}

bool indicatorBufferLowerCheck(uint8_t indicatorCounter)
{
	bool ret 	= DISABLE;
	ret 		= (indicatorCounter == 0) ? DISABLE : ENABLE;	//Check if the lower indicator boundary has been reached
	return 	ret;
}

void indicatorBufferLoad(Pcf7584Control_t* hLedIndicator)
{
	uint8_t bit = 0;
	if((hLedIndicator->indicatorLedSet == RESET) && (hLedIndicator->indicatorCounter - 1 < 0))
	{
		hLedIndicator->indicatorBuffer = RESET;				//Reset the indicator buffer if the buffer counter is 0 and the incoming command
															//is to turn off one more LED
	}
	else
	{
		if(hLedIndicator->indicatorLedSet == SET)			//One more LED will be turned on
		{													//Select the bit to be set
			bit |= 1 << (hLedIndicator->indicatorCounter - 1);
			SET_BIT(hLedIndicator->indicatorBuffer, bit);	//Set one bit in the indicator buffer
		}
		else if(hLedIndicator->indicatorLedSet == RESET) 	//One more LED will be turned off
		{
			bit |= 1 << (hLedIndicator->indicatorCounter);	//Select the bit to be cleared
			CLEAR_BIT(hLedIndicator->indicatorBuffer, bit); //clear one bit in the indicator buffer
		}
	}

}
void sirenHandler(SoundTest_t* hSiren)
{
	  if(!hSiren->testTimerEnable)							//In case of sound test don't turn off the buzzer
	  {
		  htim2.Instance->CCR1  = RESET;					//Set duty cycle to be zero
	  }
	  else
	  {
		  htim2.Instance->PSC 	= 100 + hSiren->testCounter;	//Sound test related part the pre-scaler is modified continuously
		  htim2.Instance->CCR1  = hSiren->testSoundLevel;		//to output different tones
		  if(checkTimer(&hSiren->testTimer, 15))
		  {
			  setTimer(&hSiren->testTimer);
			  if(hSiren->testDirection)
			  {
				  hSiren->testCounter += 1;					//Lower frequency of PWM signal used while the sound test mode is ON
				  if(hSiren->testCounter >= 250)
				  {
					  hSiren->testDirection ^= 1;			//Used to toggle between increasing and decreasing frequency tone
				  }
			  }
			  else
			  {
				  hSiren->testCounter -= 1;					//Increase frequency signal used while the sound test mode is ON
				  if(hSiren->testCounter <= 150)
				  {
					  hSiren->testDirection ^= 1;
				  }
			  }

		  }

	  }
}

bool adcKeyboardHandler	(uint16_t* adcBuffer, uint16_t soundLevel)
{
	bool ret 	= DISABLE;
	if(adcBuffer[0] < 3900)										//it is possible that one of the buttons has been pressed
	{																//Every led will have a debounce function
	  ret = ENABLE;
	  if((adcBuffer[0] <= 500) && (adcBuffer[0] > 1)) //First region
	  {
		  if(Button_DeBounce(adcBuffer))
		  {
			  htim2.Instance->PSC 	= 125;
			  htim2.Instance->CCR1  = soundLevel;
		  }
	  }
	  else if((adcBuffer[0] <= 1250) && (adcBuffer[0] > 750)) //Second region
	  {
		  if(Button1_DeBounce(adcBuffer))
		  {
			  htim2.Instance->PSC 	= 150;
			  htim2.Instance->CCR1  = soundLevel;
		  }
	  }
	  else if((adcBuffer[0] <= 1800) && (adcBuffer[0] > 1300)) //third region
	  {
		  if(Button2_DeBounce(adcBuffer))
		  {
			  htim2.Instance->PSC 	= 175;
			  htim2.Instance->CCR1  = soundLevel;
		  }
	  }
	  else if((adcBuffer[0] <= 2300) && (adcBuffer[0] > 1800)) //Forth region
	  {
		  if(Button3_DeBounce(adcBuffer))
		  {
			  htim2.Instance->PSC 	= 200;
			  htim2.Instance->CCR1  = soundLevel;
		  }
	  }
	  else if((adcBuffer[0] <= 2900) && (adcBuffer[0] > 2400)) //Fifth region
	  {
		  if(Button4_DeBounce(adcBuffer))
		  {
			  htim2.Instance->PSC 	= 225;
			  htim2.Instance->CCR1  = soundLevel;
		  }
	  }
	  else if((adcBuffer[0] <= 3400) && (adcBuffer[0] > 3000)) //Sixth region
	  {
		  if(Button5_DeBounce(adcBuffer))
		  {
			  htim2.Instance->PSC 	= 250;
			  htim2.Instance->CCR1  = soundLevel;
		  }
	  }
	  else if((adcBuffer[0] <= 3900) && (adcBuffer[0] > 3500)) //Seventh region
	  {
		  if(Button6_DeBounce(adcBuffer))
		  {
			  htim2.Instance->PSC 	= 275;
			  htim2.Instance->CCR1  = soundLevel;
		  }
	  }
	}
	  else
	  {
		  Button_DeBounce (adcBuffer);										//Discharge software capacitors
		  Button1_DeBounce(adcBuffer);
		  Button2_DeBounce(adcBuffer);
		  Button3_DeBounce(adcBuffer);
		  Button4_DeBounce(adcBuffer);
		  Button5_DeBounce(adcBuffer);
		  Button6_DeBounce(adcBuffer);
	  }
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
