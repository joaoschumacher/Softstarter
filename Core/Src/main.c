/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Programa feito para controlar o projeto de Softstarter. Com ele é possível variar
  * 				o tempo de descida e subida de 5 a 50 segundos. O projeto também possui o sistema de
  * 				proteção contra sobrecorrente e queda de luz.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
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

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
 uint16_t conta=0, grau=1000, estado=0, Tace=10, Tdes=10, flag10=0, queda=0, quedaTemp=0, SELECT=0;
 uint16_t analog_value=0, variacao=0, c, contaCurto;
 _Bool REPIK=0, ACELERANDO=0, DESACELERANDO=0, VMAX=0, DESLIGADO=1, PINO=0, FLAG=1, QUEDA=1, INI=0, INItemp=0, REDES=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
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
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();	// Inicia o LCD
  lcd_clear();	// Limpa o que está escrito
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  INI = HAL_GPIO_ReadPin(INHIBIT_GPIO_Port, INHIBIT_Pin); //	Lê o pino de sobrecorrente
	  if(INI && VMAX){	// Se o motor já estiver em velocidade nominal e o circuito de sobrecorrente aciona
			HAL_GPIO_WritePin(BYPASS_GPIO_Port, BYPASS_Pin, 0); //	Desliga o bypass e reseta os parâmetros
			grau=1000;
			conta=0;
			ACELERANDO=0;
			DESACELERANDO=0;
			VMAX=0;
			DESLIGADO=1;
			FLAG=1;
		}

	  REDES = HAL_GPIO_ReadPin(REDE_GPIO_Port, REDE_Pin);	//	Lê o pino do circuito pra queda de energia
	 if(!REDES && VMAX){	// Se o motor já estiver em velocidade nominal e o circuito de queda desligar
		HAL_GPIO_WritePin(BYPASS_GPIO_Port, BYPASS_Pin, 0); //	Desliga o bypass e reseta os parâmetros
		grau=1000;
		conta=0;
		ACELERANDO=0;
		DESACELERANDO=0;
		VMAX=0;
		DESLIGADO=1;
		FLAG=1;
	 }

	  PINO = HAL_GPIO_ReadPin(DETECT_GPIO_Port, DETECT_Pin); // Lê o pino do circuito do detector de zero
	  if(PINO){	//	IF para sincronizar os timers com a frequência da rede
		  HAL_TIM_Base_Start_IT(&htim2);
		  HAL_TIM_Base_Start_IT(&htim3);
	  }
	  if(REPIK==1 && FLAG==1){	//	IF para tirar o REPIK das entradas
		  HAL_Delay(100);
		  while((HAL_GPIO_ReadPin(BOTAO1_GPIO_Port, BOTAO1_Pin) == 0)&&(HAL_GPIO_ReadPin(BOTAO2_GPIO_Port, BOTAO2_Pin) == 0)&&(HAL_GPIO_ReadPin(BOTAO3_GPIO_Port, BOTAO3_Pin)==0)&&(HAL_GPIO_ReadPin(INHIBIT_GPIO_Port, INHIBIT_Pin)==0)&&(HAL_GPIO_ReadPin(REDE_GPIO_Port, REDE_Pin) == 0)){}
		  HAL_Delay(100);
		  REPIK=0;
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 6;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, OUT_TO_GATE_Pin|BYPASS_Pin|D4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|D7_Pin|D6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_RST_Pin|LCD_CS_Pin|LCD_SCK_Pin|LCD_DC_Pin
                          |LCD_MO_Pin|LCD_EN_Pin|LCD_RS_Pin|D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DETECT_Pin */
  GPIO_InitStruct.Pin = DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OUT_TO_GATE_Pin */
  GPIO_InitStruct.Pin = OUT_TO_GATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OUT_TO_GATE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INHIBIT_Pin */
  GPIO_InitStruct.Pin = INHIBIT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INHIBIT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BYPASS_Pin D4_Pin */
  GPIO_InitStruct.Pin = BYPASS_Pin|D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BOTAO3_Pin BOTAO2_Pin */
  GPIO_InitStruct.Pin = BOTAO3_Pin|BOTAO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin D7_Pin D6_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|D7_Pin|D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : REDE_Pin */
  GPIO_InitStruct.Pin = REDE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(REDE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOTAO1_Pin */
  GPIO_InitStruct.Pin = BOTAO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BOTAO1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RST_Pin LCD_CS_Pin LCD_SCK_Pin LCD_DC_Pin
                           LCD_MO_Pin LCD_EN_Pin LCD_RS_Pin D5_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin|LCD_CS_Pin|LCD_SCK_Pin|LCD_DC_Pin
                          |LCD_MO_Pin|LCD_EN_Pin|LCD_RS_Pin|D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) 	// Interrupções externas
{
	if(REPIK == 0 && QUEDA){
		if(GPIO_Pin == BOTAO2_Pin && !ACELERANDO && !DESACELERANDO){	// Determina qual parâmetro será alterado (Tempo de aceleração ou desaceleração)
			if (SELECT == 0) {
				SELECT = 1;
			} else if (SELECT == 1){
				SELECT = 2;
			} else if (SELECT == 2){
				SELECT = 0;
			}
		}
		if(GPIO_Pin == BOTAO3_Pin && !ACELERANDO && !DESACELERANDO){	// Começa a acelerar depois de apertar o botão
			DESLIGADO=0;
			ACELERANDO=1; // ACELERA
			FLAG=0;
		}
		if(GPIO_Pin == BOTAO1_Pin && !ACELERANDO && !DESACELERANDO){	// Desacelera depois de apertar o botão
			HAL_GPIO_WritePin(BYPASS_GPIO_Port, BYPASS_Pin, 0);
			VMAX = 0;
			DESACELERANDO=1; // DESACELERA
			FLAG=0;
		}
		REPIK=1;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim ->Instance==TIM3){ // timer de 100ms
		lcd_clear();
		HAL_ADC_Start(&hadc1); // Inicia a leitura do pino
		if(HAL_ADC_PollForConversion(&hadc1,10)==HAL_OK){
			// Coloca o valor já digital dentro da váriavel
			analog_value=HAL_ADC_GetValue(&hadc1);
		}
		HAL_ADC_Stop(&hadc1); // Para de ler o pino
		variacao = analog_value * (0.011601563); // (45 variacoes/3878,79) fazendo com que seja possível variar de 5 a 50 segundos
		if(SELECT==0 && !ACELERANDO && !DESACELERANDO && !VMAX){
			Tace = variacao+5;
			lcd_goto(0xC0);
			lcd_puts(">");
		} else if (SELECT==1 && !ACELERANDO && !DESACELERANDO && !VMAX) {
			Tdes = variacao+5;
			lcd_goto(0xC8);
			lcd_puts(">");
		}

		lcd_goto(0xC1);
		lcd_puts("Ts:");
		lcd_printd(Tace);
		lcd_puts("s");
		lcd_goto(0xC9);
		lcd_puts("Td:");
		lcd_printd(Tdes);
		lcd_puts("s");



		if(grau<(1000/(Tace*10)) && ACELERANDO){
			grau=0;
			ACELERANDO=0;
			VMAX=1;
			FLAG=1;
			HAL_GPIO_WritePin(BYPASS_GPIO_Port, BYPASS_Pin, 1);
		}
		if(ACELERANDO){
			grau = grau - (1000 / (Tace*10));
		}
		if(grau>(1000-(1000/(Tdes*10))) && DESACELERANDO){
			grau = 1000;
			DESACELERANDO = 0;
			DESLIGADO = 1;
			FLAG=1;
		}
		if(DESACELERANDO){
			grau = grau + (1000 / (Tdes*10));
		}
		if(ACELERANDO){
			lcd_goto(0x83);
			lcd_puts("ACELERANDO");
		}
		if(DESACELERANDO){
			lcd_goto(0x81);
			lcd_puts("DESACELERANDO");
		}
		if(VMAX){
			lcd_goto(0x83);
			lcd_puts("VEL. MAX");
		}
		if(DESLIGADO){
			lcd_goto(0x83);
			lcd_puts("DESLIGADO");
		}

	}
	if(htim ->Instance==TIM2){ // ((1/60Hz)/2)/1000 partes = timer de 8,333us
		 REDES = HAL_GPIO_ReadPin(REDE_GPIO_Port, REDE_Pin);

		 if(!REDES){
			HAL_GPIO_WritePin(BYPASS_GPIO_Port, BYPASS_Pin, 0);
			grau=1000;
			conta=0;
			ACELERANDO=0;
			DESACELERANDO=0;
			VMAX=0;
			DESLIGADO=1;
			FLAG=1;
		 }
		conta++;
		if(conta==1000){ // Desliga os timers para poder sincronizar com o detector de zero
			conta=0;
			HAL_TIM_Base_Stop_IT(&htim2);
			HAL_TIM_Base_Stop_IT(&htim3);
		}
		if(conta >= grau){ // Gera os pulsos de acordo com o grau de inicio, formando o PWM
			HAL_GPIO_WritePin(OUT_TO_GATE_GPIO_Port, OUT_TO_GATE_Pin, 1);
		}else{
			HAL_GPIO_WritePin(OUT_TO_GATE_GPIO_Port, OUT_TO_GATE_Pin, 0);
		}

	}
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
