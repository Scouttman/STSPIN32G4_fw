#include <Arduino.h>
#include "main.h"
#include "pin.h"
#include "stspin32g4.h"
// #include <SimpleFOC.h>

#ifdef SIMPLEFOC_H
BLDCMotor motor = BLDCMotor(11);
// BLDC driver instance
BLDCDriver6PWM driver = BLDCDriver6PWM(INH1,INL1,INH2,INL2,INH3,INL3);
// int phA_h,int phA_l,int phB_h,int phB_l,int phC_h,int phC_l, int en = NOT_SET
float target_velocity = 10;
#endif

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);

I2C_HandleTypeDef hi2c3;
STSPIN32G4_HandleTypeDef HdlSTSPING4;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;


void setup() {
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
  pinMode(LED_WHITE, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  digitalWrite(LED_WHITE, HIGH);
  digitalWrite(LED_YELLOW, LOW);

  pinMode(WAKE, OUTPUT);
  pinMode(READY, INPUT_PULLUP);
  pinMode(NFAULT, INPUT_PULLUP);
  digitalWrite(WAKE, HIGH); // STSPIN init should set this high but why not
  MX_I2C3_Init();
  MX_TIM1_Init();
  //   /* USER CODE BEGIN 2 */
  #ifdef SIMPLEFOC_H
  // pwm frequency to be used [Hz]
  // for atmega328 fixed to 32kHz
  // esp32/stm32/teensy configurable
  driver.pwm_frequency = 12000;
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  // Max DC voltage allowed - default voltage_power_supply
  driver.voltage_limit = 12;
  // dead_zone [0,1] - default 0.02f - 2%
  driver.dead_zone = 0.05f;

  // driver init
  driver.init();

  // enable driver
  driver.enable();
  #else
  
  // Maybe this needs to come after?
  if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK){Error_Handler();}
  if(HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK){Error_Handler();}
//  if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK){Error_Handler();}
  if(HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK){Error_Handler();}
//  if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK){Error_Handler();}
  if(HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3) != HAL_OK){Error_Handler();}
  #endif

   /*************************************************/
   /*   STSPIN32G4 driver component initialization  */
   /*************************************************/
  STSPIN32G4_init( &HdlSTSPING4 );
  // Purge registers to default values
  if(STSPIN32G4_reset(&HdlSTSPING4) != STSPIN32G4_OK){
    Error_Handler();
  }

  // Disable the Buck regulator and demap VCC UVLO on NFAULT pin
  STSPIN32G4_confVCC blah;
  STSPIN32G4_confVCC vcc = {.voltage = blah._EXT, .useNFAULT=true, .useREADY=false};
  if(STSPIN32G4_setVCC(&HdlSTSPING4, vcc) != STSPIN32G4_OK){Error_Handler();}

  // Configure VDSP protection with 4us deglitch time and map triggering on NFAULT pin
  STSPIN32G4_confVDSP hmm;
  STSPIN32G4_confVDSP vdsp = {.deglitchTime=hmm._6us, .useNFAULT=true};
  if(STSPIN32G4_setVDSP(&HdlSTSPING4, vdsp) != STSPIN32G4_OK){Error_Handler();}

  // After reset a clearing of fault is needed to enable GHSx/GLSx outputs
  if(STSPIN32G4_clearFaults(&HdlSTSPING4) != STSPIN32G4_OK){Error_Handler();}

}

void loop() {
  digitalWrite(LED_WHITE, !digitalRead(READY));
  digitalWrite(LED_YELLOW, !digitalRead(NFAULT));

	uint8_t tmp = 0;
	STSPIN32G4_readReg(&HdlSTSPING4, STSPIN32G4_I2C_STATUS, &tmp);
	STSPIN32G4_statusRegTypeDef statusReg;
	if(STSPIN32G4_getStatus(&HdlSTSPING4, &statusReg) != STSPIN32G4_OK){Error_Handler();}

	if(statusReg.vdsp){
		Error_Handler(); // over voltage protection triggered
	}
	if(statusReg.thsd){
		Error_Handler(); // Thermal shutdown
	}
	if(statusReg.vccUvlo){
		Error_Handler(); // VCC UVLO (voltage to low)
	}

  #ifdef SIMPLEFOC_H
  driver.setPwm(9,6,3);
  #endif
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x30A0A7FB;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
//	  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
//	  {
//		Error_Handler();
//	  }
	// without the above I get error 128 but the led dose not flash...

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 4;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2833;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 120;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 2124;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 708;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 100;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // /*Configure GPIO pin Output Level */
  // HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);

  // /*Configure GPIO pin Output Level */
  // HAL_GPIO_WritePin(WAKE_GPIO_Port, WAKE_Pin, GPIO_PIN_RESET);

  // /*Configure GPIO pin : LED_1_Pin */
  // GPIO_InitStruct.Pin = LED_1_Pin;
  // GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  // GPIO_InitStruct.Pull = GPIO_NOPULL;
  // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  // HAL_GPIO_Init(LED_1_GPIO_Port, &GPIO_InitStruct);

  // /*Configure GPIO pin : WAKE_Pin */
  // GPIO_InitStruct.Pin = WAKE_Pin;
  // GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  // GPIO_InitStruct.Pull = GPIO_NOPULL;
  // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  // HAL_GPIO_Init(WAKE_GPIO_Port, &GPIO_InitStruct);

  // /*Configure GPIO pins : READY_Pin NFAULT_Pin */
  // GPIO_InitStruct.Pin = READY_Pin|NFAULT_Pin;
  // GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  // GPIO_InitStruct.Pull = GPIO_PULLUP;
  // HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}
