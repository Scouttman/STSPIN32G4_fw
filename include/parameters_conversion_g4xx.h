/**
  ******************************************************************************
  * @file    parameters_conversion_g4xx.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the definitions needed to convert MC SDK parameters
  *          so as to target the STM32G4 Family.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PARAMETERS_CONVERSION_G4XX_H
#define PARAMETERS_CONVERSION_G4XX_H



/* USER CODE BEGIN Additional parameters */
/************************* CPU & ADC PERIPHERAL CLOCK CONFIG ******************/
#define SYSCLK_FREQ      170000000uL
#define TIM_CLOCK_DIVIDER  1
#define ADV_TIM_CLK_MHz  170 /* Actual TIM clk including Timer clock divider*/
#define ADC_CLK_MHz     42
#define HALL_TIM_CLK    170000000uL
#define APB1TIM_FREQ 170000000uL

#define HTMIN 1 /* Required for main.c compilation only, CCR4 is overwritten at runtime */

/*************** Timer for PWM generation & currenst sensing parameters  ******/
#define PWM_FREQUENCY   30000
#define PWM_PERIOD_CYCLES (uint16_t)((ADV_TIM_CLK_MHz*(uint32_t)1000000u/((uint32_t)(PWM_FREQUENCY)))& ( uint16_t )0xFFFE)

#define SW_DEADTIME_NS                   700 /*!< Dead-time to be inserted*/
#define DEADTIME_NS  SW_DEADTIME_NS

#define DEAD_TIME_ADV_TIM_CLK_MHz (ADV_TIM_CLK_MHz * TIM_CLOCK_DIVIDER)
#define DEAD_TIME_COUNTS_1  (DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/1000uL)

#if (DEAD_TIME_COUNTS_1 <= 255)
#define DEAD_TIME_COUNTS (uint16_t) DEAD_TIME_COUNTS_1
#elif (DEAD_TIME_COUNTS_1 <= 508)
#define DEAD_TIME_COUNTS (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/2) /1000uL) + 128)
#elif (DEAD_TIME_COUNTS_1 <= 1008)
#define DEAD_TIME_COUNTS (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/8) /1000uL) + 320)
#elif (DEAD_TIME_COUNTS_1 <= 2015)
#define DEAD_TIME_COUNTS (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/16) /1000uL) + 384)
#else
#define DEAD_TIME_COUNTS 510
#endif


/* Torque and flux regulation loops */
#define REGULATION_EXECUTION_RATE     1    /*!< FOC execution rate in*/
#define REP_COUNTER 			(uint16_t) ((REGULATION_EXECUTION_RATE *2u)-1u)

#define DEAD_TIME_COUNTS_1  (DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/1000uL)

#define DEAD_TIME_COUNTS (uint16_t) DEAD_TIME_COUNTS_1

/* USER CODE END Additional parameters */

#endif /*PARAMETERS_CONVERSION_G4XX_H*/

/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
