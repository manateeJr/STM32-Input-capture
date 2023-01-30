/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void TIM2_Init();
void TIM3_Init();
void TIM4_Init();
void Ultrasonic_Init();
void LED_Init();

volatile int rising_edge = 1;
volatile int capture_start = 0;
volatile int capture_end = 0;
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  Ultrasonic_Init();
  LED_Init();
  TIM2_Init();
  TIM3_Init();
  TIM4_Init();
  int time_us;
  int distance_cm;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  time_us = capture_end - capture_start;
	  distance_cm = time_us / 58;
	  if (distance_cm < 2) {
		  TIM2->CCR2 = 1;
	  }
	  else if (distance_cm < 25 && distance_cm > 2) {
		  TIM2->CCR2 = distance_cm * 45;
	  }
	  else if (distance_cm > 25){
		  TIM2->CCR2 = 999;
	  }
  }
  /* USER CODE END 3 */
}

void TIM3_Init() { // trigger D4 (PB5), channel 2 // THIS TIMER WORKS
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // enable timer 3
	TIM3->PSC = 16 - 1; // 1mhZ frequency
	TIM3->CNT = 0; // reset counter
	TIM3->ARR = 0xFFFF; // overflow event every 65ms
	TIM3->CCR2 = 10 - 1; // 10uS high output every 65ms

	TIM3->CCMR1 = 0b110 << 12; // pwm mode 1
	TIM3->CCER |= 0b1 << 4; // enable output compare ch2

	TIM3->CR1 |= 0b1; // enable counting
}

void TIM4_Init() { // echo D10 (PB6), channel 1
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; // enable timer 4
	TIM4->PSC = 16 - 1; // 1mhZ frequency
	TIM4->CNT = 0; // reset counter
	TIM4->ARR = 0xFFFF; // overflow every 65ms // ARR register ok

	TIM4->CCMR1 &= ~TIM_CCMR1_IC1F; // set filter to 0
	TIM4->CCMR1 &= ~TIM_CCMR1_CC1S; // clear cc1s bits
	TIM4->CCMR1 |= TIM_CCMR1_CC1S_0; // set first bit in cc1s (IC1 mapped on TI1)
	TIM4->CCMR1 &= ~TIM_CCMR1_IC1PSC; //clear input channel 1 prescaler bits, meaning capture is done on each rising/falling edge CCMR1 register OK

	TIM4->CCER |= TIM_CCER_CC1P | TIM_CCER_CC1NP; // configure polarity (trigger on both rising and falling edge)
	TIM4->CCER |= 0b1; // enable capture CCER register ok

	TIM4->DIER |= TIM_DIER_CC1IE; // enable interrupts channel 1 WORKS
	NVIC_EnableIRQ(TIM4_IRQn); // enable interrupts WORKS
	TIM4->CR1 |= 0b1; // enable counter // COUNTER works
}

void TIM4_IRQHandler(void) {
	if (TIM4->SR & TIM_SR_UIF) { // counter overflow
		TIM4->SR &= ~TIM_SR_UIF;
	}
	else if (TIM4->SR & TIM_SR_CC1IF) { // event caused interrupt
		TIM4->SR &= ~TIM_SR_CC1IF;
		if (rising_edge) {
			capture_start = TIM4->CCR1;
			rising_edge = 0;
		}
		else {
			capture_end = TIM4->CCR1;
			rising_edge = 1;
		}
	}
}

void Ultrasonic_Init() {
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	GPIOB->MODER &= ~GPIO_MODER_MODER5; // clear moder
	GPIOB->MODER &= ~GPIO_MODER_MODER6;

	GPIOB->MODER |= 0b10 << GPIO_MODER_MODER5_Pos; // set PB5 to AF
	GPIOB->MODER |= 0b10 << GPIO_MODER_MODER6_Pos; // set PB6 to AF

	GPIOB->AFR[0] &= ~GPIO_AFRL_AFRL5; // clear AFR for PB5, PB6
	GPIOB->AFR[0] &= ~GPIO_AFRL_AFRL6;

	GPIOB->AFR[0] |= 0b0010 << GPIO_AFRL_AFRL5_Pos; // AF2
	GPIOB->AFR[0] |= 0b0010 << GPIO_AFRL_AFRL6_Pos; // AF2

	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_5; // set output type
}

void LED_Init() {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	GPIOA->MODER &= ~GPIO_MODER_MODER0;
//	GPIOA->MODER &= ~GPIO_MODER_MODER1;
	GPIOA->MODER &= ~GPIO_MODER_MODER5;

	GPIOA->MODER |= 0b01 << GPIO_MODER_MODER0_Pos;
//	GPIOA->MODER |= 0b01 << GPIO_MODER_MODER1_Pos;
	GPIOA->MODER |= 0b01 << GPIO_MODER_MODER5_Pos;


	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_0;
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_1;
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_5;

	GPIOA->MODER &= ~GPIO_MODER_MODER1; // PA1 external
	GPIOA->MODER |= 0b10 << GPIO_MODER_MODER1_Pos; // alternate function
	GPIOA->AFR[0] = (GPIOA->AFR[0] & ~GPIO_AFRL_AFRL1);
	GPIOA->AFR[0] |= (0b0001 << GPIO_AFRL_AFRL1_Pos);

}

void TIM2_Init() {
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // enable timer clock
	TIM2->PSC = 16 - 1; // set prescaler to 16, so that we get a convenient reload value. PSC of 16 gives us a clock frequency of 1mhZ
	TIM2->ARR = 1000 - 1; // Clock/prescaler * arr = (16 * 10 ^ 6) / (16 * (10 ^ 3)) = 10 ^ 3 = 1khZ frequency PWM signal (1000 cycles in a second or a tick each millisecond)
	TIM2->CNT = 0; // reset counter
	TIM2->CR1 |= 0b1; // enable counter for timer2
	TIM2->CCMR1 |= 0b111 << 12; // pwm mode 2 (low - high) for ch2 tim 2
	TIM2->CCER |= 0b1 << 4; // enable output compare ch 2
}






/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
