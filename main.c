/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  SystemClock_Config();

	RCC-> AHBENR |= RCC_AHBENR_GPIOCEN; // Enable the GPIOC clock in the RCC
	// enable tim2 on rcc
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	// enable tim3 on rcc
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	
	// Configure LEDS -------------------------------------
	// Set pins PC6 PC7 to alternate mode and PC8 PC9 to output mode
	GPIOC->MODER |= (1 << 13) |  (1 << 15)  | (1 <<16) | (1 << 18);
	GPIOC->MODER &= ~((1 << 12) | (1 << 14)| (1 << 17) | (1 << 19));
	// Set pins PC6 PC7 PC8 and PC9 to output push-pull
	GPIOC->OTYPER &= ~((1 << 6) | (1 << 7) | (1 << 8) |(1 << 9));
	// Set pins PC6 PC7 PC8 and PC9 to low speed
	GPIOC->OSPEEDR &= ~((1 << 12) | (1 << 14) | (1 << 16) | (1 <<18));
	// Set pins PC6 PC7 PC8 and PC9 to no pullup, pulldown
	GPIOC->PUPDR &= ~((1 << 12) | (1 << 13) | (1 << 14) | (1 << 15) | (1 << 16) | (1 << 17) | (1 << 18) | (1 << 19));
	
	// Initialize LEDS ------------
	// Set PC6 (red lED) high
	GPIOC->ODR |= (1 << 6);
	// Set PC7 (blue LED) low
	GPIOC->ODR &= ~(1 << 7);
	// Set PC8 (orange LED) high
	GPIOC->ODR |= (1 << 8);
	// Set PC9 (green LED) low
	GPIOC->ODR &= ~(1 << 9);
	
	// CONFIGURE TIM2 ---------------------------------------
	// set prescaler to divide by 20,000 , then count by 100
	// 4 Hz
	TIM2->PSC = 19999;
	TIM2->ARR = 100;
	
	// enable tim2 update interrupt (UIE)
	TIM2->DIER |= (1 << 0);
	
	// enable UEV
	// TIM2->CR1 &= ~(1 << 1);
	// enable counter
	TIM2->CR1 |= (1 << 0);
	
	// enable tim2 interrupt in NVIC
	NVIC_EnableIRQ(TIM2_IRQn);
	
	
	// CONFIGURE TIM3 ---------------------------------------
	// set prescaler to divide by 1000, then count by 10
	TIM3->PSC = 39;
	TIM3->ARR = 25;
	
	
	// confuigure PWM mode -------------
	
	// set CCr value
	TIM3->CCR1 = 5;
	TIM3->CCR2 = 5;
	
	// output mode on all channels
	TIM3->CCMR1 &= ~((1 << 0) | (1 << 1) | (1 << 8) | (1 << 9));
	// pwm mode 1 on channel 1
	TIM3->CCMR1 |= ( (1 << 4) | (1 << 5) | (1 << 6) );
	// pwm mode 2 on channel 2
	TIM3->CCMR1 |= ( (1 << 14) | (1 << 13) );
	TIM3->CCMR1 &= ~(1 << 12);
	// enable output compare preload channels 1 and 2
	TIM3->CCMR1 |= ( (1 << 11) | (1 << 3) );
	
	// enable output in CCER
	TIM3->CCER |= ( (1 << 0) | (1 << 4) );
	
	TIM3->CR1 |= (1 << 0);
	TIM3->EGR |= TIM_EGR_UG;
	
	GPIOC->AFR[0] = GPIO_AF0_TIM3;
	
	
  /* Infinite loop */
  while (1)
  {
    
  }

}

void TIM2_IRQHandler(void){
	
	// toggle green (PC9) and orange (PC8) LEDS
	GPIOC->ODR ^= (1 << 8);
	GPIOC->ODR ^= (1 << 9);
	
	// clear pending flag bit 0 UIF
	TIM2->SR &= ~(1 << 0);
	
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

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
