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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "string.h"
#include "stdio.h"

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
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t pwm;
int act = 0;
volatile uint32_t adc_val = 0;
uint32_t adc_val_buffer[1];


int average_16(int x) {
    static int samples[16];
    static int i = 0;
    static int total = 0;

    /* Update the moving average */
    total += x - samples[i];
    samples[i] = x;

    /* Update the index */
    i = (i == 15 ? 0 : i + 1);

    return total >> 4; // Divide by 16
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT (&htim2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_ADC_Start_DMA(&hadc1, adc_val_buffer, 1);

  float dutyCycle = 0.0;

  char m1[]= "Press button 2 to control LED \r\n";
  char m2[]= "Enter m to switch manualmode\r\n";
  char m3[]= "\r\nEnter a to switch automode\r\n";
  char m4[]= "\r\nAutomode activated \r\nEnter r to increase\r\n";
  char m5[]= "Input => ";
  char m6[] = "\r\nManualMode activated \r\n";

  char receivedChar;

  HAL_UART_Transmit(&huart3, (uint8_t*)m1, strlen(m1), HAL_MAX_DELAY);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if (act == 1){
		  HAL_UART_Transmit(&huart3, (uint8_t*)m3, strlen(m3), HAL_MAX_DELAY);
		  HAL_UART_Transmit(&huart3, (uint8_t*)m2, strlen(m2), HAL_MAX_DELAY);
		  HAL_UART_Transmit(&huart3, (uint8_t*)m5, strlen(m5), HAL_MAX_DELAY);

		  if (HAL_UART_Receive(&huart3, (uint8_t*)&receivedChar, 1, HAL_MAX_DELAY) == HAL_OK){
			  HAL_UART_Transmit(&huart3, (uint8_t*)&receivedChar, 4, HAL_MAX_DELAY);
			  HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", strlen("\r\n"), HAL_MAX_DELAY);
			  if(receivedChar == 'a'){
				  HAL_UART_Transmit(&huart3, (uint8_t*)m4, strlen(m4), HAL_MAX_DELAY);

					  while(1){
						  HAL_UART_Transmit(&huart3, (uint8_t*)m5, strlen(m5), HAL_MAX_DELAY);
						  if (HAL_UART_Receive(&huart3, (uint8_t*)&receivedChar, 1, HAL_MAX_DELAY) == HAL_OK){
							  if(receivedChar == 'r'){
								  HAL_UART_Transmit(&huart3, (uint8_t*)&receivedChar, 1, HAL_MAX_DELAY);
								  HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", strlen("\r\n"), HAL_MAX_DELAY);

								  dutyCycle += 0.2;
								  if(dutyCycle > 1.0){
									  dutyCycle=0.0;
								  }
								  pwm = (GPIOD->IDR & GPIO_PIN_12) >> 12;
								  htim4.Instance->CCR1 = (10000 - 1) * dutyCycle;
							  }
							  else{
								  HAL_UART_Transmit(&huart3, (uint8_t*)&receivedChar, 1, HAL_MAX_DELAY);
								  HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", strlen("\r\n"), HAL_MAX_DELAY);
								  break;
							  }
					  	  }
					  }
			  	  }
			  if(receivedChar == 'm'){
				  HAL_UART_Transmit(&huart3, (uint8_t*)m6, strlen(m6), HAL_MAX_DELAY);
				  HAL_UART_Transmit(&huart3, (uint8_t*)"type s to stop \r\n", strlen("type s to stop \r\n"), HAL_MAX_DELAY);
				  while (1) {
				      // Check for ADC conversion and handle brightness adjustment
				      if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
				          adc_val = HAL_ADC_GetValue(&hadc1);
				          HAL_ADC_Start_DMA(&hadc1, adc_val_buffer, 1);
				          adc_val = average_16(adc_val);

				          if (adc_val >= 0 && adc_val <= 819) {
				              dutyCycle = 0.2;
				          } else {
				              dutyCycle = 1.0;
				          }
				          htim4.Instance->CCR1 = (10000 - 1) * dutyCycle;
				      }

				      // Check for UART input to see if the user typed 's'
				      if (HAL_UART_Receive(&huart3, (uint8_t*)&receivedChar, 1, 10) == HAL_OK) {
				          if (receivedChar == 's') {
				              break;  // Exit the loop when 's' is received
				          }
				      }
				  }
			  }
		  }
	  }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if (htim->Instance == TIM2){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_0){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	}
	if(GPIO_Pin == GPIO_PIN_1){
		  HAL_Delay(200);
		  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
		  act = 1;
	}
	if (GPIO_Pin == GPIO_PIN_2){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
		act = 1;
	}
}



/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
