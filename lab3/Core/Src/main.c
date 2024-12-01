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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  char inputPrompt[] = "Input => ";
  char quitMessage[] = "QUIT";
  char receivedChar;
  char unknownMessage[] = "Unknown Command\r\n";
  char inputPrompt2[] = "Display Blinking LED PRESS (1, 2)\r\nDisplay Group Members PRESS m\r\nQuit PRESS q \r\n\t Input =>";
  char studentPrompt[] = "65011662\r\nLwin Moe Aung\r\n65011657\r\nKaung Khant Zaw\r\n65011584\r\nThanat Ratti_ekkaphat\r\n";
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  HAL_UART_Transmit(&huart3, (uint8_t*)inputPrompt, strlen(inputPrompt), HAL_MAX_DELAY);
//
//	          // Wait for a character from the user
//	          if (HAL_UART_Receive(&huart3, (uint8_t*)&receivedChar, 1, HAL_MAX_DELAY) == HAL_OK) {
//	              // Echo the received character back
//	              HAL_UART_Transmit(&huart3, (uint8_t*)&receivedChar, 1, HAL_MAX_DELAY);
//
//	              HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
//
//	              // Check if the received character is 'q'
//	              if (receivedChar == 'q') {
//	                  // Display the quit message
//	                  HAL_UART_Transmit(&huart3, (uint8_t*)quitMessage, strlen(quitMessage), HAL_MAX_DELAY);
//	                  // Stop execution
//	                  break;
//	              }
//	          }






	  // NEW user code
	  HAL_UART_Transmit(&huart3, (uint8_t*)inputPrompt2, strlen(inputPrompt2), HAL_MAX_DELAY);

	  if (HAL_UART_Receive(&huart3, (uint8_t*)&receivedChar, 1, HAL_MAX_DELAY) == HAL_OK) {
			  // Echo the received character back
			  HAL_UART_Transmit(&huart3, (uint8_t*)&receivedChar, 1, HAL_MAX_DELAY);

			  HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

			  // Check if the received character is 'q'
			  if (receivedChar == 'q') {
				  // Display the quit message
				  HAL_UART_Transmit(&huart3, (uint8_t*)&quitMessage, strlen(quitMessage), HAL_MAX_DELAY);
				  // Stop execution
				  break;
			  }

			  if (receivedChar != 'q' && receivedChar != '1' && receivedChar != '2' && receivedChar != 'm') {
					  // Display the Unknown message
					  HAL_UART_Transmit(&huart3, (uint8_t*)&unknownMessage, strlen(unknownMessage), 1000);


				  }

			if (receivedChar == 'm') {
			  // Display the quit message
			  HAL_UART_Transmit(&huart3, (uint8_t*)&studentPrompt, strlen(studentPrompt), HAL_MAX_DELAY);
		  }
			if (receivedChar == '1') {
				for (int i = 0; i < 3; i++) {
					// Set the LED on
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
					HAL_Delay(500);  // Delay for 500 ms
					// Set the LED off
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
					HAL_Delay(500);  // Delay for 500 ms
				}
			}

			if (receivedChar == '2') {
				for (int i = 0; i < 3; i++) {
					// Set the LED on
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
					HAL_Delay(500);  // Delay for 500 ms
					// Set the LED off
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
					HAL_Delay(500);  // Delay for 500 ms
				}
			}
	  }
	  // End of new code

  }

  while(1){

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
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
