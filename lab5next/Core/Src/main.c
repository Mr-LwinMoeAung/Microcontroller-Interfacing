/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>  // For strlen function
#include "stm32f7xx_hal.h"

/* Private variables ---------------------------------------------------------*/

volatile uint32_t adc_val = 0;  // Variable to store ADC value


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
int avg_val_8 = 0;  // Variable to store 8-sample moving average
int avg_val_16 = 0; // Variable to store 16-sample moving average
int average_8(int x) {
    static int samples[8];
    static int i = 0;
    static int total = 0;

    /* Update the moving average */
    total += x - samples[i];
    samples[i] = x;

    /* Update the index */
    i = (i == 7 ? 0 : i + 1);

    return total >> 3; // Divide by 8
}

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

/* Private user code ---------------------------------------------------------*/
void displayHEX(uint32_t channel, uint32_t value);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();

  /* Start the ADC */
  HAL_ADC_Start(&hadc1);

//  void displayHEX(uint32_t value, uint8_t channel, float voltage) {
//      char output[50];
//      sprintf(output, "Channel: %d, Value: 0x%08lX, Voltage: %.2fV\r\n", channel, value, voltage);
//      HAL_UART_Transmit(&huart3, (uint8_t*)output, strlen(output), HAL_MAX_DELAY);
//  }

  void displayHEX(uint32_t value, uint8_t channel, float voltage) {
      char output[100];
      sprintf(output, "Channel: %u, Value: 0x%08lX, Value_in_d: %d, Voltage: %.2fV\r\n", channel, value,value, voltage);
//      sprintf(output, "Channel: %u, Value: %d, Voltage: %.2fV\r\n", channel, value, voltage);
      HAL_UART_Transmit(&huart3, (uint8_t *)output, strlen(output), HAL_MAX_DELAY);
  }

  void displayLEDs(uint32_t value) {
      // Clear all LEDs first
      HAL_GPIO_WritePin(LED_GPIO_PORT, LED3_PIN | LED4_PIN | LED5_PIN | LED6_PIN, GPIO_PIN_SET);

      // Determine LED pattern based on ADC value range
      if (value >= 0 && value <= 819) {
          HAL_GPIO_WritePin(LED_GPIO_PORT, LED3_PIN | LED4_PIN | LED5_PIN | LED6_PIN, GPIO_PIN_SET);
      } else if (value > 819 && value <= 1638) {
          HAL_GPIO_WritePin(LED_GPIO_PORT, LED3_PIN, GPIO_PIN_RESET);
      } else if (value > 1638 && value <= 2457) {
          HAL_GPIO_WritePin(LED_GPIO_PORT, LED3_PIN | LED4_PIN, GPIO_PIN_RESET);
      } else if (value > 2457 && value <= 3276) {
          HAL_GPIO_WritePin(LED_GPIO_PORT, LED3_PIN | LED4_PIN | LED5_PIN , GPIO_PIN_RESET);
      }else if (value > 3276 && value <= 4095) {
          HAL_GPIO_WritePin(LED_GPIO_PORT, LED3_PIN | LED4_PIN | LED5_PIN | LED6_PIN, GPIO_PIN_RESET);
      }
  }


  /* Infinite loop */
//  while (1)
//  {
//      /* Poll for ADC conversion */
//      if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
//          adc_val = HAL_ADC_GetValue(&hadc1);  // Read the ADC value
//
//          /* Apply averaging */
//                      int avg_val_8 = average_8(adc_val);
//                      int avg_val_16 = average_16(adc_val);
//
//                      /* Convert to voltage (assuming 3.3V reference and 12-bit resolution) */
//                      float voltage = (float)avg_val_16 * 3.3f / 4095.0f;
//
////          displayHEX(ADC_CHANNEL_0, adc_val);  // Display the value
//                      displayHEX(avg_val_16, 0, voltage);
//      }
//      HAL_Delay(400);  // 400 ms delay
//  }
//}
//  while (1) {
//          HAL_ADC_Start(&hadc1);
//          if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
//              adc_val = HAL_ADC_GetValue(&hadc1);
//
//              // Calculate average values to reduce swing effect
//              avg_val_8 = average_8(adc_val);
//              avg_val_16 = average_16(adc_val);
//
//              // Calculate voltage based on ADC value
//              float voltage = ((float)avg_val_16 / 4095.0) * 3.3;
//
//              // Display the HEX value, channel, and voltage
//              displayHEX(avg_val_16, 0, voltage);
//
//              // Display ADC value range using LEDs
//              displayLEDs(avg_val_16);
//          }
//          HAL_ADC_Stop(&hadc1);
//
//          HAL_Delay(400);
//      }
  while (1) {
      HAL_ADC_Start(&hadc1);
      if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
          adc_val = HAL_ADC_GetValue(&hadc1);

          // Adjust the ADC value handling for left alignment
          uint32_t adjusted_adc_val = adc_val >> 4;  // Shift right to get the 12 MSBs

          // Calculate average values to reduce swing effect
          avg_val_8 = average_8(adjusted_adc_val);
          avg_val_16 = average_16(adjusted_adc_val);

          // Calculate voltage based on ADC value
          float voltage = ((float)avg_val_16 / 4095.0) * 3.3;

          // Display the HEX value, channel, and voltage
          displayHEX(avg_val_16, 0, voltage);

          // Display ADC value range using LEDs
          displayLEDs(avg_val_16);
      }
      HAL_ADC_Stop(&hadc1);

      HAL_Delay(400);
  }

  }


/**
  * @brief Displays the ADC channel, value, and calculated voltage over UART.
  * @param channel: ADC channel
  * @param value: ADC value
  * @retval None
  */
//void displayHEX(uint32_t channel, uint32_t value) {
//    char output[50];  // Buffer to hold the output string
//    float voltage = ((float)value / 4095.0f) * 3.3f;  // Calculate voltage
//
//    // Format the output string
//    sprintf(output, "Channel: %u, Value: 0x%08X, Voltage: %.2fV\r\n", (unsigned int)channel, (unsigned int)value, voltage);
//
//
//    // Transmit the string over UART3
//    HAL_UART_Transmit(&huart3, (uint8_t*)output, strlen(output), HAL_MAX_DELAY);
//}



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




//PD3-6 -> LEDS
//PC0 -> Poten
