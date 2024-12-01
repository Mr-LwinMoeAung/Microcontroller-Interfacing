#include "main.h"
#include "stm32f7xx_hal.h"
#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart3;

float h=99.0, t=99.0;
uint8_t step = 0;
HAL_StatusTypeDef status;

// Function declarations
void MPU_Config(void);
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_I2C1_Init(void);
void MX_USART3_UART_Init(void);
uint16_t CRC16_2(uint8_t *, uint8_t);

int main(void)
{
    char str[50];
    uint8_t cmdBuffer[3];
    uint8_t dataBuffer[8];

    HAL_Init();
    MPU_Config();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART3_UART_Init();

    sprintf(str, "\n\rAM2320 I2C DEMO Starting .. .\n\r");
    HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 200);

    cmdBuffer[0] = 0x03;  // Command to read register
    cmdBuffer[1] = 0x00;  // Starting address
    cmdBuffer[2] = 0x04;  // Read 4 bytes (temperature + humidity)

    while (1)
    {
        sprintf(str, "Temperature = %4.1f\tHumidity = %4.1f\n\r", t, h);
        while (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC) == RESET) {}
        HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 200);

        HAL_Delay(5000);
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);

        // Wake up the sensor
        HAL_I2C_Master_Transmit(&hi2c1, 0x5c << 1, NULL, 0, 200);
        HAL_Delay(800);  // Allow time for sensor to wake up

        // Send read command to sensor
        status = HAL_I2C_Master_Transmit(&hi2c1, 0x5c << 1, cmdBuffer, 3, 200);
        if (status != HAL_OK)
        {
            sprintf(str, "I2C Transmit Error, Status: %d\n\r", status);
            HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 200);
            continue;
        }

        HAL_Delay(2);  // Wait for sensor response

        // Receive data from sensor
        status = HAL_I2C_Master_Receive(&hi2c1, 0x5c << 1, dataBuffer, 8, 200);
        if (status != HAL_OK)
        {
            sprintf(str, "I2C Receive Error\n\r");
            HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 200);
            continue;  // Skip if there's a receive error
        }

        // Validate CRC
        uint16_t Rcrc = dataBuffer[7] << 8 | dataBuffer[6];
        if (Rcrc == CRC16_2(dataBuffer, 6))
        {
            uint16_t temperature = ((dataBuffer[4] & 0x7F) << 8) + dataBuffer[5];
            t = temperature / 10.0;
            t = (((dataBuffer[4] & 0x80) >> 7) == 1) ? (t * (-1)) : t;

            uint16_t humidity = (dataBuffer[2] << 8) + dataBuffer[3];
            h = humidity / 10.0;
        }
        else
        {
            sprintf(str, "CRC mismatch\n\r");
            HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 200);
        }
    }
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
/* USER CODE BEGIN 4 */
uint16_t CRC16_2 (uint8_t *ptr, uint8_t length){
	uint16_t crc = 0xFFFF;
	uint8_t s = 0x00;
	while (length -- ) {
		crc ^= *ptr++;
		for(s = 0; s < 8; s++) {
			if((crc & 0x01) != 0) {
				crc >>= 1;
				crc ^= 0xA001;
			}else crc >>= 1;
		}
	}
	return crc;
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
