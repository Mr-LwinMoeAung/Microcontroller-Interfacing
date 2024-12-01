/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "rng.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <string.h>
#include "ILI9341_Touchscreen.h"

#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"

#include "snow_tiger.h"
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
/* Private variables ---------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart3;

float h=99.0, t=99.0;
uint8_t step = 0;
HAL_StatusTypeDef status;

uint16_t red_intensity = 0, green_intensity = 0, blue_intensity = 0;  // RGB brightness values (0-255)
const uint8_t max_intensity = 255; // Set a maximum intensity threshold
const uint8_t increment_value = 50; // How much to increase on each touch
uint8_t redTouchCount = 0;
uint8_t greenTouchCount = 0;
uint8_t blueTouchCount = 0;
uint8_t totalTouchCount = 0;
uint16_t rgb_color = 0;  // Mixed color
uint8_t touch_x, touch_y;

//uint16_t total_length_r = 0;
//uint16_t filled_length_r = 0;
int percentage_r = 0;
int percentage_g = 0;
int percentage_b = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
uint16_t CRC16_2(uint8_t *, uint8_t);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//uint16_t brightness_to_color(uint8_t brightness) {
//    return brightness * 255 / 100;
//}

// Function to mix colors and display
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  char str[50];
  uint8_t cmdBuffer[3];
  uint8_t dataBuffer[8];
  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_SPI5_Init();
  MX_TIM1_Init();
  MX_RNG_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  ILI9341_Init();//initial driver setup to drive ili9341

  sprintf(str, "\n\rAM2320 I2C DEMO Starting .. .\n\r");
  HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 200);

  cmdBuffer[0] = 0x03;  // Command to read register
  cmdBuffer[1] = 0x00;  // Starting address
  cmdBuffer[2] = 0x04;  // Read 4 bytes (temperature + humidity)
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //----------------------------------------------------------TOUCHSCREEN EXAMPLE
	  		ILI9341_Fill_Screen(WHITE);
	  		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);

	  		while(1){
//	  			ILI9341_Draw_Text(str, "Temperature = %4.1f\tHumidity = %4.1f\n\r", t, h);
	  			char display_buff[30];

	  			// Print Temperature at top-left corner

//	  			ILI9341_Set_Rotation(SCREEN_VERTICAL_1);
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


	  			  ILI9341_Draw_Filled_Circle(40, 65, 20, 0xF800);   // Red circle
	  			  ILI9341_Draw_Filled_Rectangle_Coord(80, 55, 250, 80, 0xFC10);  // Red scrollbar

	  			  ILI9341_Draw_Filled_Circle(40, 125, 20, 0x07E0);  // Green circle
	  			  ILI9341_Draw_Filled_Rectangle_Coord(80, 115, 250, 140, 0x07F0);  // Green scrollbar

	  			  ILI9341_Draw_Filled_Circle(40, 185, 20, 0x001F);  // Blue circle
	  			  ILI9341_Draw_Filled_Rectangle_Coord(80, 175, 250, 200, 0x041F);  // Blue scrollbar

	  			  sprintf(display_buff, "%.1fC  ", t);
	  			  ILI9341_Draw_Text(display_buff, 15, 15, BLACK, 2, WHITE); // Use (0, 0) for top-left

//	  			  rgb_color = ((scaled_red & 0xF8) << 8) | ((scaled_green & 0xFC) << 3) | (scaled_blue >> 3);  // RGB565 format
	  			  ILI9341_Draw_Filled_Circle(100, 20, 15, BLACK);  // Display mixed color

	  			  sprintf(display_buff, "  %.1fRH", h);
	  			  ILI9341_Draw_Text(display_buff, 120, 15, BLACK, 2, WHITE); // Adjusted position

	  			  HAL_Delay(500);


	  			  while(1)
	  				  		{
	  				  			HAL_Delay(20);

	  				  			if(TP_Touchpad_Pressed())
	  				          {

	  				  					uint16_t x_pos = 0;
	  				  					uint16_t y_pos = 0;


	  				  					HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_SET);

	  				  					uint16_t position_array[2];

	  				  					if(TP_Read_Coordinates(position_array) == TOUCHPAD_DATA_OK)
	  				  					{
	  				  					x_pos = position_array[0];
	  				  					y_pos = position_array[1];
//	  				  					ILI9341_Draw_Filled_Circle(x_pos, y_pos, 2, BLACK);

											ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
//											char counter_buff[30];
//											sprintf(counter_buff, "POS X: %.3d", x_pos);
//											ILI9341_Draw_Text(counter_buff, 10, 80, BLACK, 2, WHITE);
//											sprintf(counter_buff, "POS Y: %.3d", y_pos);
//											ILI9341_Draw_Text(counter_buff, 10, 120, BLACK, 2, WHITE);

											if (x_pos >= 164 && x_pos <= 187 && y_pos >= 83 && y_pos <= 250){
//												ILI9341_Draw_Filled_Circle(100, 20, 15, 0xF800);
												red_intensity = (red_intensity + increment_value > max_intensity) ? max_intensity : red_intensity + increment_value;
												redTouchCount++;

												ILI9341_Draw_Filled_Rectangle_Coord(80, 55, y_pos, 80, 0xC0C0);  // Silver scrollbar
												ILI9341_Draw_Filled_Rectangle_Coord(y_pos, 55, 250, 80, 0xFC10);  // Red scrollbar

//												ILI9341_Draw_Filled_Rectangle_Coord(80, 55, 250, 80, 0xFC10);  // Red scrollbar

												uint16_t total_length_r = 250 - 80;
												uint16_t filled_length_r = y_pos - 80;
												if (filled_length_r < 0) {
												    filled_length_r = 0;
												}
												percentage_r = ((float)filled_length_r / (float)total_length_r) * 100;
												char redBuffer[30];
												sprintf(redBuffer, "%d%%", percentage_r);
												ILI9341_Draw_Text(redBuffer, 230, 65, BLACK, 2, WHITE);
											}
											if (x_pos >= 100 && x_pos <= 125 && y_pos >= 83 && y_pos <= 250){
//												ILI9341_Draw_Filled_Circle(100, 20, 15, 0x07E0);
												green_intensity = (green_intensity + increment_value > max_intensity) ? max_intensity : green_intensity + increment_value;
												greenTouchCount++;
												ILI9341_Draw_Filled_Rectangle_Coord(80, 115, y_pos, 140, 0xC0C0);  // Silver scrollbar
												ILI9341_Draw_Filled_Rectangle_Coord(y_pos, 115, 250, 140, 0x07F0);  // Green scrollbar

												uint16_t total_length_g = 250 - 80;
												uint16_t filled_length_g = y_pos - 80;
												if (filled_length_g < 0) {
												    filled_length_g = 0;
												}
												percentage_g = ((float)filled_length_g / (float)total_length_g) * 100;
												char greenBuffer[30];
												sprintf(greenBuffer, "%d%%", percentage_g);
												ILI9341_Draw_Text(greenBuffer, 230, 125, BLACK, 2, WHITE);
											}
											if (x_pos >= 40 && x_pos <= 63 && y_pos >= 83 && y_pos <= 250){
//												ILI9341_Draw_Filled_Circle(100, 20, 15, 0x001F);
												blue_intensity = (blue_intensity + increment_value > max_intensity) ? max_intensity : blue_intensity + increment_value;
												blueTouchCount++;
												ILI9341_Draw_Filled_Rectangle_Coord(80, 175, y_pos, 200, 0xC0C0);  // Silver scrollbar
												ILI9341_Draw_Filled_Rectangle_Coord(y_pos, 175, 250, 200, 0x041F);  // Blue scrollbar

												uint16_t total_length_b = 250 - 80;
												uint16_t filled_length_b = y_pos - 80;
												if (filled_length_b < 0) {
												    filled_length_b = 0;
												}
												percentage_b = ((float)filled_length_b / (float)total_length_b) * 100;
												char blueBuffer[30];
												sprintf(blueBuffer, "%d%%", percentage_b);
												ILI9341_Draw_Text(blueBuffer, 230, 185, BLACK, 2, WHITE);
											}

											uint16_t red_intensity   = (percentage_r * 31) / 100;  // Red uses 5 bits (0-31)
											uint16_t green_intensity = (percentage_g * 63) / 100;  // Green uses 6 bits (0-63)
											uint16_t blue_intensity  = (percentage_b * 31) / 100;  // Blue uses 5 bits (0-31)

											uint16_t mix_color = (red_intensity << 11) | (green_intensity << 5) | blue_intensity;

//											uint16_t color = (red_intensity * 31 / max_intensity) << 11 |
//											                             (green_intensity * 63 / max_intensity) << 5 |
//											                             (blue_intensity * 31 / max_intensity);

											            // Draw the filled circle with the new mixed color
											ILI9341_Draw_Filled_Circle(100, 20, 15, mix_color);
											ILI9341_Set_Rotation(SCREEN_VERTICAL_1);

	  				  					}

//	  				  					ILI9341_Draw_Pixel(x_pos, y_pos, BLACK);
	  				          }
	  				  			else
	  				  			{
	  				  				HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);
	  				  			}

	  				  		}
	  		}
  }
  /* USER CODE END 3 */
}



void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

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

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
