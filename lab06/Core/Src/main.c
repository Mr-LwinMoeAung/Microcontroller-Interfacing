/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_it.h"
#include <stdio.h>
#include <string.h>


/* Private variables ---------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart3;


/* Global variable */
uint32_t count = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
void displayNumber(uint32_t num);

/* Main function -------------------------------------------------------------*/
int main(void) {
    /* MCU Configuration--------------------------------------------------------*/
    HAL_Init();
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_USART3_UART_Init();

    /* Start TIM1 in interrupt mode */
    HAL_TIM_Base_Start_IT(&htim1);

    /* Infinite loop */
    while (1) {
        displayNumber(count);  // Display the current count value
        HAL_Delay(400);        // Delay 400 ms
    }
}

/* Display number via UART3 */
void displayNumber(uint32_t num) {
    char buffer[20];  // Buffer for number to string conversion
    sprintf(buffer, "%lu\r\n", num);
    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/* TIM1 Initialization Function */
static void MX_TIM1_Init(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 9999;  // Adjust prescaler according to your clock (assuming 100 MHz clock)
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 99;       // Adjust period to get 1 ms (assuming 100 MHz clock)
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
}

/* USART3 Initialization Function */
static void MX_USART3_UART_Init(void) {
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) {
        Error_Handler();
    }
}

/* GPIO Initialization Function */
static void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();  // Enable GPIO clock
    /* Additional GPIO initialization code if needed */
}

/* System Clock Configuration */
void SystemClock_Config(void) {
    /* Your system clock configuration code */
}

/* Error Handler */
void Error_Handler(void) {
    while(1) {
        // Stay in error handler if something goes wrong
    }
}
