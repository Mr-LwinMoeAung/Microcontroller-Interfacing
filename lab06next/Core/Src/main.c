#include "stm32f4xx_hal.h"
#include "main.h"
#include "tim.h"
#include "usart.h"

// Private variables
static uint8_t minutes = 0;
static uint8_t seconds = 0;

// Private function prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

// Main program
int main(void) {
    // Initialize the HAL Library
    HAL_Init();

    // Configure the system clock
    SystemClock_Config();

    // Initialize all configured peripherals
    MX_GPIO_Init();
    MX_USART3_UART_Init();
    MX_TIM2_Init();

    // Main loop
    while (1) {
        // Main loop does nothing; everything is handled in TIM2 interrupt
    }
}

// TIM2 interrupt handler
void TIM2_IRQHandler(void) {
    // Check if update interrupt flag is set
    if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET) {
        // Clear the update interrupt flag
        __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);

        // Update time
        seconds++;
        if (seconds >= 60) {
            seconds = 0;
            minutes++;
        }

        // Format time string
        char time_str[6];
        snprintf(time_str, sizeof(time_str), "%02d:%02d\r", minutes, seconds);

        // Send time via UART3
        HAL_UART_Transmit(&huart3, (uint8_t *)time_str, strlen(time_str), HAL_MAX_DELAY);
    }
}

// System Clock Configuration
void SystemClock_Config(void) {
    // Configure the system clock to use the HSI oscillator
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // Configure the HSI oscillator
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    // Configure the system clock
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_ACR_LATENCY_0);
}

// GPIO initialization function
static void MX_GPIO_Init(void) {
    // Initialize GPIOs if needed (empty for this example)
}

// TIM2 initialization function
void MX_TIM2_Init(void) {
    __HAL_RCC_TIM2_CLK_ENABLE(); // Enable TIM2 clock

    TIM_HandleTypeDef htim2;
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 83999; // Adjusted prescaler
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 999; // Adjusted period
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim2);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

    HAL_TIM_Base_Start_IT(&htim2); // Start TIM2 with interrupt
}

// USART3 initialization function
void MX_USART3_UART_Init(void) {
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure USART3 TX (PB10) and RX (PB11)
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    UART_HandleTypeDef huart3;
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart3);
}
