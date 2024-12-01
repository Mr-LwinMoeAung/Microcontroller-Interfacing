#include "stm32f4xx_hal.h"
#include "usart.h"

// UART3 handle declaration
UART_HandleTypeDef huart3;

// UART3 initialization function
void MX_USART3_UART_Init(void) {
    // Enable USART3 clock
    __HAL_RCC_USART3_CLK_ENABLE();
    // Enable GPIOB clock (assuming USART3 uses GPIOB for TX and RX)
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Configure GPIO pins for USART3
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure USART3 TX pin (PB10)
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Configure USART3 RX pin (PB11)
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Configure UART3
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) {
        // Initialization Error
        Error_Handler();
    }
}

// Function to transmit data via UART3
HAL_StatusTypeDef UART3_Transmit(uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    return HAL_UART_Transmit(&huart3, pData, Size, Timeout);
}

// Function to receive data via UART3 (if needed)
HAL_StatusTypeDef UART3_Receive(uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    return HAL_UART_Receive(&huart3, pData, Size, Timeout);
}

// Error handler function
void Error_Handler(void) {
    // User can add their own implementation to report the HAL error return state
    while (1) {
        // Stay in an infinite loop
    }
}
