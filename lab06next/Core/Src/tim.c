#include "stm32f4xx_hal.h"
#include "tim.h"

// TIM1 handle declaration
TIM_HandleTypeDef htim1;

// TIM1 initialization function
void MX_TIM1_Init(void) {
    // Enable TIM1 clock
    __HAL_RCC_TIM1_CLK_ENABLE();

    // Configure TIM1
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 8399; // Adjust as needed
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 9999; // Adjust period as needed
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    // Initialize TIM1
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
        // Initialization Error
        Error_Handler();
    }

    // Configure the TIM1 clock source
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
        // Configuration Error
        Error_Handler();
    }

    // Configure TIM1 Master Output Trigger
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
        // Configuration Error
        Error_Handler();
    }

    // Start TIM1 in interrupt mode
    if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK) {
        // Starting Error
        Error_Handler();
    }
}

// TIM1 interrupt handler
void TIM1_UP_TIM10_IRQHandler(void) {
    // Check if update interrupt flag is set
    if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) != RESET) {
        // Clear the update interrupt flag
        __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);

        // Handle TIM1 interrupt tasks
        // Code for TIM1 interrupt handling
    }
}

// Error handler function
void Error_Handler(void) {
    // User can add their own implementation to report the HAL error return state
    while (1) {
        // Stay in an infinite loop
    }
}
