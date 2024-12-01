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
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"
#include <stdio.h>   // For snprintf
#include <string.h>  // For strlen


/* Private variables ---------------------------------------------------------*/
//ADC_HandleTypeDef hadc1;
//DMA_HandleTypeDef hdma_adc1;
//UART_HandleTypeDef huart3;

uint32_t adc_buffer[7]; // Buffer to hold ADC values from 7 channels

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_ADC1_Init(void);
void MX_USART3_UART_Init(void);

/* Private user code ---------------------------------------------------------*/
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET); // Turn on LED on PF0
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET); // Turn off LED on PF0
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);   // Turn on LED on PF1
        char msg[64];
        snprintf(msg, sizeof(msg), "ADC Values: %u %u %u %u %u %u %u\r\n",
        		(unsigned int)adc_buffer[0], (unsigned int)adc_buffer[1], (unsigned int)adc_buffer[2],
        		(unsigned int)adc_buffer[3], (unsigned int)adc_buffer[4],
        		(unsigned int)adc_buffer[5], (unsigned int)adc_buffer[6]);
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET); // Turn off LED on PF1
    }
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_USART3_UART_Init();

    /* Start ADC conversion with DMA */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 7);

    /* Infinite loop */
    while (1)
    {
        // Main loop can be empty as we're handling everything in the callbacks
    }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
//void MX_ADC1_Init(void)
//{
//    ADC_ChannelConfTypeDef sConfig = {0};
//
//    /* Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) */
//    hadc1.Instance = ADC1;
//    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
//    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
//    hadc1.Init.ScanConvMode = ENABLE;
//    hadc1.Init.ContinuousConvMode = DISABLE;
//    hadc1.Init.DiscontinuousConvMode = DISABLE;
//    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
//    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
//    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
//    hadc1.Init.NbrOfConversion = 7;
//    hadc1.Init.DMAContinuousRequests = ENABLE;
//    hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
//    HAL_ADC_Init(&hadc1);
//
//    /* Configure for the selected ADC regular channels to be converted. */
//    sConfig.Channel = ADC_CHANNEL_10; // PC0
//    sConfig.Rank = 1;
//    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
//    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
//
//    sConfig.Channel = ADC_CHANNEL_12; // PC2
//    sConfig.Rank = 2;
//    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
//
//    sConfig.Channel = ADC_CHANNEL_13; // PC3
//    sConfig.Rank = 3;
//    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
//
//    sConfig.Channel = ADC_CHANNEL_5; // PA5
//    sConfig.Rank = 4;
//    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
//
//    sConfig.Channel = ADC_CHANNEL_6; // PA6
//    sConfig.Rank = 5;
//    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
//
//    sConfig.Channel = ADC_CHANNEL_7; // PA7
//    sConfig.Rank = 6;
//    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
//
//    sConfig.Channel = ADC_CHANNEL_4; // PC4
//    sConfig.Rank = 7;
//    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
//}
//
///**
//  * @brief USART3 Initialization Function
//  * @param None
//  * @retval None
//  */
//void MX_USART3_UART_Init(void)
//{
//    huart3.Instance = USART3;
//    huart3.Init.BaudRate = 115200;
//    huart3.Init.WordLength = UART_WORDLENGTH_8B;
//    huart3.Init.StopBits = UART_STOPBITS_1;
//    huart3.Init.Parity = UART_PARITY_NONE;
//    huart3.Init.Mode = UART_MODE_TX_RX;
//    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
//    HAL_UART_Init(&huart3);
//}
//
///**
//  * Enable DMA controller clock
//  */
//void MX_DMA_Init(void)
//{
//    __HAL_RCC_DMA2_CLK_ENABLE();
//
//    /* ADC1 DMA Init */
//    /* ADC1 Init */
//    hdma_adc1.Instance = DMA2_Stream0;
//    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
//    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
//    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
//    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
//    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
//    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
//    hdma_adc1.Init.Mode = DMA_CIRCULAR;
//    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
//    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//    HAL_DMA_Init(&hdma_adc1);
//
//    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
//
//    /* DMA interrupt init */
//    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
//    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
//}
//
///**
//  * @brief GPIO Initialization Function
//  * @param None
//  * @retval None
//  */
//void MX_GPIO_Init(void)
//{
//    GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//    /* GPIO Ports Clock Enable */
//    __HAL_RCC_GPIOF_CLK_ENABLE();
//
//    /*Configure GPIO pin Output Level */
//    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
//
//    /*Configure GPIO pins : PF0 PF1 */
//    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
//}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    // System clock configuration here
}

void Error_Handler(void)
{
    // You can add your error handling code here
    while(1)
    {
    }
}
