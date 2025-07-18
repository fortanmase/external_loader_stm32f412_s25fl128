/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file         stm32f4xx_hal_msp.c
 * @brief        This file provides code for the MSP Initialization
 *               and de-Initialization codes.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
 * Initializes the Global MSP.
 */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
 * @brief QSPI MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hqspi: QSPI handle pointer
 * @retval None
 */
void HAL_QSPI_MspInit(QSPI_HandleTypeDef *hqspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (hqspi->Instance == QUADSPI)
  {
    /* USER CODE BEGIN QUADSPI_MspInit 0 */

    /* USER CODE END QUADSPI_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_QSPI_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**QUADSPI GPIO Configuration
    PA1     ------> QUADSPI_BK1_IO3
    PB1     ------> QUADSPI_CLK
    PC8     ------> QUADSPI_BK1_IO2
    PC9     ------> QUADSPI_BK1_IO0
    PC10     ------> QUADSPI_BK1_IO1
    PB6     ------> QUADSPI_BK1_NCS
    */
    GPIO_InitStruct.Pin = FLASH_BK1_IO3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_QSPI;
    HAL_GPIO_Init(FLASH_BK1_IO3_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = FLASH_CLK_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_QSPI;
    HAL_GPIO_Init(FLASH_CLK_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = FLASH_BK1_IO2_Pin | FLASH_BK1_IO0_Pin | FLASH_BK1_IO1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_QSPI;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = FLASH_BK1_NCS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(FLASH_BK1_NCS_GPIO_Port, &GPIO_InitStruct);

    /* USER CODE BEGIN QUADSPI_MspInit 1 */

    /* USER CODE END QUADSPI_MspInit 1 */
  }
}

/**
 * @brief QSPI MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hqspi: QSPI handle pointer
 * @retval None
 */
void HAL_QSPI_MspDeInit(QSPI_HandleTypeDef *hqspi)
{
  if (hqspi->Instance == QUADSPI)
  {
    /* USER CODE BEGIN QUADSPI_MspDeInit 0 */

    /* USER CODE END QUADSPI_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_QSPI_CLK_DISABLE();

    /**QUADSPI GPIO Configuration
    PA1     ------> QUADSPI_BK1_IO3
    PB1     ------> QUADSPI_CLK
    PC8     ------> QUADSPI_BK1_IO2
    PC9     ------> QUADSPI_BK1_IO0
    PC10     ------> QUADSPI_BK1_IO1
    PB6     ------> QUADSPI_BK1_NCS
    */
    HAL_GPIO_DeInit(FLASH_BK1_IO3_GPIO_Port, FLASH_BK1_IO3_Pin);

    HAL_GPIO_DeInit(GPIOB, FLASH_CLK_Pin | FLASH_BK1_NCS_Pin);

    HAL_GPIO_DeInit(GPIOC, FLASH_BK1_IO2_Pin | FLASH_BK1_IO0_Pin | FLASH_BK1_IO1_Pin);

    /* USER CODE BEGIN QUADSPI_MspDeInit 1 */

    /* USER CODE END QUADSPI_MspDeInit 1 */
  }
}

/**
 * @brief UART MSP Initialization
 * This function configures the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  // GPIO_InitTypeDef GPIO_InitStruct = {0};
  // if (huart->Instance == USART3)
  // {
  //   /* USER CODE BEGIN USART3_MspInit 0 */

  //   /* USER CODE END USART3_MspInit 0 */
  //   /* Peripheral clock enable */
  //   __HAL_RCC_USART3_CLK_ENABLE();

  //   __HAL_RCC_GPIOD_CLK_ENABLE();
  //   /**USART3 GPIO Configuration
  //   PD8     ------> USART3_TX
  //   PD9     ------> USART3_RX
  //   */
  //   GPIO_InitStruct.Pin = STLK_RX_Pin | STLK_TX_Pin;
  //   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  //   GPIO_InitStruct.Pull = GPIO_NOPULL;
  //   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  //   GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  //   HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  //   /* USER CODE BEGIN USART3_MspInit 1 */

  //   /* USER CODE END USART3_MspInit 1 */
  // }
}

/**
 * @brief UART MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  // if (huart->Instance == USART3)
  // {
  //   /* USER CODE BEGIN USART3_MspDeInit 0 */

  //   /* USER CODE END USART3_MspDeInit 0 */
  //   /* Peripheral clock disable */
  //   __HAL_RCC_USART3_CLK_DISABLE();

  //   /**USART3 GPIO Configuration
  //   PD8     ------> USART3_TX
  //   PD9     ------> USART3_RX
  //   */
  //   HAL_GPIO_DeInit(GPIOD, STLK_RX_Pin | STLK_TX_Pin);

  //   /* USER CODE BEGIN USART3_MspDeInit 1 */

  //   /* USER CODE END USART3_MspDeInit 1 */
  // }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
