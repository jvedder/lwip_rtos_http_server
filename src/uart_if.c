/**
  ******************************************************************************
  * @file           : uart_if.c
  * @brief          : UART API and HAL Interface
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "uart_if.h"
#include "lwip/api.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define UART_TX_THREAD_PRIO    ( osPriorityNormal )

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

/* Private variables ---------------------------------------------------------*/
static volatile int dma_complete = 0;
static int counter = 0;
static char uart_buffer[40];

/* Private function prototypes -----------------------------------------------*/
static void MX_NVIC_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_GPIO_Init(void);
static void uart_tx_thread(void *arg);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
	  while(1);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GRN_LED_Pin|RED_LED_Pin|BLUE_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GRN_LED_Pin RED_LED_Pin BLUE_LED_Pin */
  GPIO_InitStruct.Pin = GRN_LED_Pin|RED_LED_Pin|BLUE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/**
  * @brief  HAL callback when the UART DMA transmit is complete.  This sets a flag
  * to the main loop that additional data can be queued to the UART.
  *
  * Note: This overrides a weak function in the HAL with the same signature
  * @param huart: pointer on UART data structure (not used here)
  * @retval None
  */
void HAL_UART_TxCpltCallback (UART_HandleTypeDef *huart)
{
	dma_complete = 1;
}

/**
  * @brief  UART transmit thread.  Infinite loop to continuously transmit sequential numbers on the UART.
  * @param arg: pointer on argument (not used here)
  * @retval None
  */
static void uart_tx_thread(void *arg)
{
  dma_complete = 0;

  HAL_UART_Transmit_DMA( &huart3, (uint8_t *) &"Booted.\r\n", 9 );

  while(1)
  {
	  vTaskDelay(10);

	  if (dma_complete == 1)
	  {
		  dma_complete = 0;
		  counter++;
		  sprintf( uart_buffer, "rtos: %d\r\n", counter );
		  HAL_UART_Transmit_DMA( &huart3, (uint8_t*) uart_buffer, strlen(uart_buffer) );
	  }
  }
}

/* Public functions ---------------------------------------------------------*/

/**
  * @brief  Initializes and starts the UART interface.
  *
  * @retval None
  */
void uart_if_init(void)
{
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOB, GRN_LED_Pin, GPIO_PIN_SET);

  dma_complete = 1;

  sys_thread_new("UART_TX", uart_tx_thread, NULL, DEFAULT_THREAD_STACKSIZE, UART_TX_THREAD_PRIO);

}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
