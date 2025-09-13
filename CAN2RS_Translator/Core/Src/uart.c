/* ========================== uart.c ==========================
   UART1 / RS-485 driver for STM32F103:
     • Baudrate 1 000 000 baud
     • RX via DMA in circular mode into uart_rx_buf[]
     • TX via DMA with DE (PB6) control
   ========================================================= */

#include "uart.h"
#include "stm32f1xx_hal.h"

// Размер кольцевого буфера для приёма UART1
#define UART_RX_BUFSIZE 1024

// Буфер приёма (DMA circular)
uint8_t uart_rx_buf[UART_RX_BUFSIZE];

// Глобальные дескрипторы UART1 и его DMA-каналов
UART_HandleTypeDef huart1;
DMA_HandleTypeDef  hdma_usart1_rx;
DMA_HandleTypeDef  hdma_usart1_tx;

/**
 * @brief Инициализация UART1 и DMA для RS-485.
 *        DE-контроль на PB6: при передаче DE=1, по окончании — DE=0.
 */
void UART_Init(void)
{
  // Тактирование USART1 и DMA1
  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  // Настройка UART1
  huart1.Instance          = USART1;
  huart1.Init.BaudRate     = 1000000;
  huart1.Init.WordLength   = UART_WORDLENGTH_8B;
  huart1.Init.StopBits     = UART_STOPBITS_1;
  huart1.Init.Parity       = UART_PARITY_NONE;
  huart1.Init.Mode         = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

  // Настройка DMA RX (circular)
  hdma_usart1_rx.Instance                 = DMA1_Channel5;
  hdma_usart1_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_usart1_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_usart1_rx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart1_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_usart1_rx.Init.Mode                = DMA_CIRCULAR;
  hdma_usart1_rx.Init.Priority            = DMA_PRIORITY_HIGH;
  HAL_DMA_Init(&hdma_usart1_rx);
  __HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);

  // Настройка DMA TX (normal)
  hdma_usart1_tx.Instance                 = DMA1_Channel4;
  hdma_usart1_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hdma_usart1_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_usart1_tx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart1_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_usart1_tx.Init.Mode                = DMA_NORMAL;
  hdma_usart1_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;
  HAL_DMA_Init(&hdma_usart1_tx);
  __HAL_LINKDMA(&huart1, hdmatx, hdma_usart1_tx);

  // Запуск приёма по DMA в circular-режиме
  HAL_UART_Receive_DMA(&huart1, uart_rx_buf, UART_RX_BUFSIZE);
}

/**
 * @brief  Передача данных по UART1/RS-485.
 * @param  buf  Указатель на буфер с данными.
 * @param  len  Длина данных в байтах.
 */
void UART_Send(uint8_t *buf, uint16_t len)
{
  // DE = 1: включаем драйвер RS-485
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

  // Передача по DMA
  HAL_UART_Transmit_DMA(&huart1, buf, len);
}

/**
 * @brief  Колбэк HAL по окончании DMA-транзакции TX.
 *         Вызывается в прерывании, когда блок данных отправлен.
 * @param  huart  Хендл UART, где завершилась передача.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    // DE = 0: выключаем драйвер RS-485
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
  }
}
