#ifndef UART_H
#define UART_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"

// Тот же буфер, что и в uart.c
#define UART_RX_BUFSIZE 1024

// Экспортируемые из uart.c дескрипторы и буфер
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef  hdma_usart1_rx;
extern DMA_HandleTypeDef  hdma_usart1_tx;
extern uint8_t            uart_rx_buf[UART_RX_BUFSIZE];

/**
 * @brief Настроить UART1 (1 000 000 baud) + DMA circular RX + DMA TX + DE on PB6.
 */
void UART_Init(void);

/**
 * @brief Отправить len байт по RS-485 (DE автоматически в TXCplt сбросится).
 */
void UART_Send(uint8_t *buf, uint16_t len);

#endif // UART_H
