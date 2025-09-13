#ifndef DEBUG_H
#define DEBUG_H

#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>

/**
 * @brief Инициализация debug-модуля.
 *        UART1 уже проинициализирован в UART_Init(), здесь ничего не делаем.
 */
static inline void DEBUG_Init(void) {}

/**
 * @brief Печать диагностического текста по UART1/RS-485.
 */
void DEBUG_Log(const char *fmt, ...);

/**
 * @brief Периодическая самодиагностика.
 */
void DEBUG_SelfTest(void);

#endif // DEBUG_H
