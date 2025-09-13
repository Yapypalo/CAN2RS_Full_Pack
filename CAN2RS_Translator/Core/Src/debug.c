#include "debug.h"
#include "uart.h"      // для UART_Send()
#include "stm32f1xx_hal.h"
#include <stdarg.h>
#include <stdio.h>

void DEBUG_Log(const char *fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len > 0) {
        // Шлём по RS-485, пин DE=1/0 управляется внутри UART_Send()
        UART_Send((uint8_t*)buf, (uint16_t)len);
        // Даем чуть-чуть времени для передачи, можно убрать при уверенности в DMA
        HAL_Delay(2);
    }
}

void DEBUG_SelfTest(void)
{
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_tick < 1000) return;
    last_tick = now;

    // Простейший «heartbeat»
    DEBUG_Log("DBG: heartbeat %lu\r\n", now);

    // Можно добавить проверку UART1 (DMA/Flags) и CAN:
    // если что-то не так — сброситься:
#if 1
    // Пример проверки UART1 error-состояния
    // (HAL_UART_STATE_ERROR не всегда актуально, но можно расширить)
    extern UART_HandleTypeDef huart1;
    if (huart1.gState == HAL_UART_STATE_ERROR) {
        DEBUG_Log("DBG: UART1 error -> reset\r\n");
        HAL_Delay(5);
        NVIC_SystemReset();
    }
#endif
}
