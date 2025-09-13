/* ========================== sensor.h ========================== */
#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Инициализация «звездного датчика» по RS-485.
 *        Подготовка UART1, сброс зонда.
 */
void SENSOR_Init(void);

/**
 * @brief Основной процессор трансляции:
 *        - читать CAN, переслать на UART
 *        - читать UART, переслать в CAN
 *        - следить за тайм-аута­ми, сброс зонда при залипании
 */
void SENSOR_Process(void);

#endif // SENSOR_H
