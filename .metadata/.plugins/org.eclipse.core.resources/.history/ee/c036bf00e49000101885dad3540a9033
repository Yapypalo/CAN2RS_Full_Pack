/* ========================== sensor.c ==========================
   CAN↔RS485 транслятор с текстовым handshake:
     • «Работаешь?»         → «Работаю!»
     • «А датчик работает?» → запрос 0x04 → «И датчик работает»
   Бинарные фреймы (PREAMBLE…CRC) пересылаются в CAN.
   ============================================================= */

#include "sensor.h"
#include "uart.h"       // UART_Send, uart_rx_buf[], hdma_usart1_rx
#include "can_drv.h"
#include "debug.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdbool.h>

#define PREAMBLE           0xAA
#define SENSOR_CAN_TX_ID   0x100
#define SENSOR_CAN_RX_ID   0x200
#define SENSOR_TIMEOUT_MS  500   // мс

// Из uart.h:
extern uint8_t           uart_rx_buf[UART_RX_BUFSIZE];
extern DMA_HandleTypeDef hdma_usart1_rx;

// Индекс в uart_rx_buf, до которого мы уже прочитали
static uint32_t uart_last_idx;

// Тайминг сценариев
static uint32_t last_req_tick;
static bool     awaiting_sensor;
static bool     awaiting_handshake2;

// Временные буферы для разбора
static char    ac_buf[32];
static int     ac_len;
static uint8_t bin_buf[1040];
static int     bin_len;
static bool    in_bin;

/**
 * @brief  Проверяет, завершилась ли в ac_buf фраза pat
 */
static bool endswith(const char *pat, int pat_len)
{
  if (ac_len < pat_len) return false;
  return memcmp(&ac_buf[ac_len - pat_len], pat, pat_len) == 0;
}

void SENSOR_Init(void)
{
  // аппаратный ресет зонда
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

  // начальные состояния
  uart_last_idx       = UART_RX_BUFSIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
  last_req_tick       = HAL_GetTick();
  awaiting_sensor     = false;
  awaiting_handshake2 = false;
  ac_len              = 0;
  bin_len             = 0;
  in_bin              = false;
}

void SENSOR_Process(void)
{
  // 1) Прочитать все новые байты из UART1/DMA circular
  uint32_t cur_idx = UART_RX_BUFSIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);

  while (uart_last_idx != cur_idx)
  {
    uint8_t b = uart_rx_buf[uart_last_idx++];
    if (uart_last_idx >= UART_RX_BUFSIZE) uart_last_idx = 0;

    // если находим PREAMBLE — переходим в разбор бинарного фрейма
    if (!in_bin && b == PREAMBLE)
    {
      in_bin    = true;
      bin_len   = 1;
      bin_buf[0]= PREAMBLE;
      // сбрасываем ASCII-буфер
      ac_len = 0;
      continue;
    }

    // 1.1) разбор бинарника
    if (in_bin)
    {
      bin_buf[bin_len++] = b;
      if (bin_len == 8)
      {
        // заголовок прочитан — в полях 6–7 лежит длина payload
      }
      // ожидаем header(8) + data + CRC(2)
      uint16_t dlen = bin_buf[6] | (bin_buf[7] << 8);
      if (bin_len >= 8 + dlen + 2)
      {
        // got full frame
        if (awaiting_sensor && awaiting_handshake2)
        {
          // ответ на «А датчик работает?»
          UART_Send((uint8_t*)"И датчик работает", 18);
          awaiting_sensor     = false;
          awaiting_handshake2 = false;
        }
        else
        {
          // Forward first 8 bytes to CAN
          CAN_Send(SENSOR_CAN_TX_ID,
                   bin_buf,
                   (bin_len > 8 ? 8 : bin_len));
        }
        in_bin  = false;
        bin_len = 0;
      }
      continue;
    }

    // 1.2) разбор ASCII-символа
    if (b >= 32 && b <= 126 && ac_len < (int)sizeof(ac_buf))
    {
      ac_buf[ac_len++] = (char)b;

      // Сценарий 1: «Работаешь?»
      if (endswith("Работаешь?", 10))
      {
        UART_Send((uint8_t*)"Работаю!", 8);
        ac_len = 0;
        continue;
      }

      // Сценарий 2: «А датчик работает?»
      if (endswith("А датчик работает?", 19))
      {
        // соберём фрейм запроса 0x04 без payload
        uint8_t req04[] = {
          PREAMBLE,
          0x01,    // dest
          0x00,    // src
          0x00,0x00,
          0x04,    // команда
          0x00,0x00 // length=0
        };
        UART_Send(req04, sizeof(req04));
        awaiting_sensor     = true;
        awaiting_handshake2 = true;
        last_req_tick       = HAL_GetTick();
        ac_len = 0;
        continue;
      }
    }

    // else: игнорируем прочие байты
  }

  // 2) CAN → UART
  if (CAN_MessagePending())
  {
    uint16_t id; uint8_t buf[8], len;
    CAN_Receive(&id, buf, &len);
    if (id == SENSOR_CAN_RX_ID)
    {
      UART_Send(buf, len);
      awaiting_sensor = true;
      last_req_tick   = HAL_GetTick();
    }
  }

  // 3) таймаут ответа zonda
  if (awaiting_sensor &&
      (HAL_GetTick() - last_req_tick > SENSOR_TIMEOUT_MS))
  {
    DEBUG_Log("Sensor timeout, resetting...\r\n");
    SENSOR_Init();
  }
}
