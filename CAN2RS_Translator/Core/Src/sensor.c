#include "sensor.h"
#include "uart.h"
#include "can_drv.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdbool.h>

#define PREAMBLE   0xAA
#define CAN_TX_ID  0x100
#define CAN_RX_ID  0x200
#define BUF_SZ     UART_RX_BUFSIZE

extern uint8_t           uart_rx_buf[BUF_SZ];
extern DMA_HandleTypeDef  hdma_usart1_rx;

// ASCII-паттерны
static const uint8_t cmd1[]    = "AREYOULIVE?";
static const uint8_t resp1[]   = "IAMALIVE!";
static const size_t  cmd1_len  = sizeof(cmd1)-1;
static const size_t  resp1_len = sizeof(resp1)-1;

static const uint8_t cmd2[]    = "ISSENSOROK?";
static const uint8_t resp2[]   = "SENSOR_WORKS!";
static const size_t  cmd2_len  = sizeof(cmd2)-1;
static const size_t  resp2_len = sizeof(resp2)-1;

// Разбор UART→DMA
static uint32_t rx_idx;
static uint8_t  frame_buf[256];
static size_t   frame_len;
static bool     in_frame;

static uint8_t  text_buf[128];
static size_t   text_len;
static bool     awaiting2;

void SENSOR_Init(void)
{
  rx_idx    = BUF_SZ - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
  in_frame  = false;
  frame_len = 0;
  text_len  = 0;
  awaiting2 = false;
}

void SENSOR_Process(void)
{
  uint32_t new_idx = BUF_SZ - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
  while (rx_idx != new_idx)
  {
    uint8_t b = uart_rx_buf[rx_idx++];
    if (rx_idx >= BUF_SZ) rx_idx = 0;

    if (!in_frame)
    {
      if (b == PREAMBLE)
      {
        in_frame     = true;
        frame_len    = 1;
        frame_buf[0] = PREAMBLE;
        text_len     = 0;
      }
      else
      {
        if (text_len < sizeof(text_buf))
          text_buf[text_len++] = b;

        // Scenario 1
        if (text_len >= cmd1_len &&
            memcmp(&text_buf[text_len - cmd1_len], cmd1, cmd1_len) == 0)
        {
          UART_Send((uint8_t*)resp1, resp1_len);
          text_len = 0;
        }
        // Scenario 2
        else if (text_len >= cmd2_len &&
                 memcmp(&text_buf[text_len - cmd2_len], cmd2, cmd2_len) == 0)
        {
          uint8_t req04[] = { PREAMBLE, 0x01,0x00, 0,0, 0x04,0,0 };
          UART_Send(req04, sizeof(req04));
          awaiting2 = true;
          text_len  = 0;
        }
      }
    }
    else
    {
      frame_buf[frame_len++] = b;
      if (frame_len >= 8)
      {
        uint16_t dlen = frame_buf[6] | (frame_buf[7] << 8);
        if (frame_len >= 8 + dlen + 2)
        {
          if (awaiting2)
          {
            UART_Send((uint8_t*)resp2, resp2_len);
            awaiting2 = false;
          }
          else
          {
            CAN_Send(CAN_TX_ID, frame_buf, frame_len>8?8:frame_len);
          }
          in_frame = false;
        }
      }
    }
  }

  // CAN→UART
  if (CAN_MessagePending())
  {
    uint16_t id; uint8_t buf8[8]; uint8_t len8;
    CAN_Receive(&id, buf8, &len8);
    if (id == CAN_RX_ID)
      UART_Send(buf8, len8);
  }
}
