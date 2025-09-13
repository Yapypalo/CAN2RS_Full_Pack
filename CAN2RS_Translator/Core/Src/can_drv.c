/* ========================== can_drv.c ========================== */
#include "can_drv.h"

CAN_HandleTypeDef hcan1;

static CAN_RxHeaderTypeDef RxHeader;
static uint8_t rxBuf[8];
static volatile bool newMsg = false;

void CAN_Init(void)
{
  __HAL_RCC_CAN1_CLK_ENABLE();
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;              // 36MHz / 6 = 6MHz -> 1Mbps (CAN)
  hcan1.Init.Mode      = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1     = CAN_BS1_8TQ;
  hcan1.Init.TimeSeg2     = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff      = ENABLE;
  hcan1.Init.AutoWakeUp      = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked  = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  HAL_CAN_Init(&hcan1);

  // Фильтр: принять всё
  CAN_FilterTypeDef filt = {0};
  filt.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filt.FilterIdHigh   = 0;
  filt.FilterIdLow    = 0;
  filt.FilterMaskIdHigh= 0;
  filt.FilterMaskIdLow = 0;
  filt.FilterBank     = 0;
  filt.FilterMode     = CAN_FILTERMODE_IDMASK;
  filt.FilterScale    = CAN_FILTERSCALE_32BIT;
  filt.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(&hcan1, &filt);

  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

HAL_StatusTypeDef CAN_Send(uint16_t id, uint8_t *buf, uint8_t len)
{
  CAN_TxHeaderTypeDef txh = {0};
  uint32_t mailbox;
  txh.StdId = id & 0x7FF;
  txh.IDE   = CAN_ID_STD;
  txh.RTR   = CAN_RTR_DATA;
  txh.DLC   = len;
  return HAL_CAN_AddTxMessage(&hcan1, &txh, buf, &mailbox);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1) {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, rxBuf);
    newMsg = true;
  }
}

bool CAN_MessagePending(void)
{
  return newMsg;
}

void CAN_Receive(uint16_t *out_id, uint8_t *buf, uint8_t *out_len)
{
  newMsg = false;
  *out_id  = RxHeader.StdId;
  *out_len = RxHeader.DLC;
  memcpy(buf, rxBuf, RxHeader.DLC);
}
