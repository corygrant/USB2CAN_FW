/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_desc.h"
#include "usbd_def.h"
#include "usbd_core.h"
#include "usbd_cdc.h"

#include "init.h"
#include "queue.h"
#include "slcan.h"
#include <stdlib.h>
#include "gpio.h"
#include "led.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USBD_RX_DATA_SIZE  2048
#define USBD_TX_DATA_SIZE  2048
#define CAN_QUEUE_SIZE 64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */
static CAN_RxHeaderTypeDef   stCanRxHeader;
static uint8_t               nCanRxData[8];
static uint32_t              nCanTxMailbox;

enum CAN_BUS_STATE eCanBusState = OFF_BUS;
enum CAN_BITRATE eCanBitRate = CAN_BITRATE_500K;
uint32_t nCanMode = CAN_MODE_NORMAL;
FunctionalState eAutoRetry;

queue_t *qFramePool;
queue_t *qFromHost;
queue_t *qToHost;


uint8_t USBD_RxBuffer[USBD_RX_DATA_SIZE];
uint8_t USBD_TxBuffer[USBD_TX_DATA_SIZE];
USBD_HandleTypeDef hUSBD;

uint8_t nUsbConnected;

static int8_t USBD_CDC_Init(void);
static int8_t USBD_CDC_DeInit(void);
static int8_t USBD_CDC_Control(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t USBD_CDC_Receive(uint8_t* pbuf, uint32_t *Len);

USBD_CDC_ItfTypeDef USBD_Interface =
{
  USBD_CDC_Init,
  USBD_CDC_DeInit,
  USBD_CDC_Control,
  USBD_CDC_Receive
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//==============================================================================================================================================

/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t USBD_CDC_Init(void)
{
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUSBD, USBD_TxBuffer, 0);
  USBD_CDC_SetRxBuffer(&hUSBD, USBD_RxBuffer);
  return (USBD_OK);
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t USBD_CDC_DeInit(void)
{
  return (USBD_OK);
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t USBD_CDC_Control(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:

    break;

    case CDC_GET_LINE_CODING:
      pbuf[0] = (uint8_t)(115200);
      pbuf[1] = (uint8_t)(115200 >> 8);
      pbuf[2] = (uint8_t)(115200 >> 16);
      pbuf[3] = (uint8_t)(115200 >> 24);
      pbuf[4] = 0; //Stop bits (1)
      pbuf[5] = 0; //Parity (none)
      pbuf[6] = 8; //Number of bits (8)
    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t USBD_CDC_Receive(uint8_t* Buf, uint32_t *Len)
{


  struct USBD_CAN_Frame *stFrame = queue_pop_front_i(qFramePool);
  if(stFrame){
    SLCAN_Rx(Buf, Len, stFrame);
    if(queue_push_back_i(qFromHost, stFrame) == false){
      queue_push_back_i(qFramePool, stFrame);
    }
  }

  USBD_CDC_SetRxBuffer(&hUSBD, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUSBD);
  return (USBD_OK);
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t USBD_CDC_Transmit(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUSBD.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUSBD, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUSBD);
  return result;
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

  if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &stCanRxHeader, nCanRxData) != HAL_OK)
  {
    Error_Handler();
  }

  //Add message to CAN RX queue
  struct USBD_CAN_Frame *stFrame = queue_pop_front_i(qFramePool);
  if(stFrame != 0){
    memcpy(&stFrame->stRxHeader, &stCanRxHeader, sizeof(stFrame->stRxHeader));
    memcpy(&stFrame->nData, nCanRxData, sizeof(stFrame->nData));
    queue_push_back_i(qToHost, stFrame);
  }

  LedBlink(&RXLed, 20);
}

void CAN_Enable()
{
  if(eCanBusState == OFF_BUS){
    uint32_t nPrescaler = 4; //Default 500k
    uint32_t nTimeSeq1 = CAN_BS1_15TQ;

    switch (eCanBitRate)
    {
      case CAN_BITRATE_10K:
        nPrescaler = 200;
        break;
      case CAN_BITRATE_20K:
        nPrescaler = 100;
        break;
      case CAN_BITRATE_50K:
        nPrescaler = 40;
        break;
      case CAN_BITRATE_100K:
        nPrescaler = 20;
        break;
      case CAN_BITRATE_125K:
        nPrescaler = 16;
        break;
      case CAN_BITRATE_250K:
        nPrescaler = 8;
        break;
      case CAN_BITRATE_500K:
        nPrescaler = 4;
        break;
      case CAN_BITRATE_750K:
        nPrescaler = 8;
        nTimeSeq1 = CAN_BS1_13TQ;
        break;
      case CAN_BITRATE_1000K:
        nPrescaler = 2;
        break;
      case CAN_BITRATE_INVALID:
        //Do nothing - use default vals
        assert_param(0);
        break;
    }

    hcan.Instance = CAN;
    hcan.Init.Prescaler = nPrescaler;
    hcan.Init.Mode = nCanMode;
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = nTimeSeq1;
    hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = DISABLE;
    hcan.Init.AutoWakeUp = DISABLE;
    hcan.Init.AutoRetransmission = eAutoRetry;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(&hcan) != HAL_OK)
      Error_Handler();

    //---------------------------------------------------------------
    //Set CAN RX filter
    //---------------------------------------------------------------
    CAN_FilterTypeDef  sFilterConfig;

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
      Error_Handler();

    if (HAL_CAN_Start(&hcan) != HAL_OK)
      Error_Handler();

    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
      Error_Handler();

    eCanBusState = ON_BUS;
  }
}

void CAN_Disable()
{
  if(eCanBusState == ON_BUS){
    if (HAL_CAN_DeactivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
      Error_Handler();

    if (HAL_CAN_Stop(&hcan) != HAL_OK)
      Error_Handler();

    if (HAL_CAN_DeInit(&hcan) != HAL_OK)
      Error_Handler();

    eCanBusState = OFF_BUS;
  }
}

void CAN_SetBitRate(uint8_t nBitRate)
{
  if(eCanBusState == OFF_BUS){
    if(nBitRate < CAN_BITRATE_INVALID){
      eCanBitRate = (enum CAN_BITRATE)nBitRate;
    }
  }
}

void CAN_SetMode(uint32_t nMode)
{
  if(eCanBusState == OFF_BUS){
    if(nMode == CAN_MODE_NORMAL)
      nCanMode = nMode;
    if(nMode == CAN_MODE_LOOPBACK)
      nCanMode = nMode;
    if(nMode == CAN_MODE_SILENT)
      nCanMode = nMode;
    if(nMode == CAN_MODE_SILENT_LOOPBACK)
      nCanMode = nMode;
  }
}

void CAN_SetAutoRetry(uint8_t nAutoRetry)
{
  if(eCanBusState == OFF_BUS){
    if(nAutoRetry == 1)
      eAutoRetry = ENABLE;
    else
      eAutoRetry = DISABLE;
  }
}

void HAL_RCC_CSSCallback(void) {
  //Catch the HSE failure and take proper actions
  while (1)
    {
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  if(SystemClock_Config() != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN SysInit */
  HAL_RCC_EnableCSS();
  /* USER CODE END SysInit */

  InitGPIO();

  /* USER CODE BEGIN 2 */

  /* Init Device Library, add supported class and start the library. */
  if (USBD_Init(&hUSBD, &FS_Desc, DEVICE_FS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_RegisterClass(&hUSBD, &USBD_CDC) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_CDC_RegisterInterface(&hUSBD, &USBD_Interface) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_Start(&hUSBD) != USBD_OK)
  {
    Error_Handler();
  }

  //Points to frame memory location
  qFramePool  = queue_create(CAN_QUEUE_SIZE);

  //Queue of pointers to pointers popped from the frame pool
  qFromHost   = queue_create(CAN_QUEUE_SIZE);

  //Queue of pointers to pointers popped from the frame pool
  qToHost     = queue_create(CAN_QUEUE_SIZE);

  //Allocate memory that the frame pool points to
  struct USBD_CAN_Frame *msgbuf = calloc(CAN_QUEUE_SIZE, sizeof(struct USBD_CAN_Frame));
  for (unsigned i=0; i<CAN_QUEUE_SIZE; i++) {
    //Fill the frame pool with pointers to the memory that was just allocated
    queue_push_back(qFramePool, &msgbuf[i]);
  }


  LedBlink(&RXLed, 1000);
  LedBlink(&TXLed, 1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    LedUpdate(&TXLed);
    LedUpdate(&RXLed);

    //Check for messages in USB To Host queue
    //Send to USB host
    //If success - add pointer back to frame pool
    //If failed - add pointer back to the front of the to host queue
    struct USBD_CAN_Frame *stToHostFrame = queue_pop_front(qToHost);
    if(stToHostFrame != 0){
      //Build USB array
      uint8_t nUsbData[30];
      uint8_t nUsbDataLen = SLCAN_Tx(stToHostFrame, nUsbData);

      if(USBD_CDC_Transmit(nUsbData, (uint16_t)nUsbDataLen) == USBD_OK){
        queue_push_back(qFramePool, stToHostFrame);
      } else{
        queue_push_front(qToHost, stToHostFrame);
      }
    }

    struct USBD_CAN_Frame *stFromHostFrame = queue_pop_front(qFromHost);
    if(stFromHostFrame != 0){
      switch(stFromHostFrame->eUsbdCmd){
      case USBD_CMD_OPEN_CAN:
        CAN_Enable();
        queue_push_back(qFramePool, stFromHostFrame);
        break;

      case USBD_CMD_CLOSE_CAN:
        CAN_Disable();
        queue_push_back(qFramePool, stFromHostFrame);
        break;

      case USBD_CMD_SET_CAN_BITRATE:
        CAN_SetBitRate(stFromHostFrame->nData[1]);
        queue_push_back(qFramePool, stFromHostFrame);
        break;

      case USBD_CMD_SET_CAN_MODE:
        CAN_SetMode(  (stFromHostFrame->nData[1] << 24) + (stFromHostFrame->nData[2] << 16) +
                      (stFromHostFrame->nData[3] << 8) + stFromHostFrame->nData[4]);
        queue_push_back(qFramePool, stFromHostFrame);
        break;

      case USBD_CMD_SET_CAN_AUTORETRY:
        CAN_SetAutoRetry(stFromHostFrame->nData[1]);
        queue_push_back(qFramePool, stFromHostFrame);
        break;

      case USBD_CMD_GET_VERSION:
        break;

      default:
        if((stFromHostFrame->eUsbdCmd == USBD_CMD_CAN_TRANSMIT_11BIT) ||
            (stFromHostFrame->eUsbdCmd == USBD_CMD_CAN_REMOTE_11BIT) ||
            (stFromHostFrame->eUsbdCmd == USBD_CMD_CAN_TRANSMIT_29BIT) ||
            (stFromHostFrame->eUsbdCmd == USBD_CMD_CAN_REMOTE_29BIT)){
          //Add to queue
          if(HAL_CAN_AddTxMessage(&hcan, &stFromHostFrame->stTxHeader, stFromHostFrame->nData, &nCanTxMailbox) == HAL_OK){
            //Tx success - put pointer back in memory pool
            LedBlink(&TXLed, 20);
            queue_push_back(qFramePool, stFromHostFrame);
          }else{
            //Tx failed - add back to front of queue
            queue_push_front(qFromHost, stFromHostFrame);
          }
        }
        break;
      }
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */



/* USER CODE BEGIN 4 */

/* Configuration Descriptor */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
