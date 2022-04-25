/*
 * slcan.h
 *
 *  Created on: Feb 23, 2021
 *      Author: cgrant
 */

#ifndef INC_SLCAN_H_
#define INC_SLCAN_H_

#include "stdlib.h"
#include "stm32f3xx_hal.h"

enum CAN_BITRATE {
    CAN_BITRATE_10K = 0,
    CAN_BITRATE_20K,
    CAN_BITRATE_50K,
    CAN_BITRATE_100K,
    CAN_BITRATE_125K,
    CAN_BITRATE_250K,
    CAN_BITRATE_500K,
    CAN_BITRATE_750K,
    CAN_BITRATE_1000K,
    CAN_BITRATE_INVALID,
};

enum CAN_BUS_STATE {
    OFF_BUS,
    ON_BUS
};

enum USBD_CMD{
  USBD_CMD_OPEN_CAN = 'O',
  USBD_CMD_CLOSE_CAN = 'C',
  USBD_CMD_SET_CAN_BITRATE = 'S',
  USBD_CMD_SET_CAN_MODE = 'M',
  USBD_CMD_SET_CAN_AUTORETRY = 'A',
  USBD_CMD_GET_VERSION = 'V',
  USBD_CMD_CAN_TRANSMIT_11BIT = 't',
  USBD_CMD_CAN_TRANSMIT_29BIT = 'T',
  USBD_CMD_CAN_REMOTE_11BIT = 'r',
  USBD_CMD_CAN_REMOTE_29BIT = 'R'
};

/*
struct __attribute__((__packed__)) USBD_CAN_Frame{
  enum USBD_CMD eUsbdCmd;
  CAN_TxHeaderTypeDef stTxHeader;
  CAN_RxHeaderTypeDef stRxHeader;
  uint8_t nData[8];
};
*/
struct USBD_CAN_Frame{
  enum USBD_CMD eUsbdCmd;
  CAN_TxHeaderTypeDef stTxHeader;
  CAN_RxHeaderTypeDef stRxHeader;
  uint8_t nData[8];
};

void SLCAN_Rx(uint8_t* nRxData, uint32_t* nRxDataLen, struct USBD_CAN_Frame *stRxFrame);
uint8_t SLCAN_Tx(struct USBD_CAN_Frame *stTxFrame, uint8_t* nTxData);

#endif /* INC_SLCAN_H_ */
