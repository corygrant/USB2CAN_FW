/*
 * slcan.c
 *
 *  Created on: Feb 23, 2021
 *      Author: cgrant
 */

#include "slcan.h"

void SLCAN_Rx(uint8_t* nRxData, uint32_t* nRxDataLen, struct USBD_CAN_Frame *stRxFrame){
  uint8_t nDataFirstPos;

  stRxFrame->eUsbdCmd = (enum USBD_CMD)nRxData[0];

  // Convert from ASCII (2nd character to end)
  for (uint8_t i = 1; i < *nRxDataLen; i++)
  {
      // Lowercase letters
      if(nRxData[i] >= 'a')
          nRxData[i] = nRxData[i] - 'a' + 10;
      // Uppercase letters
      else if(nRxData[i] >= 'A')
          nRxData[i] = nRxData[i] - 'A' + 10;
      // Numbers
      else
          nRxData[i] = nRxData[i] - '0';
  }

  if( (stRxFrame->eUsbdCmd == USBD_CMD_SET_CAN_BITRATE) ||
      (stRxFrame->eUsbdCmd == USBD_CMD_SET_CAN_MODE) ||
      (stRxFrame->eUsbdCmd == USBD_CMD_SET_CAN_BITRATE)){
    stRxFrame->nData[1] = nRxData[1];
  }

  if(stRxFrame->eUsbdCmd == USBD_CMD_CAN_TRANSMIT_11BIT){
    stRxFrame->stTxHeader.StdId = ((nRxData[1] & 0xF) << 8) + ((nRxData[2] & 0xF) << 4) + (nRxData[3] & 0xF);

    stRxFrame->stTxHeader.DLC = nRxData[4];

    nDataFirstPos = 5;

    for(int i=0; i < stRxFrame->stTxHeader.DLC; i++){
      stRxFrame->nData[i] = ((nRxData[i + nDataFirstPos] & 0xF) << 4) + (nRxData[i + nDataFirstPos + 1] & 0xF);
      nDataFirstPos++;
    }

    stRxFrame->stTxHeader.ExtId = 0;
    stRxFrame->stTxHeader.IDE = CAN_ID_STD;
    stRxFrame->stTxHeader.RTR = CAN_RTR_DATA;
    stRxFrame->stTxHeader.TransmitGlobalTime = DISABLE;
  }

  if(stRxFrame->eUsbdCmd == USBD_CMD_CAN_REMOTE_11BIT){
    stRxFrame->stTxHeader.StdId = ((nRxData[1] & 0xF) << 8) + ((nRxData[2] & 0xF) << 4) + (nRxData[3] & 0xF);

    stRxFrame->stTxHeader.ExtId = 0;
    stRxFrame->stTxHeader.IDE = CAN_ID_STD;
    stRxFrame->stTxHeader.RTR = CAN_RTR_REMOTE;
    stRxFrame->stTxHeader.TransmitGlobalTime = DISABLE;
  }

  if(stRxFrame->eUsbdCmd == USBD_CMD_CAN_TRANSMIT_29BIT){
    stRxFrame->stTxHeader.ExtId = ((nRxData[1] & 0xF) << 28) + ((nRxData[2] & 0xF) << 24) + ((nRxData[3] & 0xF) << 20) + ((nRxData[4] & 0xF) << 16) +
                                  ((nRxData[5] & 0xF) << 12) + ((nRxData[6] & 0xF) << 8) + ((nRxData[7] & 0xF) << 4) + (nRxData[8] & 0xF);

    stRxFrame->stTxHeader.DLC = nRxData[9];

    nDataFirstPos = 10;

    for(int i=0; i < stRxFrame->stTxHeader.DLC; i++){
      stRxFrame->nData[i] = ((nRxData[i + nDataFirstPos] & 0xF) << 4) + (nRxData[i + nDataFirstPos + 1] & 0xF);
      nDataFirstPos++;
    }

    stRxFrame->stTxHeader.StdId = 0;
    stRxFrame->stTxHeader.IDE = CAN_ID_EXT;
    stRxFrame->stTxHeader.RTR = CAN_RTR_DATA;
    stRxFrame->stTxHeader.TransmitGlobalTime = DISABLE;
  }

  if(stRxFrame->eUsbdCmd == USBD_CMD_CAN_REMOTE_29BIT){
    stRxFrame->stTxHeader.ExtId = ((nRxData[1] & 0xF) << 28) + ((nRxData[2] & 0xF) << 24) + ((nRxData[3] & 0xF) << 20) + ((nRxData[4] & 0xF) << 16) +
                                  ((nRxData[5] & 0xF) << 12) + ((nRxData[6] & 0xF) << 8) + ((nRxData[7] & 0xF) << 4) + (nRxData[8] & 0xF);

    stRxFrame->stTxHeader.StdId = 0;
    stRxFrame->stTxHeader.IDE = CAN_ID_EXT;
    stRxFrame->stTxHeader.RTR = CAN_RTR_REMOTE;
    stRxFrame->stTxHeader.TransmitGlobalTime = DISABLE;
  }
}

uint8_t SLCAN_Tx(struct USBD_CAN_Frame *stTxFrame, uint8_t* nTxData){
  uint8_t nFirstDataPos;
  uint8_t nLastDataPos;

  for(uint8_t i=0; i < 30; i++)
    nTxData[i] = '\0';

  if(stTxFrame->stRxHeader.RTR == CAN_RTR_DATA){
    if(stTxFrame->stRxHeader.IDE == CAN_ID_STD){
      nTxData[0] = 't';
    } else{
      nTxData[0] = 'T';
    }
  } else{
    if(stTxFrame->stRxHeader.IDE == CAN_ID_STD){
      nTxData[0] = 'r';
    } else{
      nTxData[0] = 'R';
    }
  }

  if(stTxFrame->stRxHeader.IDE == CAN_ID_STD){
    nTxData[1] = (stTxFrame->stRxHeader.StdId >> 8) & 0xF;
    nTxData[2] = (stTxFrame->stRxHeader.StdId >> 4) & 0xF;
    nTxData[3] = stTxFrame->stRxHeader.StdId & 0xF;

    nTxData[4] = (stTxFrame->stRxHeader.DLC & 0xFF);
    nFirstDataPos = 5;
  } else{
    nTxData[1] = (stTxFrame->stRxHeader.StdId >> 28) & 0xF;
    nTxData[2] = (stTxFrame->stRxHeader.StdId >> 24) & 0xF;
    nTxData[3] = (stTxFrame->stRxHeader.StdId >> 20) & 0xF;
    nTxData[4] = (stTxFrame->stRxHeader.StdId >> 16) & 0xF;
    nTxData[5] = (stTxFrame->stRxHeader.StdId >> 12) & 0xF;
    nTxData[6] = (stTxFrame->stRxHeader.StdId >> 8) & 0xF;
    nTxData[7] = (stTxFrame->stRxHeader.StdId >> 4) & 0xF;
    nTxData[8] = stTxFrame->stRxHeader.StdId & 0xF;

    nTxData[9] = (stTxFrame->stRxHeader.DLC & 0xFF);
    nFirstDataPos = 10;
  }

  nLastDataPos = nFirstDataPos;
  for(uint8_t i=0; i < stTxFrame->stRxHeader.DLC; i++){
    nTxData[i + nFirstDataPos] = (stTxFrame->nData[i] >> 4);
    nFirstDataPos++;
    nTxData[i + nFirstDataPos] = (stTxFrame->nData[i] & 0x0F);
    nLastDataPos = i + nFirstDataPos;
  }

  for(uint8_t j = 1; j <= nLastDataPos; j++){
    if(nTxData[j] < 0xA){
      nTxData[j] += 0x30;
    } else{
      nTxData[j] += 0x37;
    }
  }

  nLastDataPos++;
  nTxData[nLastDataPos] = '\r';

  //Increment by 1 to return total size (include 0 index)
  return ++nLastDataPos;
}
