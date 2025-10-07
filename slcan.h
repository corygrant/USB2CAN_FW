#pragma once

#include <cstdint>
#include "port.h"

enum class SLCAN_Cmd
{
    None = 0,
    Open = 'O',
    Close = 'C',
    SetBitrate = 'S',
    SetMode = 'M',
    SetAutoRetry = 'A',
    GetVersion = 'V',
    Transmit11Bit = 't',
    Transmit29Bit = 'T',
    Remote11Bit = 'r',
    Remote29Bit = 'R'
};

SLCAN_Cmd ParseSLCAN_Cmd(uint8_t nByte);
void ConvertSLCAN(uint8_t *nData, uint8_t nDataLen);
CanBitrate GetSLCAN_Bitrate(uint8_t nByte);
void FormatSLCAN_Version(uint8_t nMajor, uint8_t nMinor, uint8_t *nData);
void ParseSLCAN(uint8_t *nRxData, uint8_t nRxDataLen, CANRxFrame *stRxFrame);
uint8_t FormatSLCAN(CANTxFrame *stFrame, uint8_t *nData);