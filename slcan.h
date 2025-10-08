#pragma once

#include <cstdint>
#include "port.h"

class SLCAN{
public:
    static SLCAN_Cmd ParseCmd(uint8_t nByte);
    static void Parse(uint8_t *nRxData, uint8_t nRxDataLen, SlcanRxFrame *stRxFrame);

    static void Convert(uint8_t *nData, uint8_t nDataLen);
    static CanBitrate GetBitrate(uint8_t nByte);
    static CanMode GetMode(uint8_t nByte);
    static bool GetAutoRetry(uint8_t nByte);

    static void FormatVersion(uint8_t nMajor, uint8_t nMinor, uint8_t *nData);
    static uint8_t Format(CANTxFrame *stFrame, uint8_t *nData);

private:

};