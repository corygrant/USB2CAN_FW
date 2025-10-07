#include "slcan.h"

SLCAN_Cmd SLCAN::ParseCmd(uint8_t nByte)
{
    switch (nByte)
    {
    case 'O':
        return SLCAN_Cmd::Open;
    case 'C':
        return SLCAN_Cmd::Close;
    case 'S':
        return SLCAN_Cmd::SetBitrate;
    case 'M':
        return SLCAN_Cmd::SetMode;
    case 'A':
        return SLCAN_Cmd::SetAutoRetry;
    case 'V':
        return SLCAN_Cmd::GetVersion;
    case 't':
        return SLCAN_Cmd::Transmit11Bit;
    case 'T':
        return SLCAN_Cmd::Transmit29Bit;
    case 'r':
        return SLCAN_Cmd::Remote11Bit;
    case 'R':
        return SLCAN_Cmd::Remote29Bit;
    default:
        return SLCAN_Cmd::GetVersion; // Default to GetVersion if unknown command
    }
}

CanBitrate SLCAN::GetBitrate(uint8_t nByte)
{

    if ((nByte == 4) || (nByte == '4'))
        return CanBitrate::Bitrate_125K;

    if ((nByte == 5) || (nByte == '5'))
        return CanBitrate::Bitrate_250K;

    if ((nByte == 6) || (nByte == '6'))
        return CanBitrate::Bitrate_500K;

    if ((nByte == 8) || (nByte == '8'))
        return CanBitrate::Bitrate_1000K;

    return CanBitrate::Bitrate_500K; // Invalid bitrate
}

void SLCAN::Convert(uint8_t *nData, uint8_t nDataLen)
{
    // Convert from ASCII (2nd character to end)
    for (uint8_t i = 1; i < nDataLen; i++)
    {
        // Lowercase letters
        if (nData[i] >= 'a')
            nData[i] = nData[i] - 'a' + 10;
        // Uppercase letters
        else if (nData[i] >= 'A')
            nData[i] = nData[i] - 'A' + 10;
        // Numbers
        else
            nData[i] = nData[i] - '0';
    }
}

void SLCAN::FormatVersion(uint8_t nMajor, uint8_t nMinor, uint8_t *nData)
{
    nData[0] = 'V';
    nData[1] = (nMajor / 10) + '0';
    nData[2] = (nMajor % 10) + '0';
    nData[3] = (nMinor / 10) + '0';
    nData[4] = (nMinor % 10) + '0';
}

void SLCAN::Parse(uint8_t *nRxData, uint8_t nRxDataLen, SlcanRxFrame *stRxFrame)
{
    uint8_t nDataFirstPos;

    stRxFrame->eCmd = ParseCmd(nRxData[0]);

    // Convert from ASCII (2nd character to end)
    for (uint8_t i = 1; i < nRxDataLen; i++)
    {
        // Lowercase letters
        if (nRxData[i] >= 'a')
            nRxData[i] = nRxData[i] - 'a' + 10;
        // Uppercase letters
        else if (nRxData[i] >= 'A')
            nRxData[i] = nRxData[i] - 'A' + 10;
        // Numbers
        else
            nRxData[i] = nRxData[i] - '0';
    }

    if ((stRxFrame->eCmd == SLCAN_Cmd::SetBitrate) ||
        (stRxFrame->eCmd == SLCAN_Cmd::SetAutoRetry))
    {
        stRxFrame->frame.data8[1] = nRxData[1];
    }

    if (stRxFrame->eCmd == SLCAN_Cmd::SetMode)
    {
        stRxFrame->frame.data8[1] = nRxData[1];
        stRxFrame->frame.data8[2] = nRxData[2];
        stRxFrame->frame.data8[3] = nRxData[3];
        stRxFrame->frame.data8[4] = nRxData[4];
    }

    if (stRxFrame->eCmd == SLCAN_Cmd::Transmit11Bit)
    {
        stRxFrame->frame.SID = ((nRxData[1] & 0xF) << 8) + ((nRxData[2] & 0xF) << 4) + (nRxData[3] & 0xF);

        stRxFrame->frame.DLC = nRxData[4];

        nDataFirstPos = 5;

        for (int i = 0; i < stRxFrame->frame.DLC; i++)
        {
            stRxFrame->frame.data8[i] = ((nRxData[i + nDataFirstPos] & 0xF) << 4) + (nRxData[i + nDataFirstPos + 1] & 0xF);
            nDataFirstPos++;
        }

        stRxFrame->frame.EID = 0;
        stRxFrame->frame.IDE = CAN_IDE_STD;
        stRxFrame->frame.RTR = CAN_RTR_DATA;
    }

    if (stRxFrame->eCmd == SLCAN_Cmd::Remote11Bit)
    {
        stRxFrame->frame.SID = ((nRxData[1] & 0xF) << 8) + ((nRxData[2] & 0xF) << 4) + (nRxData[3] & 0xF);

        stRxFrame->frame.EID = 0;
        stRxFrame->frame.IDE = CAN_IDE_STD;
        stRxFrame->frame.RTR = CAN_RTR_REMOTE;
    }

    if (stRxFrame->eCmd == SLCAN_Cmd::Transmit29Bit)
    {
        stRxFrame->frame.EID = ((nRxData[1] & 0xF) << 28) + ((nRxData[2] & 0xF) << 24) + ((nRxData[3] & 0xF) << 20) + ((nRxData[4] & 0xF) << 16) +
                         ((nRxData[5] & 0xF) << 12) + ((nRxData[6] & 0xF) << 8) + ((nRxData[7] & 0xF) << 4) + (nRxData[8] & 0xF);

        stRxFrame->frame.DLC = nRxData[9];

        nDataFirstPos = 10;

        for (int i = 0; i < stRxFrame->frame.DLC; i++)
        {
            stRxFrame->frame.data8[i] = ((nRxData[i + nDataFirstPos] & 0xF) << 4) + (nRxData[i + nDataFirstPos + 1] & 0xF);
            nDataFirstPos++;
        }

        stRxFrame->frame.SID = 0;
        stRxFrame->frame.IDE = CAN_IDE_EXT;
        stRxFrame->frame.RTR = CAN_RTR_DATA;
    }

    if (stRxFrame->eCmd == SLCAN_Cmd::Remote29Bit)
    {
        stRxFrame->frame.EID = ((nRxData[1] & 0xF) << 28) + ((nRxData[2] & 0xF) << 24) + ((nRxData[3] & 0xF) << 20) + ((nRxData[4] & 0xF) << 16) +
                                      ((nRxData[5] & 0xF) << 12) + ((nRxData[6] & 0xF) << 8) + ((nRxData[7] & 0xF) << 4) + (nRxData[8] & 0xF);

        stRxFrame->frame.SID = 0;
        stRxFrame->frame.IDE = CAN_IDE_EXT;
        stRxFrame->frame.RTR = CAN_RTR_REMOTE;
    }
}

uint8_t SLCAN::Format(CANTxFrame *stFrame, uint8_t *nData)
{
    uint8_t nFirstDataPos;
    uint8_t nLastDataPos;

    for (uint8_t i = 0; i < 30; i++)
        nData[i] = '\0';

    if (stFrame->RTR == CAN_RTR_DATA)
    {
        if (stFrame->IDE == CAN_IDE_STD)
        {
            nData[0] = 't';
        }
        else
        {
            nData[0] = 'T';
        }
    }
    else
    {
        if (stFrame->IDE == CAN_IDE_STD)
        {
            nData[0] = 'r';
        }
        else
        {
            nData[0] = 'R';
        }
    }

    if (stFrame->IDE == CAN_IDE_STD)
    {
        nData[1] = (stFrame->SID >> 8) & 0xF;
        nData[2] = (stFrame->SID >> 4) & 0xF;
        nData[3] = stFrame->SID & 0xF;

        nData[4] = (stFrame->DLC & 0xFF);
        nFirstDataPos = 5;
    }
    else
    {
        nData[1] = (stFrame->SID >> 28) & 0xF;
        nData[2] = (stFrame->SID >> 24) & 0xF;
        nData[3] = (stFrame->SID >> 20) & 0xF;
        nData[4] = (stFrame->SID >> 16) & 0xF;
        nData[5] = (stFrame->SID >> 12) & 0xF;
        nData[6] = (stFrame->SID >> 8) & 0xF;
        nData[7] = (stFrame->SID >> 4) & 0xF;
        nData[8] = stFrame->SID & 0xF;

        nData[9] = (stFrame->DLC & 0xFF);
        nFirstDataPos = 10;
    }

    nLastDataPos = nFirstDataPos;
    for (uint8_t i = 0; i < stFrame->DLC; i++)
    {
        nData[i + nFirstDataPos] = (stFrame->data8[i] >> 4);
        nFirstDataPos++;
        nData[i + nFirstDataPos] = (stFrame->data8[i] & 0x0F);
        nLastDataPos = i + nFirstDataPos;
    }

    for (uint8_t j = 1; j <= nLastDataPos; j++)
    {
        if (nData[j] < 0xA)
        {
            nData[j] += 0x30;
        }
        else
        {
            nData[j] += 0x37;
        }
    }

    nLastDataPos++;
    nData[nLastDataPos] = '\r';

    // Increment by 1 to return total size (include 0 index)
    return ++nLastDataPos;
}