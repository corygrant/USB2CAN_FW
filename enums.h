#pragma once

#include <cstdint>

enum class CanBitrate : uint8_t
{
    Bitrate_1000K,
    Bitrate_500K,
    Bitrate_250K,
    Bitrate_125K
};

enum class CanMode : uint8_t
{
    Normal,
    Silent,
    Loopback
};

enum class BusState : uint8_t
{
    OffBus,
    OnBus
};

enum class SLCAN_Cmd
{
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

struct SlcanRxFrame
{
    SLCAN_Cmd eCmd;
    CANRxFrame frame;
};