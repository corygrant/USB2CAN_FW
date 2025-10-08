#include "port.h"
#include "usb2can_config.h"

static const CANConfig canConfig1000 =
{
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    /*
     For 36MHz http://www.bittiming.can-wiki.info/ gives us Pre-scaler=2, Seq 1=15 and Seq 2=2. Subtract '1' for register values
    */
    CAN_BTR_SJW(0) | CAN_BTR_BRP(1)  | CAN_BTR_TS1(14) | CAN_BTR_TS2(1),
};

static const CANConfig canConfig500 =
{
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    /*
     For 36MHz http://www.bittiming.can-wiki.info/ gives us Pre-scaler=4, Seq 1=15 and Seq 2=2. Subtract '1' for register values
    */
    CAN_BTR_SJW(0) | CAN_BTR_BRP(3)  | CAN_BTR_TS1(14) | CAN_BTR_TS2(1),
};

static const CANConfig canConfig250 =
{
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    /*
     For 36MHz http://www.bittiming.can-wiki.info/ gives us Pre-scaler=8, Seq 1=15 and Seq 2=2. Subtract '1' for register values
    */
    CAN_BTR_SJW(0) | CAN_BTR_BRP(7)  | CAN_BTR_TS1(14) | CAN_BTR_TS2(1),
};

static const CANConfig canConfig125 =
{
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    /*
     For 36MHz http://www.bittiming.can-wiki.info/ gives us Pre-scaler=16, Seq 1=15 and Seq 2=2. Subtract '1' for register values
    */
    CAN_BTR_SJW(0) | CAN_BTR_BRP(15)  | CAN_BTR_TS1(14) | CAN_BTR_TS2(1),
};

CANConfig GetCanConfig(CanBitrate bitrate, CanMode eMode, bool bAutoRetry) 
{
    CANConfig config;

    switch(bitrate) {
        case CanBitrate::Bitrate_1000K:
            config = canConfig1000;
            break;
        case CanBitrate::Bitrate_500K:
            config = canConfig500;
            break;
        case CanBitrate::Bitrate_250K:
            config = canConfig250;
            break;
        case CanBitrate::Bitrate_125K:
            config = canConfig125;
            break;
        default:
            config = canConfig500;
    }

    if(eMode == CanMode::Silent){
        config.btr |= CAN_BTR_SILM;
    } 

    if(eMode == CanMode::Loopback){
        config.btr |= CAN_BTR_LBKM;
    }

    if(!bAutoRetry){
        config.mcr |= CAN_MCR_NART;
    }

    return config;
}
