#include "ch.hpp"
#include "hal.h"

#include "usb2can_config.h"
#include "can.h"
#include "usb.h"
#include "enums.h"
#include "slcan.h"
#include "mailbox.h"

SlcanRxFrame stRxFrame;
CANTxFrame stTxFrame;
CanBitrate eBitrate = CanBitrate::Bitrate_500K;
CanMode eCanMode = CanMode::Normal;
BusState eBusState = BusState::OffBus;

/*
Available commands
O - Open channel
C - Close channel
S - Set bitrate (250k, 500k, 1M)
M - Set CAN mode (normal, silent)
A - Set auto retransmission (on/off)

T - Transmit 29 bit
t - Transmit 11 bit
R - Remote 29 bit
r - Remote 11 bit
V - Get version
*/

/*
 * Application entry point.
 */
int main(void)
{

    halInit();
    chSysInit();

    InitUsb();

    while (true)
    {
        if (usbGetDriverStateI(&USBD1) == USB_ACTIVE)
        {
            msg_t res;
            do
            {
                res = FetchUsbRxFrame(&stRxFrame);
                if (res == MSG_OK)
                {
                    // Process the received USB frame
                    switch (stRxFrame.eCmd)
                    {
                    case SLCAN_Cmd::Open:
                        res = InitCan(eBitrate, eCanMode);
                        if (res == MSG_OK)
                            eBusState = BusState::OnBus;
                        break;

                    case SLCAN_Cmd::Close:
                        StopCan();
                        ClearMailboxes();
                        eBusState = BusState::OffBus;
                        break;

                    case SLCAN_Cmd::SetBitrate:
                        if (eBusState == BusState::OnBus)
                            break; // Cannot change bitrate while on bus

                        if(stRxFrame.frame.DLC >= 2)
                            eBitrate = SLCAN::GetBitrate(stRxFrame.frame.data8[1]);
                        break;

                    case SLCAN_Cmd::SetMode:
                        if (eBusState == BusState::OnBus)
                            break; // Cannot change bitrate while on bus

                        //if(stRxFrame.DLC >= 2)
                        //    eBitrate = SLCAN::GetBitrate(stRxFrame.data8[1]);
                        break;

                    case SLCAN_Cmd::SetAutoRetry:
                        if (eBusState == BusState::OnBus)
                            break; // Cannot change bitrate while on bus

                        //if(stRxFrame.DLC >= 2)
                        //    eBitrate = SLCAN::GetBitrate(stRxFrame.data8[1]);
                        break;

                    case SLCAN_Cmd::GetVersion:
                        stTxFrame.IDE = CAN_IDE_STD;
                        stTxFrame.SID = 0;

                        SLCAN::FormatVersion(MAJOR_VERSION, MINOR_VERSION, stTxFrame.data8);
                        stTxFrame.DLC = 5;
                        PostUsbTxFrame(&stTxFrame);
                        break;

                    case SLCAN_Cmd::Transmit11Bit:
                    case SLCAN_Cmd::Transmit29Bit:
                    case SLCAN_Cmd::Remote11Bit:
                    case SLCAN_Cmd::Remote29Bit:
                        if (eBusState == BusState::OffBus)
                                break; // Cannot transmit while off bus
                        PostCanTxFrame(&stTxFrame);
                        break;
                    }
                }
            } while (res == MSG_OK);

            chThdSleepMicroseconds(30);
        }
        else
        {
            chThdSleepMilliseconds(50);
        }
    }
}
