#include "ch.hpp"
#include "hal.h"

#include "usb2can_config.h"
#include "can.h"
#include "usb.h"
#include "enums.h"
#include "slcan.h"
#include "mailbox.h"

SLCAN_Cmd eCmd;
CANRxFrame stRxFrame;
CANTxFrame stTxFrame;
msg_t res;
CanBitrate eBitrate = CanBitrate::Bitrate_500K;

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
            do
            {
                res = FetchUsbRxFrame(&stRxFrame);
                if (res == MSG_OK)
                {
                    // Process the received USB frame
                    eCmd = ParseSLCAN_Cmd(stRxFrame.data8[0]);
                    switch (eCmd)
                    {
                    case SLCAN_Cmd::Open:
                        res = InitCan(eBitrate);
                        if (res == MSG_OK)
                        {
                            // Successfully opened CAN
                            // Respond with 'O'\r
                        }
                        else
                        {
                            // Failed to open CAN
                            // Respond with \x07\r
                        }
                        break;
                    case SLCAN_Cmd::Close:
                        StopCan();
                        if (res == MSG_OK)
                        {
                            // Successfully closed CAN
                            // Respond with 'C'\r
                        }
                        else
                        {
                            // Failed to close CAN
                            // Respond with \x07\r
                        }
                        break;
                    case SLCAN_Cmd::SetBitrate:
                        if(stRxFrame.DLC >= 2)
                            eBitrate = GetSLCAN_Bitrate(stRxFrame.data8[1]);

                        // If invalid bitrate, respond with \x07\r
                        // Else respond with 'Sn\r'

                        break;
                    case SLCAN_Cmd::SetMode:
                        // Handle SetMode command
                        // If invalid mode, respond with \x07\r
                        // Else respond with 'Mn\r'
                        break;
                    case SLCAN_Cmd::SetAutoRetry:
                        // Handle SetAutoRetry command
                        // If invalid setting, respond with \x07\r
                        // Else respond with 'An\r'
                        break;
                    case SLCAN_Cmd::GetVersion:
                        stTxFrame.IDE = CAN_IDE_STD;
                        stTxFrame.SID = 0;

                        FormatSLCAN_Version(MAJOR_VERSION, MINOR_VERSION, stTxFrame.data8);
                        stTxFrame.DLC = 5;
                        PostUsbTxFrame(&stTxFrame);
                        break;
                    case SLCAN_Cmd::Transmit11Bit:
                    case SLCAN_Cmd::Transmit29Bit:
                    case SLCAN_Cmd::Remote11Bit:
                    case SLCAN_Cmd::Remote29Bit:
                        PostCanTxFrame(&stTxFrame);
                        break;
                    case SLCAN_Cmd::None:
                        // No command
                        // Respond with \x07\r
                        break;
                    default:
                        // Unknown or no command
                        // Respond with \x07\r
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
