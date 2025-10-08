#include "can.h"
#include "hal.h"
#include "port.h"
#include "mailbox.h"
#include "hw_devices.h"
#include "usb2can_config.h"

#include <iterator>
#define RX_TIMEOUT_MS 100

static THD_WORKING_AREA(waCanTxThread, 128);
void CanTxThread(void *)
{
    chRegSetThreadName("CAN Tx");

    CANTxFrame msg;

    while (1)
    {
        // Send all messages in the TX queue
        msg_t res;
        do
        {
            res = FetchCanTxFrame(&msg);
            if (res == MSG_OK)
            {
                msg.IDE = CAN_IDE_STD;
                msg.RTR = CAN_RTR_DATA;
                res = canTransmitTimeout(&CAND1, CAN_ANY_MAILBOX, &msg, TIME_IMMEDIATE);
                // Returns true if mailbox full or nothing connected
                // TODO: What to do if no tx?

                chThdSleepMicroseconds(CAN_TX_MSG_SPLIT);
            }
        } while (res == MSG_OK);

        if (chThdShouldTerminateX())
            chThdExit(MSG_OK);

        chThdSleepMicroseconds(30);
    }
}

static THD_WORKING_AREA(waCanRxThread, 128);
void CanRxThread(void *)
{
    CANRxFrame msg;
    CANTxFrame txMsg;

    chRegSetThreadName("CAN Rx");

    while (true)
    {

        msg_t res = canReceiveTimeout(&CAND1, CAN_ANY_MAILBOX, &msg, TIME_IMMEDIATE);
        if (res == MSG_OK)
        {
            //Copy RX to TX frame
            txMsg.DLC = msg.DLC;
            txMsg.IDE = msg.IDE;
            txMsg.RTR = msg.RTR;
            if (msg.IDE == CAN_IDE_STD)
                txMsg.SID = msg.SID;
            else
                txMsg.EID = msg.EID;
            txMsg.data64[0] = msg.data64[0];
            
            res = PostUsbTxFrame(&txMsg);

            rxLed.Blink(20);
        }

        if (chThdShouldTerminateX())
            chThdExit(MSG_OK);

        chThdSleepMicroseconds(30);
    }
}

static thread_t *canTxThreadRef;
static thread_t *canRxThreadRef;

msg_t InitCan(CanBitrate eBitrate, CanMode eMode, bool bAutoRetry)
{
    //Stop if already running
    if (canTxThreadRef || canRxThreadRef)
        StopCan();
    
    CANConfig config = GetCanConfig(eBitrate, eMode, bAutoRetry);
    msg_t ret = canStart(&CAND1, &config);
    if (ret != HAL_RET_SUCCESS)
        return ret;
    canTxThreadRef = chThdCreateStatic(waCanTxThread, sizeof(waCanTxThread), NORMALPRIO + 1, CanTxThread, nullptr);
    canRxThreadRef = chThdCreateStatic(waCanRxThread, sizeof(waCanRxThread), NORMALPRIO + 1, CanRxThread, nullptr);

    return HAL_RET_SUCCESS;
}

void StopCan()
{
    if (!canTxThreadRef && !canRxThreadRef)
        return;

    // Signal threads to terminate
    chThdTerminate(canTxThreadRef);
    chThdTerminate(canRxThreadRef);

    // Wait for threads to exit
    chThdWait(canTxThreadRef);
    chThdWait(canRxThreadRef);

    // Stop CAN driver
    canStop(&CAND1);

    // Reset thread references
    canTxThreadRef = NULL;
    canRxThreadRef = NULL;
}