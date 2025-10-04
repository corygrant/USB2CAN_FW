#include "can.h"
#include "hal.h"
#include "port.h"
#include "mailbox.h"
#include "usb2can_config.h"

#include <iterator>
#define RX_TIMEOUT_MS 100

static uint32_t nLastCanRxTime;

static THD_WORKING_AREA(waCanTxThread, 256);
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
            res = FetchTxFrame(&msg);
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

    chRegSetThreadName("CAN Rx");

    while (true)
    {

        msg_t res = canReceiveTimeout(&CAND1, CAN_ANY_MAILBOX, &msg, TIME_IMMEDIATE);
        if (res == MSG_OK)
        {
            nLastCanRxTime = SYS_TIME;

            res = PostRxFrame(&msg);
        }

        if (chThdShouldTerminateX())
            chThdExit(MSG_OK);

        chThdSleepMicroseconds(30);
    }
}

static thread_t *canTxThreadRef;
static thread_t *canRxThreadRef;

msg_t InitCan(CanBitrate eBitrate)
{
    msg_t ret = canStart(&CAND1, &GetCanConfig(eBitrate));
    if (ret != HAL_RET_SUCCESS)
        return ret;
    canTxThreadRef = chThdCreateStatic(waCanTxThread, sizeof(waCanTxThread), NORMALPRIO + 1, CanTxThread, nullptr);
    canRxThreadRef = chThdCreateStatic(waCanRxThread, sizeof(waCanRxThread), NORMALPRIO + 1, CanRxThread, nullptr);

    return HAL_RET_SUCCESS;
}

bool CanRxIsActive()
{
    return (SYS_TIME - nLastCanRxTime) < RX_TIMEOUT_MS;
}