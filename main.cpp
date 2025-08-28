#include "ch.h"
#include "hal.h"

#include "canboard_config.h"
#include "can.h"

/*
 * Application entry point.
 */
int main(void)
{

  halInit();
  chSysInit();

  InitCan();

  while (true)
  {
    UpdateSwPos();
    chThdSleepMilliseconds(500);
  }
}
