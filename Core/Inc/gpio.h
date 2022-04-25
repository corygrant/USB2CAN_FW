/*
 * gpio.h
 *
 *  Created on: Apr 24, 2022
 *      Author: coryg
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_

#include "stm32f3xx_hal.h"

//Create a typedef defining a simple function pointer
//to be used for LED's
typedef void (*GPIOFunc)(void);

//this struct holds function pointers to turn each LED
//on and off
typedef struct
{
  const GPIOFunc On;
  const GPIOFunc Off;
  uint32_t nOnUntil;
}GPIO_Output;

uint_fast8_t ReadUSB_VBUS( void );

extern GPIO_Output TXLed;
extern GPIO_Output RXLed;
extern GPIO_Output Extra;
extern GPIO_Output USB_PU;

void InitGPIO(void);

#endif /* INC_GPIO_H_ */
