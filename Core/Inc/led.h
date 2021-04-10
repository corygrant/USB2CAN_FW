/*
 * led.h
 *
 *  Created on: Feb 23, 2021
 *      Author: coryg
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#include "stdint.h"
#include "stm32f3xx_hal.h"

typedef struct{
  GPIO_TypeDef *pPort;
  uint16_t nPin;
  uint32_t nOnUntil;
} Led_t;

void LedInit(Led_t* stLed, GPIO_TypeDef *pPort, uint16_t nPin);
void LedUpdate(Led_t* stLed);
void LedBlink(Led_t* stLed, uint16_t nOnTime);
#endif /* INC_LED_H_ */
