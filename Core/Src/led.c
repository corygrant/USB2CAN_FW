/*
 * led.c
 *
 *  Created on: Feb 23, 2021
 *      Author: coryg
 */

#include "led.h"

void LedInit(Led_t* stLed, GPIO_TypeDef *pPort, uint16_t nPin){
  stLed->pPort = pPort;
  stLed->nPin = nPin;
}

void LedUpdate(Led_t* stLed){
  uint32_t nNow = HAL_GetTick();

  if(nNow < stLed->nOnUntil){
    stLed->pPort->ODR |= stLed->nPin;
  } else{
    stLed->pPort->ODR &= ~stLed->nPin;
  }
}

void LedBlink(Led_t* stLed, uint16_t nOnTime){
  stLed->nOnUntil = HAL_GetTick() + nOnTime;
}
