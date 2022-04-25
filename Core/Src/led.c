/*
 * led.c
 *
 *  Created on: Feb 23, 2021
 *      Author: coryg
 */

#include "led.h"

void LedUpdate(GPIO_Output* out){
  uint32_t nNow = HAL_GetTick();

  if(nNow < out->nOnUntil){
    out->On();
  } else{
    out->Off();
  }
}

void LedBlink(GPIO_Output* out, uint16_t nOnTime){
  out->nOnUntil = HAL_GetTick() + nOnTime;
}
