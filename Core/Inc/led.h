/*
 * led.h
 *
 *  Created on: Feb 23, 2021
 *      Author: coryg
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#include "stdint.h"
#include "gpio.h"

void LedUpdate(GPIO_Output* out);
void LedBlink(GPIO_Output* out, uint16_t nOnTime);
#endif /* INC_LED_H_ */
