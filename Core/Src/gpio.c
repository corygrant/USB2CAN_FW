/*
 * gpio.c
 *
 *  Created on: Apr 24, 2022
 *      Author: coryg
 */

#include "gpio.h"

void TXLedOn ( void ) {HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);}
void TXLedOff ( void ) {HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);}
GPIO_Output TXLed = { TXLedOn, TXLedOff };

void RXLedOn ( void ) {HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);}
void RXLedOff ( void ) {HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);}
GPIO_Output RXLed = { RXLedOn, RXLedOff };

void ExtraOn ( void ) {HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);}
void ExtraOff ( void ) {HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);}
GPIO_Output Extra = { ExtraOn, ExtraOff };

void USB_PU_On ( void ) {HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);}
void USB_PU_Off ( void ) {HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);}
GPIO_Output USB_PU = { USB_PU_On, USB_PU_Off };

uint_fast8_t ReadUSB_VBUS( void ){ return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);}

void InitGPIO(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : EXTRA_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PU_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}
