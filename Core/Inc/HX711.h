/*
 * HX711.h
 *
 *  Created on: Oct 13, 2025
 *      Author: TRƯƠNG VŨ HOÀI PHÚ
 */

#ifndef INC_HX711_H_
#define INC_HX711_H_

#include "stm32f1xx_hal.h"  // hoặc header HAL phù hợp cho MCU của bạn
#include <stdint.h>
#include <stdbool.h>

typedef struct {
  GPIO_TypeDef *data_port;
  uint16_t      data_pin;
  GPIO_TypeDef *sck_port;
  uint16_t      sck_pin;

  int32_t  offset;    // zero offset (raw)
  float    scale;     // raw per gram (or raw per unit)
  uint8_t  gain;      // 128,64,32
} HX711_HandleTypeDef;

/* init - khai báo chân (DATA, SCK) */
void HX711_Init(HX711_HandleTypeDef *h, GPIO_TypeDef *data_port, uint16_t data_pin,
                GPIO_TypeDef *sck_port, uint16_t sck_pin);

/* DWT init để dùng delay microsecond (gọi 1 lần ở startup) */
void HX711_DWT_Init(void);

/* low level read raw 24-bit (blocking until ready) */
int32_t HX711_ReadRaw(HX711_HandleTypeDef *h);

/* read average of n samples */
int32_t HX711_ReadAverage(HX711_HandleTypeDef *h, uint8_t times);

/* tare: set offset as average of `times` samples */
void HX711_Tare(HX711_HandleTypeDef *h, uint8_t times);

/* set/get scale and offset */
void HX711_SetScale(HX711_HandleTypeDef *h, float scale);
float HX711_GetScale(HX711_HandleTypeDef *h);
void HX711_SetOffset(HX711_HandleTypeDef *h, int32_t offset);
int32_t HX711_GetOffset(HX711_HandleTypeDef *h);

/* get units (converted weight) */
float HX711_GetUnits(HX711_HandleTypeDef *h, uint8_t times);

/* power management */
void HX711_PowerDown(HX711_HandleTypeDef *h);
void HX711_PowerUp(HX711_HandleTypeDef *h);

/* gain: 128, 64, 32 (returns true if valid) */
bool HX711_SetGain(HX711_HandleTypeDef *h, uint8_t gain);

#endif /* INC_HX711_H_ */
