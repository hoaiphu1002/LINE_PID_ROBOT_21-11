/*
 * loadcell.h
 *
 *  Created on: Oct 13, 2025
 *      Author: TRƯƠNG VŨ HOÀI PHÚ
 */

#ifndef INC_LOADCELL_H_
#define INC_LOADCELL_H_

#include "stm32f1xx_hal.h"
#include "HX711.h"
#include <stdbool.h>

/* --- Cấu trúc chứa thông tin loadcell --- */
typedef struct {
    HX711_HandleTypeDef hx711;
    float scale_factor;
    int32_t zero_offset;
    float noise_threshold;
    float tolerance;
    float W1;
    float W2;
} LoadCell_HandleTypeDef;

/* --- API chính --- */
void  LoadCell_Init(LoadCell_HandleTypeDef *lc,
                    GPIO_TypeDef *dout_port, uint16_t dout_pin,
                    GPIO_TypeDef *sck_port, uint16_t sck_pin,
                    float scale_factor, int32_t zero_offset);

float LoadCell_ReadGram(LoadCell_HandleTypeDef *lc, uint8_t samples);
float LoadCell_ReadKg(LoadCell_HandleTypeDef *lc, uint8_t samples);
void  LoadCell_TareAuto(LoadCell_HandleTypeDef *lc);
void  LoadCell_Print(LoadCell_HandleTypeDef *lc);

#endif /* INC_LOADCELL_H_ */
