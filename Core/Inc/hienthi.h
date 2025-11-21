/*
 * hienthi.h
 *
 *  Created on: Oct 13, 2025
 *      Author: TRƯƠNG VŨ HOÀI PHÚ
 */

#ifndef INC_HIENTHI_H_
#define INC_HIENTHI_H_


#include "stm32f1xx_hal.h"
#include <string.h>

extern UART_HandleTypeDef huart1;

void print_uart(const char *msg);

#endif /* INC_HIENTHI_H_ */
