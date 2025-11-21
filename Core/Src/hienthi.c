/*
 * hienthi.c
 *
 *  Created on: Oct 13, 2025
 *      Author: TRƯƠNG VŨ HOÀI PHÚ
 */
#include "hienthi.h"
void print_uart(const char *msg)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}
