/*
 * HX711.c
 *
 *  Created on: Oct 13, 2025
 *      Author: TRƯƠNG VŨ HOÀI PHÚ
 */
#include "HX711.h"

/* Microsecond delay using DWT cycle counter */
static inline void HX711_Delay_us(uint32_t us)
{
#ifdef DWT
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000U);
  while ((DWT->CYCCNT - start) < ticks) { __NOP(); }
#else
  // Fallback (less accurate). Calibrate loop count if necessary.
  uint32_t i;
  volatile uint32_t count = (uint32_t)(us * 8); // very rough for 72MHz
  for (i = 0; i < count; i++) { __NOP(); }
#endif
}

/* Initialize DWT counter for microsecond delay - call once in main */
void HX711_DWT_Init(void)
{
  // Enable TRC
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  // Reset the cycle counter
  DWT->CYCCNT = 0;
  // Enable the cycle counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* Configure handle */
void HX711_Init(HX711_HandleTypeDef *h, GPIO_TypeDef *data_port, uint16_t data_pin,
                GPIO_TypeDef *sck_port, uint16_t sck_pin)
{
  h->data_port = data_port;
  h->data_pin  = data_pin;
  h->sck_port  = sck_port;
  h->sck_pin   = sck_pin;

  h->offset = 0;
  h->scale  = 1.0f;
  h->gain   = 128; // default

  // Ensure SCK low
  HAL_GPIO_WritePin(h->sck_port, h->sck_pin, GPIO_PIN_RESET);
}

/* Set gain: accepted 128,64,32 */
bool HX711_SetGain(HX711_HandleTypeDef *h, uint8_t gain)
{
  if (gain != 128 && gain != 64 && gain != 32) return false;
  h->gain = gain;
  // After changing gain, one read cycle required by HX711.
  // We leave it to user - next read will pulse correct number.
  return true;
}

/* Low level read - blocking until data ready */
int32_t HX711_ReadRaw(HX711_HandleTypeDef *h)
{
  // Wait for DRDY -> DATA pin goes low
  uint32_t timeout = HAL_GetTick() + 1000;
  while (HAL_GPIO_ReadPin(h->data_port, h->data_pin) == GPIO_PIN_SET) {
    if (HAL_GetTick() > timeout) return 0; // timeout - return 0
  }

  uint32_t count = 0;
  for (uint8_t i = 0; i < 24; i++) {
    // clock high
    HAL_GPIO_WritePin(h->sck_port, h->sck_pin, GPIO_PIN_SET);
    HX711_Delay_us(1);
    count <<= 1;
    // clock low
    HAL_GPIO_WritePin(h->sck_port, h->sck_pin, GPIO_PIN_RESET);
    HX711_Delay_us(1);

    if (HAL_GPIO_ReadPin(h->data_port, h->data_pin) == GPIO_PIN_SET) {
      count++;
    }
  }

  // set gain/channel by sending extra pulses:
  uint8_t pulses = 1; // for gain 128
  if (h->gain == 128) pulses = 1;
  else if (h->gain == 64) pulses = 3;
  else if (h->gain == 32) pulses = 2;

  for (uint8_t p = 0; p < pulses; p++) {
    HAL_GPIO_WritePin(h->sck_port, h->sck_pin, GPIO_PIN_SET);
    HX711_Delay_us(1);
    HAL_GPIO_WritePin(h->sck_port, h->sck_pin, GPIO_PIN_RESET);
    HX711_Delay_us(1);
  }

  // Convert from unsigned 24 bit to signed 32 bit
  if (count & 0x800000) { // negative
    count |= 0xFF000000;
  } else {
    count &= 0x00FFFFFF;
  }

  return (int32_t)count;
}

int32_t HX711_ReadAverage(HX711_HandleTypeDef *h, uint8_t times)
{
  if (times == 0) times = 1;
  int64_t sum = 0;
  for (uint8_t i = 0; i < times; i++) {
    sum += HX711_ReadRaw(h);
    HAL_Delay(0); // yield
  }
  return (int32_t)(sum / times);
}

void HX711_Tare(HX711_HandleTypeDef *h, uint8_t times)
{
  int32_t avg = HX711_ReadAverage(h, times);
  h->offset = avg;
}

void HX711_SetScale(HX711_HandleTypeDef *h, float scale)
{
  if (scale == 0.0f) return;
  h->scale = scale;
}

float HX711_GetScale(HX711_HandleTypeDef *h)
{
  return h->scale;
}

void HX711_SetOffset(HX711_HandleTypeDef *h, int32_t offset)
{
  h->offset = offset;
}

int32_t HX711_GetOffset(HX711_HandleTypeDef *h)
{
  return h->offset;
}

float HX711_GetUnits(HX711_HandleTypeDef *h, uint8_t times)
{
  int32_t raw = HX711_ReadAverage(h, times);
  float units = ((float)raw - (float)h->offset) / h->scale;
  return units;
}

void HX711_PowerDown(HX711_HandleTypeDef *h)
{
  // set SCK high for >60us to enter power down
  HAL_GPIO_WritePin(h->sck_port, h->sck_pin, GPIO_PIN_RESET);
  HX711_Delay_us(1);
  HAL_GPIO_WritePin(h->sck_port, h->sck_pin, GPIO_PIN_SET);
  HX711_Delay_us(100);
}

void HX711_PowerUp(HX711_HandleTypeDef *h)
{
  // set SCK low to wake up
  HAL_GPIO_WritePin(h->sck_port, h->sck_pin, GPIO_PIN_RESET);
  HX711_Delay_us(1);
}


