/*
 * loadcell.c
 *  Created on: Oct 13, 2025
 *  Author: TRƯƠNG VŨ HOÀI PHÚ
 */

#include "loadcell.h"
#include "hienthi.h"  // dùng print_uart()
#include <stdio.h>
#include <string.h>
#include <math.h>

/* --- CẤU HÌNH MẶC ĐỊNH --- */
#define NOISE_THRESHOLD_DEFAULT  0.2f
#define TOLERANCE_DEFAULT        6.0f
#define W1_DEFAULT               1000.0f
#define W2_DEFAULT               2000.0f

/* --- HÀM KHỞI TẠO --- */
void LoadCell_Init(LoadCell_HandleTypeDef *lc,
                   GPIO_TypeDef *dout_port, uint16_t dout_pin,
                   GPIO_TypeDef *sck_port, uint16_t sck_pin,
                   float scale_factor, int32_t zero_offset)
{
    HX711_Init(&lc->hx711, dout_port, dout_pin, sck_port, sck_pin);
    HX711_DWT_Init();

    lc->scale_factor   = scale_factor;
    lc->zero_offset    = zero_offset;
    lc->noise_threshold = NOISE_THRESHOLD_DEFAULT;
    lc->tolerance       = TOLERANCE_DEFAULT;
    lc->W1 = W1_DEFAULT;
    lc->W2 = W2_DEFAULT;

    HX711_SetScale(&lc->hx711, lc->scale_factor);
    HX711_SetOffset(&lc->hx711, lc->zero_offset);

    print_uart("\r\n=== 🧭 KHỞI ĐỘNG CÂN HX711 ===\r\n");
    char msg[64];
    float start_weight = HX711_GetUnits(&lc->hx711, 10);
    sprintf(msg, "Trọng lượng ban đầu: %.2f g\r\n", start_weight);
    print_uart(msg);

    if (fabsf(start_weight) > 1.0f) {
        print_uart("⚠️ Phát hiện lệch điểm 0 → tự động tare...\r\n");
        LoadCell_TareAuto(lc);
    }

    print_uart("✅ Cân sẵn sàng!\r\n\r\n");
}

/* --- TỰ ĐỘNG TARE --- */
void LoadCell_TareAuto(LoadCell_HandleTypeDef *lc)
{
    HX711_Tare(&lc->hx711, 10);
    lc->zero_offset = HX711_GetOffset(&lc->hx711);
}

/* --- ĐỌC GIÁ TRỊ TRUNG BÌNH (gram) --- */
float LoadCell_ReadGram(LoadCell_HandleTypeDef *lc, uint8_t samples)
{
    float weight = HX711_GetUnits(&lc->hx711, samples);

    // Lọc nhiễu: 0-10g → 0g
    if (weight >= 0 && weight <= 10.0f)
        weight = 0;

    // Âm → 0g
    if (weight < 0)
        weight = 0;

    // Khoảng W1 ± tolerance → W1
    if (weight >= (lc->W1 - lc->tolerance) && weight <= (lc->W1 + lc->tolerance))
        weight = lc->W1;

    // Khoảng W2 ± tolerance → W2
    else if (weight >= (lc->W2 - lc->tolerance) && weight <= (lc->W2 + lc->tolerance))
        weight = lc->W2;

    return weight;
}

/* --- ĐỌC GIÁ TRỊ (kg) --- */
float LoadCell_ReadKg(LoadCell_HandleTypeDef *lc, uint8_t samples)
{
    return LoadCell_ReadGram(lc, samples) / 1000.0f;
}

/* --- HÀM TỔNG HỢP: ĐỌC + IN RA UART --- */
void LoadCell_Print(LoadCell_HandleTypeDef *lc)
{
    float weight = LoadCell_ReadGram(lc, 2);

    char msg[64];
    sprintf(msg, "Trọng lượng: %.2f g\r\n", weight);
    print_uart(msg);
}
