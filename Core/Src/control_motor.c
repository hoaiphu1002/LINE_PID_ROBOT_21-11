///*
// * control_motor.c
// *
// *  Created on: Oct 22, 2025
// *      Author: TRƯƠNG VŨ HOÀI PHÚ
// *
// *  Sửa:
// *  - Thống nhất đơn vị (m) cho chu vi, RPM->m/s, m/s->duty.
// *  - Giữ Motor_ReadSpeed() dựa trên CONTROL_PERIOD_S = 0.01s (10 ms).
// *  - Đã đánh dấu các chỗ FIXED.
// */
//
//#include "control_motor.h"
//#include <math.h>
//#include "stdio.h"
//#include "hienthi.h"
//
///*--------------------------------------------------------------
// * 1. CÁC THÔNG SỐ CƠ BẢN (FIXED: thống nhất đơn vị)
// *-------------------------------------------------------------*/
//#define ENCODER_PPR        1320.0f     // (330 xung * 4 do chế độ TI12)
//#define CONTROL_PERIOD_S   0.01f       // Thời gian cập nhật tốc độ: 10ms (s)
//
//#define WHEEL_DIAMETER_M   0.065f      // Đường kính bánh xe: 65 mm = 0.065 m
//#define WHEEL_CIRC_M       (M_PI * WHEEL_DIAMETER_M)  // Chu vi bánh xe (m)
//
//#define PWM_MAX            100.0f      // PWM duty tối đa (100%)
//
//// Thông số motor
//#define MOTOR_RPM_MAX      333.0f      // rpm (theo đặc tính motor)
//#define MOTOR_SPEED_MAX    (WHEEL_CIRC_M * MOTOR_RPM_MAX / 60.0f)  // m/s
//
//// FIXED: chuyển RPM -> m/s dùng mét, công thức chuẩn
//#define RPM_TO_MS(rpm)     (WHEEL_CIRC_M * (rpm) / 60.0f)
//
//// FIXED: đổi m/s -> duty (%) dựa trên tốc độ cực đại MOTOR_SPEED_MAX
//#define SPEED_TO_DUTY(v)   (((v) / MOTOR_SPEED_MAX) * 100.0f)
//
///*--------------------------------------------------------------
// * 2. CHÂN ĐIỀU KHIỂN TB6612 (THIẾT LẬP CHIỀU)
// *-------------------------------------------------------------*/
//
//#define LEFT_IN1_GPIO_Port  GPIOB
//#define LEFT_IN1_Pin        GPIO_PIN_0
//#define LEFT_IN2_GPIO_Port  GPIOB
//#define LEFT_IN2_Pin        GPIO_PIN_1
//
//#define RIGHT_IN1_GPIO_Port GPIOA
//#define RIGHT_IN1_Pin       GPIO_PIN_5
//#define RIGHT_IN2_GPIO_Port GPIOA
//#define RIGHT_IN2_Pin       GPIO_PIN_6
//
///*--------------------------------------------------------------
// * 3. CẤU HÌNH NGƯỢC
// *-------------------------------------------------------------*/
//#define LEFT_INVERTED  0
//#define RIGHT_INVERTED 0
//
///*--------------------------------------------------------------
// * 4. BIẾN TOÀN CỤC LƯU TRẠNG THÁI ENCODER & TỐC ĐỘ
// *-------------------------------------------------------------*/
//static int32_t enc_left_prev = 0;
//static int32_t enc_right_prev = 0;
//static float rpm_left = 0.0f;
//static float rpm_right = 0.0f;
//
///*--------------------------------------------------------------
// * 5. KHAI BÁO EXTERN (handlers từ main.c)
// *-------------------------------------------------------------*/
//extern TIM_HandleTypeDef htim1;
//extern TIM_HandleTypeDef htim2;
//extern TIM_HandleTypeDef htim3;
//extern TIM_HandleTypeDef htim4;
//
///*--------------------------------------------------------------
// * 6. HÀM KHỞI TẠO
// *-------------------------------------------------------------*/
//void Motor_Init(void)
//{
//    /* Khởi động PWM */
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);   // PWM right
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   // PWM left
//    __HAL_TIM_MOE_ENABLE(&htim1);
//
//    /* Khởi động encoder */
//    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // Encoder trái
//    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // Encoder phải
//
//    /* Ghi nhận giá trị ban đầu */
//    enc_left_prev  = __HAL_TIM_GET_COUNTER(&htim2);
//    enc_right_prev = __HAL_TIM_GET_COUNTER(&htim4);
//
//    /* Dừng motor ban đầu */
//    Motor_SetDuty(0, 0);
//    Motor_SetDirectionLeft(DIR_COAST);
//    Motor_SetDirectionRight(DIR_COAST);
//}
//
///*--------------------------------------------------------------
// * 7. SET DIRECTION (unchanged)
// *-------------------------------------------------------------*/
//void Motor_SetDirectionLeft(uint8_t dir)
//{
//    switch (dir)
//    {
//        case DIR_FORWARD:
//            HAL_GPIO_WritePin(LEFT_IN1_GPIO_Port, LEFT_IN1_Pin, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(LEFT_IN2_GPIO_Port, LEFT_IN2_Pin, GPIO_PIN_SET);
//            break;
//        case DIR_BACKWARD:
//            HAL_GPIO_WritePin(LEFT_IN1_GPIO_Port, LEFT_IN1_Pin, GPIO_PIN_SET);
//            HAL_GPIO_WritePin(LEFT_IN2_GPIO_Port, LEFT_IN2_Pin, GPIO_PIN_RESET);
//            break;
//        case DIR_COAST:
//        default:
//            HAL_GPIO_WritePin(LEFT_IN1_GPIO_Port, LEFT_IN1_Pin, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(LEFT_IN2_GPIO_Port, LEFT_IN2_Pin, GPIO_PIN_RESET);
//            break;
//    }
//}
//
//void Motor_SetDirectionRight(uint8_t dir)
//{
//    switch (dir)
//    {
//        case DIR_FORWARD:
//            HAL_GPIO_WritePin(RIGHT_IN1_GPIO_Port, RIGHT_IN1_Pin, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(RIGHT_IN2_GPIO_Port, RIGHT_IN2_Pin, GPIO_PIN_SET);
//            break;
//        case DIR_BACKWARD:
//            HAL_GPIO_WritePin(RIGHT_IN1_GPIO_Port, RIGHT_IN1_Pin, GPIO_PIN_SET);
//            HAL_GPIO_WritePin(RIGHT_IN2_GPIO_Port, RIGHT_IN2_Pin, GPIO_PIN_RESET);
//            break;
//        case DIR_COAST:
//        default:
//            HAL_GPIO_WritePin(RIGHT_IN1_GPIO_Port, RIGHT_IN1_Pin, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(RIGHT_IN2_GPIO_Port, RIGHT_IN2_Pin, GPIO_PIN_RESET);
//            break;
//    }
//}
//
///*--------------------------------------------------------------
// * 8. SET DUTY (mapping TIM1 CH1 = left, TIM1 CH4 = right)
// *-------------------------------------------------------------*/
//void Motor_SetDuty(float left_percent, float right_percent)
//{
//    if (left_percent < 0) left_percent = 0;
//    if (left_percent > 100) left_percent = 100;
//    if (right_percent < 0) right_percent = 0;
//    if (right_percent > 100) right_percent = 100;
//
//    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
//
//    uint32_t ccr_left  = (uint32_t)((left_percent  / 100.0f) * (arr + 1));
//    uint32_t ccr_right = (uint32_t)((right_percent / 100.0f) * (arr + 1));
//
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_left);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ccr_right);
//}
//
///*--------------------------------------------------------------
// * 9. ĐỌC ENCODER & TÍNH RPM
// *    Ghi chú: Motor_ReadSpeed() được thiết kế để gọi mỗi CONTROL_PERIOD_S (0.01s).
// *-------------------------------------------------------------*/
//void Motor_ReadSpeed(void)
//{
//    int32_t enc_left_now  = __HAL_TIM_GET_COUNTER(&htim2);
//    int32_t enc_right_now = __HAL_TIM_GET_COUNTER(&htim4);
//
//    int32_t delta_left  = enc_left_now - enc_left_prev;
//    if (delta_left > 32767)  delta_left -= 65536;
//    if (delta_left < -32768) delta_left += 65536;
//
//    int32_t delta_right = enc_right_now - enc_right_prev;
//    if (delta_right > 32767)  delta_right -= 65536;
//    if (delta_right < -32768) delta_right += 65536;
//
//    enc_left_prev  = enc_left_now;
//    enc_right_prev = enc_right_now;
//
//    // số vòng trong khoảng CONTROL_PERIOD_S
//    float rev_per_tick_left  = 1.0f / ENCODER_PPR;
//    float rev_per_tick_right = 1.0f / ENCODER_PPR;
//
//    float rps_left  = (delta_left  * rev_per_tick_left)  / CONTROL_PERIOD_S;
//    float rps_right = (delta_right * rev_per_tick_right) / CONTROL_PERIOD_S;
//
//    rpm_left  = rps_left  * 60.0f;
//    rpm_right = rps_right * 60.0f;
//}
//
///*--------------------------------------------------------------
// * 10. LẤY RPM (không can thiệp dấu) & VẬN TỐC (m/s)
// *-------------------------------------------------------------*/
//float Motor_GetLeftRPM(void)  {
//	return rpm_left;
//}
//float Motor_GetRightRPM(void) {
//	return rpm_right;
//}
//
//float Motor_GetSpeedLeftMS(void)  {
//	return -RPM_TO_MS(Motor_GetLeftRPM());
//}
//
//float Motor_GetSpeedRightMS(void) {
//	return RPM_TO_MS(Motor_GetRightRPM());
//}
//float Motor_GetSpeedAverageMS(void) {
//	return (Motor_GetSpeedLeftMS() + Motor_GetSpeedRightMS()) / 2.0f;
//}
//
///*--------------------------------------------------------------
// * 11. Motor_SetSpeedMps nhận cả âm/dương: set direction + duty tự động
// *-------------------------------------------------------------*/
//void Motor_SetSpeedMps(float left_speed_mps, float right_speed_mps)
//{
//    float phys_left = (LEFT_INVERTED)  ? -left_speed_mps  : left_speed_mps;
//    float phys_right= (RIGHT_INVERTED) ? -right_speed_mps : right_speed_mps;
//
//    if (phys_left > 0.0f) {
//        Motor_SetDirectionLeft(DIR_FORWARD);
//    } else if (phys_left < 0.0f) {
//        Motor_SetDirectionLeft(DIR_BACKWARD);
//    } else {
//        Motor_SetDirectionLeft(DIR_COAST);
//    }
//
//    if (phys_right > 0.0f) {
//        Motor_SetDirectionRight(DIR_FORWARD);
//    } else if (phys_right < 0.0f) {
//        Motor_SetDirectionRight(DIR_BACKWARD);
//    } else {
//        Motor_SetDirectionRight(DIR_COAST);
//    }
//
//    float duty_left  = SPEED_TO_DUTY(fabsf(phys_left));
//    float duty_right = SPEED_TO_DUTY(fabsf(phys_right));
//
//    if (duty_left  > PWM_MAX) duty_left  = PWM_MAX;
//    if (duty_right > PWM_MAX) duty_right = PWM_MAX;
//
//    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
//    uint32_t ccr_left  = (uint32_t)((duty_left  / 100.0f) * (arr + 1));
//    uint32_t ccr_right = (uint32_t)((duty_right / 100.0f) * (arr + 1));
//
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_left);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ccr_right);
//}
//
///*--------------------------------------------------------------
// * 12. Debug nâng cao
// *-------------------------------------------------------------*/
//void Motor_Debug_CheckPWM(void)
//{
//    // đọc CCR và ARR (TIM1 PWM)
//    uint32_t ccr1 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);
//    uint32_t ccr4 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_4);
//    uint32_t arr  = __HAL_TIM_GET_AUTORELOAD(&htim1);
//    float duty_l_percent = (float)ccr1 / (arr + 1) * 100.0f;
//    float duty_r_percent = (float)ccr4 / (arr + 1) * 100.0f;
//
//    // đọc counter encoder hiện tại để quan sát
//    int32_t enc_left_now  = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
//    int32_t enc_right_now = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
//
//    // direction pins
//    GPIO_PinState L1 = HAL_GPIO_ReadPin(LEFT_IN1_GPIO_Port, LEFT_IN1_Pin);
//    GPIO_PinState L2 = HAL_GPIO_ReadPin(LEFT_IN2_GPIO_Port, LEFT_IN2_Pin);
//    GPIO_PinState R1 = HAL_GPIO_ReadPin(RIGHT_IN1_GPIO_Port, RIGHT_IN1_Pin);
//    GPIO_PinState R2 = HAL_GPIO_ReadPin(RIGHT_IN2_GPIO_Port, RIGHT_IN2_Pin);
//
//    // RPM & speed (đã cập nhật bởi Motor_ReadSpeed trước khi gọi hàm debug)
//    float left_rpm  = rpm_left;
//    float right_rpm = rpm_right;
//    float v_left = RPM_TO_MS(left_rpm);
//    float v_right= RPM_TO_MS(right_rpm);
//
//    // In ngắn gọn và đủ thông tin để chẩn đoán
//    char msg[256];
//    sprintf(msg,
//        "ENC_L=%ld ENC_R=%ld | CCR1=%lu CCR4=%lu | dutyL=%.1f%% dutyR=%.1f%%\r\n"
//        "RPM_L=%+.2f RPM_R=%+.2f | V_L=%+.3f m/s V_R=%+.3f m/s\r\n"
//        "DIR_L: IN1=%d IN2=%d | DIR_R: IN1=%d IN2=%d\r\n",
//        (long)enc_left_now, (long)enc_right_now,
//        ccr1, ccr4, duty_l_percent, duty_r_percent,
//        left_rpm, right_rpm, v_left, v_right,
//        (int)L1, (int)L2, (int)R1, (int)R2);
//
//    print_uart(msg);
//}
