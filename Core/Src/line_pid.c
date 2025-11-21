///*
// * line_pid.c
// *
// *  Created on: Nov 20, 2025
// *      Author: TRƯƠNG VŨ HOÀI PHÚ
// */
//
//#include "line_pid.h"
//#include "control_motor.h"
//#include <stdio.h>
//
//// Sensor variables
//uint16_t cam_raw[5];
//uint16_t cam_norm[5];
//uint16_t cam_min[5] = {177,158,161,162,152};
//uint16_t cam_max[5] = {4095,4095,4095,4095,4095};
//
//// External ADC values
//extern volatile uint16_t cam0, cam1, cam2, cam3, cam4;
//
///*--------------------------------------------------------------
// * 1. KHỞI TẠO PID
// *-------------------------------------------------------------*/
//void Line_PID_Init(Line_PID_Controller* pid, float kp, float ki, float kd, int base_speed)
//{
//    pid->Kp = kp;
//    pid->Ki = ki;
//    pid->Kd = kd;
//    pid->integral = 0;
//    pid->last_error = 0;
//    pid->base_speed = base_speed;
//}
//
//void Line_PID_Reset(Line_PID_Controller* pid)
//{
//    pid->integral = 0;
//    pid->last_error = 0;
//}
//
///*--------------------------------------------------------------
// * 2. CHUẨN HÓA CẢM BIẾN
// *-------------------------------------------------------------*/
//void Line_NormalizeSensors(void)
//{
//    cam_raw[0] = cam0;
//    cam_raw[1] = cam1;
//    cam_raw[2] = cam2;
//    cam_raw[3] = cam3;
//    cam_raw[4] = cam4;
//
//    for(int i = 0; i < 5; i++)
//    {
//        int32_t val = cam_raw[i];
//        if(val < cam_min[i]) val = cam_min[i];
//        if(val > cam_max[i]) val = cam_max[i];
//        cam_norm[i] = (val - cam_min[i]) * 1000 / (cam_max[i] - cam_min[i]);
//    }
//}
//
///*--------------------------------------------------------------
// * 3. TÍNH VỊ TRÍ LINE
// *-------------------------------------------------------------*/
//int16_t Line_GetPosition(void)
//{
//    int32_t sum_val = 0;
//    int32_t sum_weight = 0;
//    int16_t weight[5] = {-260, -130, 0, 130, 260};
//
//    for(int i = 0; i < 5; i++)
//    {
//        sum_val += cam_norm[i] * weight[i];
//        sum_weight += cam_norm[i];
//    }
//
//    if(sum_weight == 0)
//        return 0;
//
//    return sum_val / sum_weight;
//}
//
///*--------------------------------------------------------------
// * 4. CẬP NHẬT PID LINE
// *-------------------------------------------------------------*/
//void Line_PID_Update(Line_PID_Controller* pid)
//{
//    Line_NormalizeSensors();
//    float error = Line_GetPosition();
//
//    pid->integral += error;
//    float derivative = error - pid->last_error;
//
//    float correction = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
//
//    int speedL = pid->base_speed - correction;
//    int speedR = pid->base_speed + correction;
//
//    // Giới hạn tốc độ
//    if(speedL < 0) speedL = 0;
//    if(speedL > 999) speedL = 999;
//    if(speedR < 0) speedR = 0;
//    if(speedR > 999) speedR = 999;
//
//    // Chuyển đổi duty cycle (0-999) sang phần trăm (0-100)
//    float duty_left = (float)speedL / 999.0f * 100.0f;
//    float duty_right = (float)speedR / 999.0f * 100.0f;
//
//    Motor_SetDuty(duty_left, duty_right);
//    pid->last_error = error;
//}
