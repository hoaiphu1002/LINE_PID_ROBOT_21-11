/*
 * control_motor.h
 *
 *  Created on: Oct 22, 2025
 *      Author: TRƯƠNG VŨ HOÀI PHÚ
 */

#ifndef INC_CONTROL_MOTOR_H_
#define INC_CONTROL_MOTOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* --- External handles --- */
extern TIM_HandleTypeDef htim1;   // PWM Left
extern TIM_HandleTypeDef htim2;   // Encoder Right
extern TIM_HandleTypeDef htim3;   // PWM Right
extern TIM_HandleTypeDef htim4;   // Encoder Left

/* --- Public APIs --- */
void Motor_Init(void);
void Motor_SetDuty(float left_percent, float right_percent);
void Motor_SetDirectionLeft(uint8_t dir);
void Motor_SetDirectionRight(uint8_t dir);
void Motor_ReadSpeed(void);           // gọi mỗi 10ms để tính RPM
float Motor_GetLeftRPM(void);
float Motor_GetRightRPM(void);
float Motor_GetSpeedLeftMS(void);
float Motor_GetSpeedRightMS(void);
float Motor_GetSpeedAverageMS(void);
void Motor_SetSpeedMps(float left_speed_mps, float right_speed_mps);
void Motor_Debug_CheckPWM(void);



/* --- Hướng quay --- */
#define DIR_FORWARD   0   // quay thuận
#define DIR_BACKWARD  1   // quay nghịch
#define DIR_BRAKE     2   // phanh (2 chân HIGH)
#define DIR_COAST     3   // trôi tự do (2 chân LOW)

#ifdef __cplusplus
}
#endif

#endif /* INC_CONTROL_MOTOR_H_ */
