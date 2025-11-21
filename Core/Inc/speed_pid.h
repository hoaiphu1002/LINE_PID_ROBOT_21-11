/*
 * speed_pid.h
 *
 *  Created on: Nov 20, 2025
 *      Author: TRƯƠNG VŨ HOÀI PHÚ
 */

#ifndef INC_SPEED_PID_H_
#define INC_SPEED_PID_H_

#include "main.h"

// Speed PID controller
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float last_error;
    float setpoint;
    float output;
} Speed_PID_Controller;

// Function prototypes
void Speed_PID_Init(Speed_PID_Controller* pid, float kp, float ki, float kd);
void Speed_PID_Reset(Speed_PID_Controller* pid);
void Speed_PID_SetSetpoint(Speed_PID_Controller* pid, float setpoint);
float Speed_PID_Update(Speed_PID_Controller* pid, float current_speed);
void Speed_Debug_PIDUpdate(Speed_PID_Controller* pid, float current_speed, float output, const char* motor_side);

#endif /* INC_SPEED_PID_H_ */
