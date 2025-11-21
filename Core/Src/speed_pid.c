/*
 * speed_pid.c
 *
 *  Created on: Nov 20, 2025
 *      Author: TRƯƠNG VŨ HOÀI PHÚ
 */
#include "speed_pid.h"
#include "control_motor.h"
#include <stdio.h>

/*--------------------------------------------------------------
 * 1. KHỞI TẠO PID TỐC ĐỘ
 *-------------------------------------------------------------*/
void Speed_PID_Init(Speed_PID_Controller* pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral = 0;
    pid->last_error = 0;
    pid->setpoint = 0;
    pid->output = 0;
}

void Speed_PID_Reset(Speed_PID_Controller* pid)
{
    pid->integral = 0;
    pid->last_error = 0;
    pid->output = 0;
}

/*--------------------------------------------------------------
 * 2. SET SETPOINT (m/s)
 *-------------------------------------------------------------*/
void Speed_PID_SetSetpoint(Speed_PID_Controller* pid, float setpoint)
{
    pid->setpoint = setpoint;
}

/*--------------------------------------------------------------
 * 3. CẬP NHẬT PID TỐC ĐỘ
 *-------------------------------------------------------------*/
/*--------------------------------------------------------------
 * 3. CẬP NHẬT PID TỐC ĐỘ (SỬA THÀNH PWM OUTPUT)
 *-------------------------------------------------------------*/
float Speed_PID_Update(Speed_PID_Controller* pid, float current_speed)
{
    float error = pid->setpoint - current_speed;

    pid->integral += error;
    // Giới hạn integral để tránh windup
    if(pid->integral > 100.0f) pid->integral = 100.0f;
    if(pid->integral < -100.0f) pid->integral = -100.0f;

    float derivative = error - pid->last_error;

    // Output là PWM % (-100% đến +100%)
    pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    // Giới hạn output PWM
    if(pid->output > 100.0f) pid->output = 100.0f;
    if(pid->output <0.0f) pid->output = 0.0f;

    pid->last_error = error;

    return pid->output;
}

/*--------------------------------------------------------------
 * 4. DEBUG SPEED PID UPDATE
 *-------------------------------------------------------------*/
void Speed_Debug_PIDUpdate(Speed_PID_Controller* pid, float current_speed, float output, const char* motor_side)
{
    float error = pid->setpoint - current_speed;

    char msg[256];
    sprintf(msg, "SPEED_PID %s: Set=%.3f Cur=%.3f Err=%.3f | Out=%.3f PWM=%.1f%%\r\n",
            motor_side, pid->setpoint, current_speed, error, output,
            (output / pid->setpoint) * 100.0f); // Ước lượng % PWM
    print_uart(msg);
}
