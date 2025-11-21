/*
 * line_pid.h
 *
 *  Created on: Nov 20, 2025
 *      Author: TRƯƠNG VŨ HOÀI PHÚ
 */

#ifndef INC_LINE_PID_H_
#define INC_LINE_PID_H_


#include "main.h"

// PID parameters
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float last_error;
    int base_speed;
} Line_PID_Controller;

// Function prototypes
void Line_PID_Init(Line_PID_Controller* pid, float kp, float ki, float kd, int base_speed);
void Line_PID_Reset(Line_PID_Controller* pid);
int16_t Line_GetPosition(void);
void Line_PID_Update(Line_PID_Controller* pid);
void Line_NormalizeSensors(void);
void Line_Debug_PIDOutput(Line_PID_Controller* pid, int speedL, int speedR);


// External variables
extern uint16_t cam_raw[5];
extern uint16_t cam_norm[5];
extern uint16_t cam_min[5];
extern uint16_t cam_max[5];

#endif /* INC_LINE_PID_H_ */
