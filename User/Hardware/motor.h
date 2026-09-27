#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdbool.h>
#include <stdint.h>

void Motor_Init(void);
void Motor_SetEnabled(bool enabled);
bool Motor_IsEnabled(void);
void Motor_SetPWM(int16_t left, int16_t right);
void Motor_EmergencyStop(void);

#endif
