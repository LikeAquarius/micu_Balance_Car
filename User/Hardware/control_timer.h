#ifndef __CONTROL_TIMER_H
#define __CONTROL_TIMER_H

#include <stdint.h>

void ControlTimer_Init(void);
uint16_t ControlTimer_GetLastExecutionUs(void);
uint16_t ControlTimer_GetMaxExecutionUs(void);
uint32_t ControlTimer_GetOverrunCount(void);

#endif
