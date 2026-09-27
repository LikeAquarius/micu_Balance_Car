#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#include "ch32v20x.h"
#include <stdbool.h>

typedef enum
{
    BSP_GPIO_INPUT_FLOATING = 0,
    BSP_GPIO_INPUT_PULL_UP,
    BSP_GPIO_INPUT_PULL_DOWN
} BSP_GPIO_InputMode;

void BSP_GPIO_EnableClock(GPIO_TypeDef *port);
void BSP_GPIO_ConfigOutput(GPIO_TypeDef *port, uint16_t pin, GPIOMode_TypeDef mode);
void BSP_GPIO_ConfigInput(GPIO_TypeDef *port, uint16_t pin, BSP_GPIO_InputMode mode);
void BSP_GPIO_Write(GPIO_TypeDef *port, uint16_t pin, bool high);
void BSP_GPIO_Toggle(GPIO_TypeDef *port, uint16_t pin);
bool BSP_GPIO_Read(GPIO_TypeDef *port, uint16_t pin);

#endif
