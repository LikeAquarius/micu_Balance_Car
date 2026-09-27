#include "bsp_gpio.h"

void BSP_GPIO_EnableClock(GPIO_TypeDef *port)
{
    uint32_t clock = 0;

    if(port == GPIOA)      clock = RCC_APB2Periph_GPIOA;
    else if(port == GPIOB) clock = RCC_APB2Periph_GPIOB;
    else if(port == GPIOC) clock = RCC_APB2Periph_GPIOC;
    else if(port == GPIOD) clock = RCC_APB2Periph_GPIOD;

    if(clock != 0)
    {
        RCC_APB2PeriphClockCmd(clock, ENABLE);
    }
}

void BSP_GPIO_ConfigOutput(GPIO_TypeDef *port, uint16_t pin, GPIOMode_TypeDef mode)
{
    GPIO_InitTypeDef gpio;

    BSP_GPIO_EnableClock(port);
    gpio.GPIO_Pin = pin;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = mode;
    GPIO_Init(port, &gpio);
}

void BSP_GPIO_ConfigInput(GPIO_TypeDef *port, uint16_t pin, BSP_GPIO_InputMode mode)
{
    GPIO_InitTypeDef gpio;

    BSP_GPIO_EnableClock(port);
    gpio.GPIO_Pin = pin;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;

    switch(mode)
    {
        case BSP_GPIO_INPUT_PULL_UP:
            gpio.GPIO_Mode = GPIO_Mode_IPU;
            break;
        case BSP_GPIO_INPUT_PULL_DOWN:
            gpio.GPIO_Mode = GPIO_Mode_IPD;
            break;
        default:
            gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
            break;
    }

    GPIO_Init(port, &gpio);
}

void BSP_GPIO_Write(GPIO_TypeDef *port, uint16_t pin, bool high)
{
    if(high)
    {
        GPIO_SetBits(port, pin);
    }
    else
    {
        GPIO_ResetBits(port, pin);
    }
}

void BSP_GPIO_Toggle(GPIO_TypeDef *port, uint16_t pin)
{
    if(GPIO_ReadOutputDataBit(port, pin) == Bit_SET)
    {
        GPIO_ResetBits(port, pin);
    }
    else
    {
        GPIO_SetBits(port, pin);
    }
}

bool BSP_GPIO_Read(GPIO_TypeDef *port, uint16_t pin)
{
    return GPIO_ReadInputDataBit(port, pin) == Bit_SET;
}
