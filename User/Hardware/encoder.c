#include "encoder.h"
#include "balance_config.h"
#include "ch32v20x.h"

static void encoder_timer_init(TIM_TypeDef *timer)
{
    TIM_TimeBaseInitTypeDef base = {0};

    base.TIM_Period = 0xFFFFU;
    base.TIM_Prescaler = 0U;
    base.TIM_ClockDivision = TIM_CKD_DIV1;
    base.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(timer, &base);
    TIM_EncoderInterfaceConfig(timer, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_SetCounter(timer, 0U);
    TIM_Cmd(timer, ENABLE);
}

void Encoder_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* Schematic: right encoder PA0/PA1 -> TIM2, left PA6/PA7 -> TIM3. */
    encoder_timer_init(TIM2);
    encoder_timer_init(TIM3);
}

void Encoder_Reset(void)
{
    TIM_SetCounter(TIM2, 0U);
    TIM_SetCounter(TIM3, 0U);
}

int16_t Encoder_ReadLeft(void)
{
    int16_t count = (int16_t)TIM_GetCounter(TIM3);
    TIM_SetCounter(TIM3, 0U);
    return (int16_t)(count * BALANCE_LEFT_ENCODER_SIGN);
}

int16_t Encoder_ReadRight(void)
{
    int16_t count = (int16_t)TIM_GetCounter(TIM2);
    TIM_SetCounter(TIM2, 0U);
    return (int16_t)(count * BALANCE_RIGHT_ENCODER_SIGN);
}
