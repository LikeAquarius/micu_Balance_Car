#include "motor.h"
#include "ch32v20x.h"

#define MOTOR_COMMAND_MAX    1000
#define MOTOR_DEAD_ZONE      100U
#define MOTOR_TIMER_PERIOD   4799U

static bool motor_enabled;

static uint16_t command_to_compare(int16_t command)
{
    uint32_t magnitude = command < 0 ? (uint32_t)(-command) : (uint32_t)command;
    uint32_t compensated;

    if(magnitude == 0U) return 0U;
    if(magnitude > MOTOR_COMMAND_MAX) magnitude = MOTOR_COMMAND_MAX;
    compensated = MOTOR_DEAD_ZONE +
                  ((uint32_t)MOTOR_COMMAND_MAX - MOTOR_DEAD_ZONE) * magnitude /
                  MOTOR_COMMAND_MAX;
    return (uint16_t)(compensated * (MOTOR_TIMER_PERIOD + 1U) /
                      MOTOR_COMMAND_MAX);
}

static void left_stop(void)
{
    GPIO_ResetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_14);
    TIM_SetCompare1(TIM1, 0U);
}

static void right_stop(void)
{
    GPIO_ResetBits(GPIOA, GPIO_Pin_10 | GPIO_Pin_11);
    TIM_SetCompare2(TIM1, 0U);
}

void Motor_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_TimeBaseInitTypeDef timer = {0};
    TIM_OCInitTypeDef output = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_TIM1, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &gpio);

    GPIO_ResetBits(GPIOA, GPIO_Pin_10 | GPIO_Pin_11);
    GPIO_ResetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);

    timer.TIM_Period = MOTOR_TIMER_PERIOD;
    timer.TIM_Prescaler = 0U;
    timer.TIM_ClockDivision = TIM_CKD_DIV1;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    timer.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(TIM1, &timer);

    output.TIM_OCMode = TIM_OCMode_PWM1;
    output.TIM_OutputState = TIM_OutputState_Enable;
    output.TIM_OutputNState = TIM_OutputNState_Disable;
    output.TIM_Pulse = 0U;
    output.TIM_OCPolarity = TIM_OCPolarity_High;
    output.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OC1Init(TIM1, &output);
    TIM_OC2Init(TIM1, &output);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
    motor_enabled = false;
}

void Motor_SetEnabled(bool enabled)
{
    if(!enabled)
    {
        left_stop();
        right_stop();
        GPIO_ResetBits(GPIOB, GPIO_Pin_15);
        motor_enabled = false;
        return;
    }

    left_stop();
    right_stop();
    GPIO_SetBits(GPIOB, GPIO_Pin_15);
    motor_enabled = true;
}

bool Motor_IsEnabled(void)
{
    return motor_enabled;
}

void Motor_SetPWM(int16_t left, int16_t right)
{
    if(!motor_enabled)
    {
        left_stop();
        right_stop();
        return;
    }

    if(left > MOTOR_COMMAND_MAX) left = MOTOR_COMMAND_MAX;
    if(left < -MOTOR_COMMAND_MAX) left = -MOTOR_COMMAND_MAX;
    if(right > MOTOR_COMMAND_MAX) right = MOTOR_COMMAND_MAX;
    if(right < -MOTOR_COMMAND_MAX) right = -MOTOR_COMMAND_MAX;

    if(left > 0)
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_13);
        GPIO_SetBits(GPIOB, GPIO_Pin_14);
        TIM_SetCompare1(TIM1, command_to_compare(left));
    }
    else if(left < 0)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_13);
        GPIO_ResetBits(GPIOB, GPIO_Pin_14);
        TIM_SetCompare1(TIM1, command_to_compare(left));
    }
    else left_stop();

    if(right > 0)
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_11);
        GPIO_ResetBits(GPIOA, GPIO_Pin_10);
        TIM_SetCompare2(TIM1, command_to_compare(right));
    }
    else if(right < 0)
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_11);
        GPIO_SetBits(GPIOA, GPIO_Pin_10);
        TIM_SetCompare2(TIM1, command_to_compare(right));
    }
    else right_stop();
}

void Motor_EmergencyStop(void)
{
    Motor_SetEnabled(false);
}
