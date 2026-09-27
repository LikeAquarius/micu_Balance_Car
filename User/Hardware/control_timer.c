#include "control_timer.h"
#include "balance.h"
#include "ch32v20x.h"

#define CONTROL_TIMER_PERIOD_US 5000U

static volatile uint16_t last_execution_us;
static volatile uint16_t max_execution_us;
static volatile uint32_t overrun_count;

void TIM4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void ControlTimer_Init(void)
{
    TIM_TimeBaseInitTypeDef timer = {0};
    NVIC_InitTypeDef nvic = {0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    timer.TIM_Period = 5000U - 1U;
    timer.TIM_Prescaler = 96U - 1U;
    timer.TIM_ClockDivision = TIM_CKD_DIV1;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &timer);
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    nvic.NVIC_IRQChannel = TIM4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0U;
    nvic.NVIC_IRQChannelSubPriority = 0U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    TIM_Cmd(TIM4, ENABLE);
}

void TIM4_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        uint16_t start_count;
        uint16_t end_count;
        uint16_t elapsed_us;

        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        start_count = TIM_GetCounter(TIM4);
        Balance_ControlStep();
        end_count = TIM_GetCounter(TIM4);

        /* TIM4以1 MHz计数，所以计数差可直接看作微秒。 */
        if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
        {
            /* 控制中断超过5 ms，记录超期并让显示值封顶。 */
            ++overrun_count;
            elapsed_us = CONTROL_TIMER_PERIOD_US;
        }
        else if(end_count >= start_count)
        {
            elapsed_us = end_count - start_count;
        }
        else
        {
            elapsed_us = (uint16_t)(CONTROL_TIMER_PERIOD_US - start_count + end_count);
        }

        last_execution_us = elapsed_us;
        if(elapsed_us > max_execution_us)
        {
            max_execution_us = elapsed_us;
        }
    }
}

uint16_t ControlTimer_GetLastExecutionUs(void) { return last_execution_us; }
uint16_t ControlTimer_GetMaxExecutionUs(void) { return max_execution_us; }
uint32_t ControlTimer_GetOverrunCount(void) { return overrun_count; }
