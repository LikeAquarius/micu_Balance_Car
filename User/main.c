/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Description        : CH32V203 two-wheel self-balancing car.
 *******************************************************************************/

#include "debug.h"
#include "Hardware/balance.h"
#include "Hardware/control_timer.h"
#include "Hardware/oled.h"
#include <stdio.h>

/* 一个控制tick为5 ms：40 tick刷新显示，1000 tick切换页面。 */
#define DISPLAY_REFRESH_TICKS  40U
#define DISPLAY_PAGE_TICKS     1000U

/*
 * 倾角阈值保护编译开关：
 * 1 = 融合角超过BALANCE_FALL_ANGLE_DEG后锁定关闭电机；
 * 0 = 不检查倾角阈值，仅建议在架空、断开电机动力电源时调试使用。
 * 修改后需要重新编译并烧录固件。
 */
const uint8_t g_balance_fall_protection_enabled = 0U;

//状态枚举转字符串
static const char *state_text(BalanceState state)
{
    switch(state)
    {
        case BALANCE_STATE_CALIBRATING: return "CALIBRATING";
        case BALANCE_STATE_RUNNING: return "BALANCING";
        case BALANCE_STATE_FALLEN: return "FALLEN";
        case BALANCE_STATE_SENSOR_ERROR: return "MPU ERROR";
        default: return "BOOT";
    }
}

//简易浮点数格式化
static void format_fixed1(char *buffer, size_t length, float value)
{
    /* newlib-nano默认不启用printf浮点格式，手动转为一位小数字符串可节省Flash。 */
    int scaled = (int)(value * 10.0f);
    int magnitude = scaled < 0 ? -scaled : scaled;
    (void)snprintf(buffer, length, "%c%d.%d",
                   scaled < 0 ? '-' : '+', magnitude / 10, magnitude % 10);
}

//第一页‑MPU 原始传感器页面
static void display_mpu_page(const BalanceTelemetry *data, BalanceState state)
{
    char line[32];
    char temperature[16];

    /* 第一页保留原始ADC计数，便于判断接线、轴方向、饱和和传感器掉线。 */
    OLED_ClearBuffer();
    OLED_Print(0, 0, "MPU6050 RAW  1/2");
    (void)snprintf(line, sizeof(line), "AX:%6d AY:%6d", data->accel_x, data->accel_y);
    OLED_Print(1, 0, line);
    if(state == BALANCE_STATE_SENSOR_ERROR)
    {
        (void)snprintf(line, sizeof(line), "AZ:%6d ID:ERROR", data->accel_z);
    }
    else
    {
        (void)snprintf(line, sizeof(line), "AZ:%6d ID:68 OK", data->accel_z);
    }
    OLED_Print(2, 0, line);
    (void)snprintf(line, sizeof(line), "GX:%6d GY:%6d", data->gyro_x, data->gyro_y);
    OLED_Print(3, 0, line);
    (void)snprintf(line, sizeof(line), "GZ:%6d", data->gyro_z);
    OLED_Print(4, 0, line);
    format_fixed1(temperature, sizeof(temperature), data->temperature_c);
    (void)snprintf(line, sizeof(line), "TEMP:%s C", temperature);
    OLED_Print(5, 0, line);
    OLED_Print(6, 0, "ACC:2G  GYRO:500DPS");
    (void)snprintf(line, sizeof(line), "STATE:%s", state_text(state));
    OLED_Print(7, 0, line);
    OLED_Refresh();
}

static void display_angle_page(const BalanceTelemetry *data, BalanceState state)
{
    char line[32];
    char value_a[16];
    char value_b[16];

    /* 第二页展示从原始数据到最终控制角的姿态解算链路。 */
    OLED_ClearBuffer();
    OLED_Print(0, 0, "BALANCE ANGLE 2/2");
    format_fixed1(value_a, sizeof(value_a), data->accel_angle_deg);
    (void)snprintf(line, sizeof(line), "ACC ABS:%s DEG", value_a);
    OLED_Print(1, 0, line);
    format_fixed1(value_a, sizeof(value_a), data->relative_accel_angle_deg);
    (void)snprintf(line, sizeof(line), "ACC REL:%s DEG", value_a);
    OLED_Print(2, 0, line);
    format_fixed1(value_a, sizeof(value_a), data->gyro_rate_dps);
    (void)snprintf(line, sizeof(line), "GYR RATE:%s DPS", value_a);
    OLED_Print(3, 0, line);
    format_fixed1(value_a, sizeof(value_a), data->gyro_predicted_angle_deg);
    (void)snprintf(line, sizeof(line), "GYR PRED:%s DEG", value_a);
    OLED_Print(4, 0, line);
    format_fixed1(value_a, sizeof(value_a), data->fused_pitch_deg);
    (void)snprintf(line, sizeof(line), "FUSED:%s DEG", value_a);
    OLED_Print(5, 0, line);
    format_fixed1(value_a, sizeof(value_a), data->upright_angle_deg);
    format_fixed1(value_b, sizeof(value_b), 45.0f);
    (void)snprintf(line, sizeof(line), "ZERO:%s LIM:%s", value_a, value_b);
    OLED_Print(6, 0, line);
    (void)snprintf(line, sizeof(line), "STATE:%s", state_text(state));
    OLED_Print(7, 0, line);
    OLED_Refresh();
}

static void display_status(void)
{
    BalanceTelemetry telemetry;
    BalanceState state = Balance_GetState();
    uint32_t ticks = Balance_GetControlTicks();

    /* 用200 Hz控制计数切页，不依赖主循环和OLED传输耗时。 */
    Balance_GetTelemetry(&telemetry);
    if(((ticks / DISPLAY_PAGE_TICKS) & 1U) == 0U)
    {
        display_mpu_page(&telemetry, state);
    }
    else
    {
        display_angle_page(&telemetry, state);
    }
}

int main(void)
{
    bool oled_ready;
    uint32_t last_display_tick = 0U;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();

    oled_ready = OLED_Init();
    Balance_Init();
    ControlTimer_Init();

    while(1)
    {
        /* 显示始终放在主循环；实时平衡控制由高优先级TIM4中断执行。 */
        if(oled_ready)
        {
            uint32_t now = Balance_GetControlTicks();
            if((uint32_t)(now - last_display_tick) >= DISPLAY_REFRESH_TICKS)
            {
                last_display_tick = now;
                display_status();
            }
        }
        Delay_Ms(10);
    }
}
