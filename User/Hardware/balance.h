#ifndef __BALANCE_H
#define __BALANCE_H

#include <stdint.h>

typedef enum
{
    BALANCE_STATE_BOOT = 0,
    BALANCE_STATE_CALIBRATING,
    BALANCE_STATE_RUNNING,
    BALANCE_STATE_FALLEN,
    BALANCE_STATE_SENSOR_ERROR
} BalanceState;

typedef struct
{
    int16_t accel_x;                  /* 加速度X轴原始计数 */
    int16_t accel_y;                  /* 加速度Y轴原始计数 */
    int16_t accel_z;                  /* 加速度Z轴原始计数 */
    int16_t gyro_x;                   /* 陀螺仪X轴原始计数 */
    int16_t gyro_y;                   /* 陀螺仪Y轴原始计数，平衡控制使用此轴 */
    int16_t gyro_z;                   /* 陀螺仪Z轴原始计数 */
    float temperature_c;              /* MPU6050内部温度，单位摄氏度 */
    float accel_angle_deg;            /* 加速度计计算的绝对俯仰角 */
    float relative_accel_angle_deg;   /* 减去直立零点后的加速度角 */
    float gyro_rate_dps;              /* 扣除零偏后的俯仰角速度 */
    float gyro_predicted_angle_deg;   /* 陀螺仪积分得到的一步预测角 */
    float fused_pitch_deg;            /* 互补滤波输出，也是PD反馈角 */
    float upright_angle_deg;          /* 上电校准得到的机械直立零点 */
} BalanceTelemetry;

/* 在main.c中定义：1启用倾角阈值保护，0关闭。 */
extern const uint8_t g_balance_fall_protection_enabled;

void Balance_Init(void);
void Balance_ControlStep(void);
BalanceState Balance_GetState(void);
float Balance_GetPitch(void);
float Balance_GetSpeed(void);
int16_t Balance_GetLeftPwm(void);
int16_t Balance_GetRightPwm(void);
uint8_t Balance_GetCalibrationPercent(void);
uint32_t Balance_GetControlTicks(void);
void Balance_GetTelemetry(BalanceTelemetry *telemetry);

#endif
