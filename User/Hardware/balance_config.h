#ifndef __BALANCE_CONFIG_H
#define __BALANCE_CONFIG_H

/* 控制调度：姿态环200 Hz，速度环每10次姿态环运行一次。 */
#define BALANCE_CONTROL_HZ                 200U
#define BALANCE_CONTROL_DT                 0.005f
#define BALANCE_SPEED_DIVIDER              10U
#define BALANCE_SPEED_DT                   0.050f

/* 上电必须连续稳定约2秒；运动超限时校准计数会重新开始。 */
#define BALANCE_CALIBRATION_SAMPLES        400U
#define BALANCE_CALIBRATION_MAX_ANGLE_DEG  15.0f
#define BALANCE_CALIBRATION_MAX_GYRO_DPS   5.0f
/* 相对直立角超过45度立即锁定关断；滤波系数越大越相信陀螺仪。 */
#define BALANCE_FALL_ANGLE_DEG             45.0f
#define BALANCE_COMPLEMENTARY_ALPHA        0.985f

/* 起调增益，输出按Motor_SetPWM(-1000..1000)的范围缩放，并非最终实车参数。 */
#define BALANCE_ANGLE_KP                   22.0f
#define BALANCE_ANGLE_KD                   0.90f
#define BALANCE_SPEED_KP                   70.0f
#define BALANCE_SPEED_KI                   25.0f
#define BALANCE_SPEED_INTEGRAL_LIMIT       8.0f
#define BALANCE_SPEED_OUTPUT_LIMIT         300.0f

#define BALANCE_START_PWM_LIMIT            300
#define BALANCE_FULL_PWM_LIMIT             1000
#define BALANCE_SOFT_START_TICKS           BALANCE_CONTROL_HZ

/* 11 pulse encoder, x4 decoding, 9.27666:1 gearbox. */
#define BALANCE_ENCODER_COUNTS_PER_REV     408.173f

/* Change only these signs if a wheel or encoder runs in the opposite direction. */
#define BALANCE_LEFT_MOTOR_SIGN            1
#define BALANCE_RIGHT_MOTOR_SIGN           1
#define BALANCE_LEFT_ENCODER_SIGN          (-1)
#define BALANCE_RIGHT_ENCODER_SIGN         (-1)

#endif
