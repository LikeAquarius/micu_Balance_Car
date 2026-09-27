#include "balance.h"
#include "balance_config.h"
#include "encoder.h"
#include "motor.h"
#include "mpu6050.h"
#include <math.h>
#include <stdbool.h>

#define RAD_TO_DEG             57.2957795f
#define MPU_GYRO_LSB_PER_DPS   65.5f
#define SENSOR_FAILURE_LIMIT   3U

static volatile BalanceState balance_state = BALANCE_STATE_BOOT;
static volatile float pitch_deg;
static volatile float average_speed_rps;
static volatile int16_t left_pwm;
static volatile int16_t right_pwm;
static volatile uint16_t calibration_count;

/*
 * 下面的数据由200 Hz控制中断更新，由主循环只读并显示。
 * 显示层不再直接访问MPU6050，避免重复采样破坏控制周期的一致性。
 */
static volatile uint32_t control_ticks;
static volatile int16_t raw_accel_x;
static volatile int16_t raw_accel_y;
static volatile int16_t raw_accel_z;
static volatile int16_t raw_gyro_x;
static volatile int16_t raw_gyro_y;
static volatile int16_t raw_gyro_z;
static volatile float sensor_temperature_c;
static volatile float accel_angle_deg;
static volatile float relative_accel_angle_deg;
static volatile float gyro_rate_dps;
static volatile float gyro_predicted_angle_deg;
static volatile float target_pitch_deg;
static volatile float telemetry_angle_error_deg;
static volatile float angle_pd_output;

static float gyro_bias_dps;
static float upright_angle_deg;
static float calibration_angle_sum;
static float calibration_gyro_sum;
static float speed_integral;
static uint16_t running_ticks;
static uint8_t speed_divider;
static uint8_t sensor_failures;

static float clamp_float(float value, float minimum, float maximum)
{
    if(value > maximum) return maximum;
    if(value < minimum) return minimum;
    return value;
}

static int16_t clamp_pwm(float value, int16_t limit)
{
    if(value > (float)limit) return limit;
    if(value < (float)-limit) return (int16_t)-limit;
    return (int16_t)value;
}

static void enter_safe_state(BalanceState state)
{
    left_pwm = 0;
    right_pwm = 0;
    target_pitch_deg = 0.0f;
    telemetry_angle_error_deg = 0.0f;
    angle_pd_output = 0.0f;
    speed_integral = 0.0f;
    balance_state = state;
    Motor_EmergencyStop();
}

static void calibration_step(float accel_angle, float gyro_rate)
{
    if(fabsf(accel_angle) > BALANCE_CALIBRATION_MAX_ANGLE_DEG ||
       fabsf(gyro_rate) > BALANCE_CALIBRATION_MAX_GYRO_DPS)
    {
        calibration_count = 0U;
        calibration_angle_sum = 0.0f;
        calibration_gyro_sum = 0.0f;
        return;
    }

    calibration_angle_sum += accel_angle;
    calibration_gyro_sum += gyro_rate;
    ++calibration_count;

    if(calibration_count >= BALANCE_CALIBRATION_SAMPLES)
    {
        upright_angle_deg = calibration_angle_sum / (float)calibration_count;
        gyro_bias_dps = calibration_gyro_sum / (float)calibration_count;
        pitch_deg = 0.0f;
        speed_integral = 0.0f;
        target_pitch_deg = 0.0f;
        speed_divider = 0U;
        running_ticks = 0U;
        Encoder_Reset();
        Motor_SetEnabled(true);
        balance_state = BALANCE_STATE_RUNNING;
    }
}

void Balance_Init(void)
{
    Motor_Init();
    Motor_EmergencyStop();
    Encoder_Init();
    Encoder_Reset();

    pitch_deg = 0.0f;
    average_speed_rps = 0.0f;
    left_pwm = 0;
    right_pwm = 0;
    calibration_count = 0U;
    control_ticks = 0U;
    raw_accel_x = 0;
    raw_accel_y = 0;
    raw_accel_z = 0;
    raw_gyro_x = 0;
    raw_gyro_y = 0;
    raw_gyro_z = 0;
    sensor_temperature_c = 0.0f;
    accel_angle_deg = 0.0f;
    relative_accel_angle_deg = 0.0f;
    gyro_rate_dps = 0.0f;
    gyro_predicted_angle_deg = 0.0f;
    target_pitch_deg = 0.0f;
    telemetry_angle_error_deg = 0.0f;
    angle_pd_output = 0.0f;
    calibration_angle_sum = 0.0f;
    calibration_gyro_sum = 0.0f;
    sensor_failures = 0U;

    if(!MPU6050_Init())
    {
        enter_safe_state(BALANCE_STATE_SENSOR_ERROR);
        return;
    }
    balance_state = BALANCE_STATE_CALIBRATING;
}

void Balance_ControlStep(void)
{
    MPU6050_Sample sample;
    float accel_angle;
    float gyro_rate;
    float relative_accel_angle;
    float angle_error;
    float balance_output;
    float total_output;
    int16_t pwm_limit;

    /* 5 ms控制节拍同时作为OLED刷新和5秒换页的稳定时间基准。 */
    ++control_ticks;

    if(balance_state == BALANCE_STATE_SENSOR_ERROR ||
       balance_state == BALANCE_STATE_FALLEN)
    {
        return;
    }

    if(!MPU6050_ReadSample(&sample))
    {
        if(++sensor_failures >= SENSOR_FAILURE_LIMIT)
        {
            enter_safe_state(BALANCE_STATE_SENSOR_ERROR);
        }
        return;
    }
    sensor_failures = 0U;

    /* 保存本次控制实际使用的同一份样本，供OLED观察。 */
    raw_accel_x = sample.ax;
    raw_accel_y = sample.ay;
    raw_accel_z = sample.az;
    raw_gyro_x = sample.gx;
    raw_gyro_y = sample.gy;
    raw_gyro_z = sample.gz;
    sensor_temperature_c = (float)sample.temperature / 340.0f + 36.53f;

    /*
     * 安装方向决定使用X/Z求俯仰角、Y轴陀螺仪求俯仰角速度。
     * 按当前公式，车体顶部朝MPU6050的+X方向倾斜时角度为正；
     * +X是否指向车头必须在断开电机动力后通过OLED实测确认。
     */
    accel_angle = -atan2f((float)sample.ax, (float)sample.az) * RAD_TO_DEG;
    gyro_rate = (float)sample.gy / MPU_GYRO_LSB_PER_DPS;
    accel_angle_deg = accel_angle;
    gyro_rate_dps = gyro_rate;

    if(balance_state == BALANCE_STATE_CALIBRATING)
    {
        /* 校准期间不输出电机，只累计稳定样本求零点和陀螺零偏。 */
        calibration_step(accel_angle, gyro_rate);
        return;
    }

    gyro_rate -= gyro_bias_dps;
    relative_accel_angle = accel_angle - upright_angle_deg;
    relative_accel_angle_deg = relative_accel_angle;
    gyro_rate_dps = gyro_rate;
    /*
     * 互补滤波：陀螺仪负责短时动态，加速度计缓慢修正长期漂移。
     * gyro_predicted_angle_deg单独保存，便于OLED对比融合前后的角度。
     */
    gyro_predicted_angle_deg = pitch_deg + gyro_rate * BALANCE_CONTROL_DT;
    pitch_deg = BALANCE_COMPLEMENTARY_ALPHA *
                gyro_predicted_angle_deg +
                (1.0f - BALANCE_COMPLEMENTARY_ALPHA) * relative_accel_angle;

    /* 主程序可在编译前选择是否启用跌倒角度保护。 */
    if(g_balance_fall_protection_enabled != 0U &&
       fabsf(pitch_deg) > BALANCE_FALL_ANGLE_DEG)
    {
        enter_safe_state(BALANCE_STATE_FALLEN);
        return;
    }

    if(++speed_divider >= BALANCE_SPEED_DIVIDER)
    {
        /* 姿态环每10次执行一次速度环：200 Hz / 10 = 20 Hz。 */
        int16_t left_count = Encoder_ReadLeft();
        int16_t right_count = Encoder_ReadRight();
        float left_speed = (float)left_count /
                           (BALANCE_ENCODER_COUNTS_PER_REV * BALANCE_SPEED_DT);
        float right_speed = (float)right_count /
                            (BALANCE_ENCODER_COUNTS_PER_REV * BALANCE_SPEED_DT);
        float measured_speed = (left_speed + right_speed) * 0.5f;
        float speed_error;

        speed_divider = 0U;
        average_speed_rps = average_speed_rps * 0.70f + measured_speed * 0.30f;
        if(g_balance_speed_loop_enabled != 0U)
        {
            /*
             * 外环只产生目标倾角，内环PD再把角度误差转成PWM。
             * 这是真正的串级控制，也便于先单独整定角度环。
             */
            speed_error = -average_speed_rps;
            speed_integral += speed_error * BALANCE_SPEED_DT;
            speed_integral = clamp_float(speed_integral,
                                         -BALANCE_SPEED_INTEGRAL_LIMIT,
                                         BALANCE_SPEED_INTEGRAL_LIMIT);
            target_pitch_deg = BALANCE_SPEED_KP * speed_error +
                               BALANCE_SPEED_KI * speed_integral;
            target_pitch_deg = clamp_float(target_pitch_deg,
                                           -BALANCE_SPEED_TARGET_ANGLE_LIMIT,
                                           BALANCE_SPEED_TARGET_ANGLE_LIMIT);
        }
        else
        {
            /* 调试角度环时清除外环状态，避免重新开启时积分突变。 */
            speed_integral = 0.0f;
            target_pitch_deg = 0.0f;
        }
    }

    /* 速度环关闭时target_pitch_deg恒为0，即纯角度PD模式。 */
    angle_error = target_pitch_deg - pitch_deg;
    balance_output = BALANCE_ANGLE_KP * angle_error -
                     BALANCE_ANGLE_KD * gyro_rate;
    total_output = balance_output;
    telemetry_angle_error_deg = angle_error;
    angle_pd_output = balance_output;

    if(running_ticks < BALANCE_SOFT_START_TICKS)
    {
        pwm_limit = (int16_t)(BALANCE_START_PWM_LIMIT +
                    ((int32_t)(BALANCE_FULL_PWM_LIMIT - BALANCE_START_PWM_LIMIT) *
                     running_ticks) / BALANCE_SOFT_START_TICKS);
        ++running_ticks;
    }
    else
    {
        pwm_limit = BALANCE_FULL_PWM_LIMIT;
    }

    left_pwm = clamp_pwm(total_output * BALANCE_LEFT_MOTOR_SIGN, pwm_limit);
    right_pwm = clamp_pwm(total_output * BALANCE_RIGHT_MOTOR_SIGN, pwm_limit);
    Motor_SetPWM(left_pwm, right_pwm);
}

BalanceState Balance_GetState(void) { return balance_state; }
float Balance_GetPitch(void) { return pitch_deg; }
float Balance_GetSpeed(void) { return average_speed_rps; }
int16_t Balance_GetLeftPwm(void) { return left_pwm; }
int16_t Balance_GetRightPwm(void) { return right_pwm; }

uint8_t Balance_GetCalibrationPercent(void)
{
    uint32_t percent = (uint32_t)calibration_count * 100U /
                       BALANCE_CALIBRATION_SAMPLES;
    return (uint8_t)(percent > 100U ? 100U : percent);
}

uint32_t Balance_GetControlTicks(void) { return control_ticks; }

void Balance_GetTelemetry(BalanceTelemetry *telemetry)
{
    if(telemetry == NULL)
    {
        return;
    }

    /* 复制快照，保持OLED模块与控制模块内部变量解耦。 */
    telemetry->accel_x = raw_accel_x;
    telemetry->accel_y = raw_accel_y;
    telemetry->accel_z = raw_accel_z;
    telemetry->gyro_x = raw_gyro_x;
    telemetry->gyro_y = raw_gyro_y;
    telemetry->gyro_z = raw_gyro_z;
    telemetry->temperature_c = sensor_temperature_c;
    telemetry->accel_angle_deg = accel_angle_deg;
    telemetry->relative_accel_angle_deg = relative_accel_angle_deg;
    telemetry->gyro_rate_dps = gyro_rate_dps;
    telemetry->gyro_predicted_angle_deg = gyro_predicted_angle_deg;
    telemetry->fused_pitch_deg = pitch_deg;
    telemetry->upright_angle_deg = upright_angle_deg;
    telemetry->target_pitch_deg = target_pitch_deg;
    telemetry->angle_error_deg = telemetry_angle_error_deg;
    telemetry->angle_pd_output = angle_pd_output;
}
