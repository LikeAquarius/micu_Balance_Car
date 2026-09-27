#ifndef __MOTOR_CONFIG_H
#define __MOTOR_CONFIG_H

/* Motor_SetPWM()的输入命令范围。 */
#define MOTOR_COMMAND_MAX          1000

/* 绝对值不超过该值时按零输出处理，避免零点换向抖动。 */
#define MOTOR_COMMAND_STOP_BAND    5

/* 左右电机静摩擦不同，启动死区需要分别标定。 */
#define MOTOR_LEFT_DEAD_ZONE       100U
#define MOTOR_RIGHT_DEAD_ZONE      100U

/* 96 MHz / (4799 + 1) = 20 kHz PWM。 */
#define MOTOR_TIMER_PERIOD         4799U

#endif
