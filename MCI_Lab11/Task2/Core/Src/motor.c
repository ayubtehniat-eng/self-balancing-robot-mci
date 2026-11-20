/**
  ******************************************************************************
  * @file    motor.c
  * @brief   Device driver for the two DC motors, via the TB6612 on the shield.
  ******************************************************************************
  */
#include "motor.h"
#include "config.h"
#include <math.h>

static TIM_HandleTypeDef *tim_right = NULL;
static TIM_HandleTypeDef *tim_left  = NULL;

/* The last commanded value for each motor, used by the slew-rate limiter */
static float last_cmd[2] = { 0.0f, 0.0f };

/* Each motor has two direction pins. The pair selects which way it turns:
   HIGH/LOW one way, LOW/HIGH the other, LOW/LOW stopped.                   */
static void set_direction(Motor_t motor, GPIO_PinState in1, GPIO_PinState in2)
{
    if (motor == MOTOR_RIGHT)
    {
        HAL_GPIO_WritePin(R_IN1_GPIO_Port, R_IN1_Pin, in1);
        HAL_GPIO_WritePin(R_IN2_GPIO_Port, R_IN2_Pin, in2);
    }
    else
    {
        HAL_GPIO_WritePin(L_IN1_GPIO_Port, L_IN1_Pin, in1);
        HAL_GPIO_WritePin(L_IN2_GPIO_Port, L_IN2_Pin, in2);
    }
}

/* Duty = CCR / (ARR + 1), and ARR is 999, so a percentage maps onto
   CCR = percent * 10.                                                      */
static void set_duty(Motor_t motor, float percent)
{
    if (percent < 0.0f)            percent = 0.0f;
    if (percent > MOTOR_MAX_DUTY)  percent = MOTOR_MAX_DUTY;

    uint32_t ccr = (uint32_t)(percent * 10.0f);

    if (motor == MOTOR_RIGHT)
    {
        __HAL_TIM_SET_COMPARE(tim_right, TIM_CHANNEL_1, ccr);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(tim_left, TIM_CHANNEL_1, ccr);
    }
}

void Motor_Init(TIM_HandleTypeDef *htim_right, TIM_HandleTypeDef *htim_left)
{
    tim_right = htim_right;
    tim_left  = htim_left;

    HAL_TIM_PWM_Start(tim_right, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(tim_left,  TIM_CHANNEL_1);

    Motor_Stop();
}

void Motor_SetSigned(Motor_t motor, float percent)
{
    /* Per-motor sign, so a wheel wired the other way round is a one-character
       change in config.h rather than a rewiring job. */
    percent *= (motor == MOTOR_RIGHT) ? MOTOR_R_SIGN : MOTOR_L_SIGN;

    /* Slew-rate limit. The controller can swing from one extreme to the other
       within a single 10 ms step, and asking a geared motor to reverse that
       fast makes the robot snap about and draws a large current spike. This
       caps how far the command may move per step, which smooths the response
       without adding any lag in normal operation. */
    float delta = percent - last_cmd[motor];
    if (delta >  MOTOR_SLEW_PER_STEP) percent = last_cmd[motor] + MOTOR_SLEW_PER_STEP;
    if (delta < -MOTOR_SLEW_PER_STEP) percent = last_cmd[motor] - MOTOR_SLEW_PER_STEP;
    last_cmd[motor] = percent;

    float magnitude = fabsf(percent);

    /* Friction compensation. A geared motor will not turn at all below a few
       percent duty, so small corrections would be lost entirely. Adding the
       floor on top of the demand, rather than replacing it, keeps the
       response continuous through zero: without that the output would jump
       from nothing straight to the floor value and the robot would buzz
       between the two. Set MOTOR_MIN_DUTY to 0 to disable. */
    if (magnitude > 0.1f && MOTOR_MIN_DUTY > 0.0f)
    {
        magnitude = MOTOR_MIN_DUTY
                    + magnitude * (MOTOR_MAX_DUTY - MOTOR_MIN_DUTY) / MOTOR_MAX_DUTY;
    }

    if (magnitude < 0.1f)
    {
        set_direction(motor, GPIO_PIN_RESET, GPIO_PIN_RESET);   /* stopped */
        set_duty(motor, 0.0f);
        return;
    }

    if (percent > 0.0f)
    {
        set_direction(motor, GPIO_PIN_SET, GPIO_PIN_RESET);
    }
    else
    {
        set_direction(motor, GPIO_PIN_RESET, GPIO_PIN_SET);
    }

    set_duty(motor, magnitude);
}

void Motor_SetBoth(float percent)
{
#if MOTORS_ENABLED
    Motor_SetSigned(MOTOR_RIGHT, percent);
    Motor_SetSigned(MOTOR_LEFT,  percent);
#else
    (void)percent;    /* motors disabled: useful when checking the controller */
#endif
}

void Motor_Stop(void)
{
    last_cmd[MOTOR_RIGHT] = 0.0f;
    last_cmd[MOTOR_LEFT]  = 0.0f;

    set_duty(MOTOR_RIGHT, 0.0f);
    set_duty(MOTOR_LEFT,  0.0f);
    set_direction(MOTOR_RIGHT, GPIO_PIN_RESET, GPIO_PIN_RESET);
    set_direction(MOTOR_LEFT,  GPIO_PIN_RESET, GPIO_PIN_RESET);
}
