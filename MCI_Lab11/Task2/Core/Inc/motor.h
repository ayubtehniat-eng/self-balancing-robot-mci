/**
  ******************************************************************************
  * @file    motor.h
  * @brief   Device driver for the two DC motors on the Keyestudio shield.
  *
  *          Right motor: PWM on PC6 (TIM3_CH1), direction on PA1 and PA2
  *          Left  motor: PWM on PA15 (TIM2_CH1), direction on PC10 and PC9
  *
  * Callers work in signed percentages. The sign chooses the direction, the
  * magnitude becomes the PWM duty cycle.
  ******************************************************************************
  */
#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

typedef enum
{
    MOTOR_RIGHT = 0,
    MOTOR_LEFT  = 1
} Motor_t;

/* Start both PWM channels with the motors stopped */
void Motor_Init(TIM_HandleTypeDef *htim_right, TIM_HandleTypeDef *htim_left);

/* Drive one motor. percent runs from -100 to +100. */
void Motor_SetSigned(Motor_t motor, float percent);

/* Drive both motors together, which is what the balance controller does */
void Motor_SetBoth(float percent);

/* Cut both motors immediately and clear the slew-rate memory */
void Motor_Stop(void);

#endif /* MOTOR_H */
