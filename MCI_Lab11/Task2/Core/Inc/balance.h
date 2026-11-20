/**
  ******************************************************************************
  * @file    balance.h
  * @brief   The balance controller: ties the IMU, the filter, the PID and the
  *          motors into one fixed-rate control loop.
  ******************************************************************************
  */
#ifndef BALANCE_H
#define BALANCE_H

#include "main.h"

/* Bring up the sensors, calibrate them, measure the balance angle and arm
   the motors. Call once, before the control timer is started. */
void Balance_Init(I2C_HandleTypeDef *hi2c, SPI_HandleTypeDef *hspi,
                  TIM_HandleTypeDef *htim_right, TIM_HandleTypeDef *htim_left);

/* One control step: read, filter, control, drive. Called from the timer ISR
   every DT seconds, so it contains no printing and no delays. */
void Balance_Step(void);

/* True when the ISR has produced a fresh set of values worth printing.
   Reading it clears the flag. */
uint8_t Balance_PrintDue(void);

/* Send the latest values to the terminal. Called from the main loop only. */
void Balance_Print(void);

#endif /* BALANCE_H */
