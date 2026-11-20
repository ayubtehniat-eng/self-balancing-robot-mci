/**
  ******************************************************************************
  * @file    tilt.h
  * @brief   Tilt angle estimation with a complementary filter.
  ******************************************************************************
  */
#ifndef TILT_H
#define TILT_H

#include "imu.h"

typedef struct
{
    float acc_angle;   /* from gravity alone: true on average, but noisy     */
    float gyro_angle;  /* from integrating the rate: smooth, but drifts      */
    float angle;       /* the filter output, and the controller's feedback   */
    float rate;        /* the angular rate about the tilt axis, in deg/s     */
} Tilt_t;

/* Tilt angle from the accelerometer alone, in degrees */
float Tilt_AccAngle(const IMU_t *imu);

/* Seed all three estimates from the accelerometer, so the filter does not
   have to climb from zero when the robot starts out leaning */
void Tilt_Init(Tilt_t *tilt, const IMU_t *imu);

/* One filter step. Must be called exactly every dt seconds. */
void Tilt_Update(Tilt_t *tilt, const IMU_t *imu, float dt);

#endif /* TILT_H */
