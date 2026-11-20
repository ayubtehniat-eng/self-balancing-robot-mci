/**
  ******************************************************************************
  * @file    tilt.c
  * @brief   Tilt angle estimation with a complementary filter.
  *
  * Neither sensor can give the angle on its own:
  *   the accelerometer measures the direction of gravity, so it is correct on
  *   average but jumps with every bump and vibration;
  *   the gyroscope measures how fast the robot is turning, so integrating it
  *   is smooth and instant but accumulates a drift that never goes away.
  *
  * The filter takes the gyroscope's change and nudges the running estimate
  * towards the accelerometer by a small fraction each step. Fast movement
  * comes from the gyroscope, the long-term average from the accelerometer.
  ******************************************************************************
  */
#include "tilt.h"
#include "config.h"
#include <math.h>

float Tilt_AccAngle(const IMU_t *imu)
{
    /* As the robot leans, gravity is shared between the forward axis and the
       up axis. The two readings are the sides of a right triangle whose
       angle is the lean, so atan2 recovers it. atan2 is used rather than a
       division so that a zero up-component and the sign of the lean are both
       handled correctly. */
    float fwd = ACC_FWD_SIGN * imu->acc_g[ACC_FWD_AXIS];
    float up  = ACC_UP_SIGN  * imu->acc_g[ACC_UP_AXIS];

    return atan2f(fwd, up) * RAD_TO_DEG;
}

void Tilt_Init(Tilt_t *tilt, const IMU_t *imu)
{
    tilt->acc_angle  = Tilt_AccAngle(imu);
    tilt->gyro_angle = tilt->acc_angle;
    tilt->angle      = tilt->acc_angle;
    tilt->rate       = 0.0f;
}

void Tilt_Update(Tilt_t *tilt, const IMU_t *imu, float dt)
{
    /* Angular rate about the axis the robot tips around */
    tilt->rate = GYRO_SIGN * imu->gyro_dps[GYRO_AXIS];

    tilt->acc_angle   = Tilt_AccAngle(imu);
    tilt->gyro_angle += tilt->rate * dt;      /* kept only for comparison */

    /* angle = 0.98 * (angle + rate * dt) + 0.02 * acc_angle
       The 0.98 share follows the gyroscope, so the estimate is smooth and
       reacts at once. The 0.02 share pulls gently towards the accelerometer,
       which cancels the gyroscope's drift with a time constant of
       alpha * dt / (1 - alpha), about half a second here. */
    tilt->angle = FILTER_ALPHA * (tilt->angle + tilt->rate * dt)
                  + (1.0f - FILTER_ALPHA) * tilt->acc_angle;
}
