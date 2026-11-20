/**
  ******************************************************************************
  * @file    pid.h
  * @brief   PID controller with anti-windup and derivative on measurement.
  ******************************************************************************
  */
#ifndef PID_H
#define PID_H

typedef struct
{
    float kp, ki, kd;     /* gains                                          */
    float integral;       /* running sum of error * dt                      */
    float rate_lpf;       /* low-pass filtered rate, used by the D term     */
    float p_term;         /* each term is stored so it can be printed and   */
    float i_term;         /* examined separately while tuning               */
    float d_term;
    float error;
    float output;         /* control signal, as a percentage, -100 to +100  */
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd);

/* Clear the accumulated history. Call this whenever the controller has been
   out of action, so stale terms cannot kick the motors when it restarts. */
void PID_Reset(PID_t *pid);

/**
  * @brief  One controller update.
  * @param  setpoint      the angle we want, in degrees
  * @param  measured      the filtered angle, in degrees
  * @param  measured_rate how fast the measured angle is changing, in deg/s
  *                       (the gyroscope reading, used for the D term)
  * @param  dt            the sampling interval in seconds
  * @retval the control signal, as a percentage from -100 to +100
  */
float PID_Update(PID_t *pid, float setpoint, float measured,
                 float measured_rate, float dt);

#endif /* PID_H */
