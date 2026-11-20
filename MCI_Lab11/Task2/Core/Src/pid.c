/**
  ******************************************************************************
  * @file    pid.c
  * @brief   PID controller with anti-windup and derivative on measurement.
  ******************************************************************************
  */
#include "pid.h"
#include "config.h"
#include <math.h>

void PID_Init(PID_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    PID_Reset(pid);
}

void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->rate_lpf = 0.0f;
    pid->p_term   = 0.0f;
    pid->i_term   = 0.0f;
    pid->d_term   = 0.0f;
    pid->error    = 0.0f;
    pid->output   = 0.0f;
}

float PID_Update(PID_t *pid, float setpoint, float measured,
                 float measured_rate, float dt)
{
    pid->error = setpoint - measured;

    /* ---- P: react to how far off we are right now ---- */
    pid->p_term = pid->kp * pid->error;

    /* ---- D: react to how fast the error is changing ----
       The derivative is taken from the gyroscope rather than by differencing
       the angle. Two reasons. Differencing a quantised angle every 10 ms
       amplifies its noise enormously, which on this robot showed up as
       violent twitching. And because the error is (setpoint - angle), its
       derivative is simply minus the rate at which the robot is turning,
       which the gyroscope already measures directly and cleanly. */
    /* Smooth the rate before differentiating against it. Without this, every
       bit of gyro noise is multiplied by KD and sent straight to the motors,
       which is why raising KD normally makes the wheels buzz. */
    pid->rate_lpf += PID_D_LPF_ALPHA * (measured_rate - pid->rate_lpf);
    pid->d_term = -pid->kd * pid->rate_lpf;

    float out_pd = pid->p_term + pid->d_term;

    /* ---- I: react to how long we have been off ----
       Two anti-windup measures protect it.
       First, conditional integration: the sum only grows if doing so would
       not keep the output saturated. Holding the robot tilted by hand would
       otherwise drive it to its limit and leave the motors pinned on after
       the robot is released.
       Second, a leak: sitting exactly at the setpoint makes the error zero,
       so the sum would never shrink by itself and a stored correction would
       keep the wheels turning. Near the setpoint it decays away instead. */
    float candidate = pid->integral + pid->error * dt;
    float test_out  = out_pd + pid->ki * candidate;

    if (fabsf(test_out) < PID_OUTPUT_LIMIT || (test_out * pid->error) < 0.0f)
    {
        pid->integral = candidate;
    }

    if (fabsf(pid->error) < PID_I_DEADBAND)
    {
        pid->integral *= PID_I_DECAY;
    }

    if (pid->integral >  PID_INTEGRAL_LIMIT) pid->integral =  PID_INTEGRAL_LIMIT;
    if (pid->integral < -PID_INTEGRAL_LIMIT) pid->integral = -PID_INTEGRAL_LIMIT;

    pid->i_term = pid->ki * pid->integral;

    /* ---- Sum and clamp to what the motors can actually deliver ---- */
    float out = out_pd + pid->i_term;

    if (out >  PID_OUTPUT_LIMIT) out =  PID_OUTPUT_LIMIT;
    if (out < -PID_OUTPUT_LIMIT) out = -PID_OUTPUT_LIMIT;

    pid->output = out;
    return out;
}
