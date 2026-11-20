/**
  ******************************************************************************
  * @file    config.h
  * @brief   Every tunable number for Lab 11 lives here, and nowhere else.
  *
  * Keeping the constants in one header means the control loop, the filter and
  * the motor driver can never disagree about dt, and tuning is a one-file edit.
  ******************************************************************************
  */
#ifndef CONFIG_H
#define CONFIG_H

/* ===================== Control loop timing =============================== */
/* TIM6 fires the control loop at this rate. DT is derived from it, so the
   filter and the PID can never be given different sampling intervals.        */
#define LOOP_HZ             100U
#define DT                  (1.0f / (float)LOOP_HZ)   /* 0.01 s              */
#define PRINT_EVERY         5U      /* print every 5th loop -> 20 Hz         */

/* ===================== Complementary filter ============================== */
#define FILTER_ALPHA        0.98f   /* weight given to the gyroscope         */
#define RAD_TO_DEG          57.2958f

/* ===================== Sensor axes =======================================
   Found by tilting the robot and watching the raw values. The board is
   mounted upside down, so the Z axis points downwards and reads about -1 g.  */
#define GYRO_AXIS           1       /* 0 = X, 1 = Y, 2 = Z                   */
#define GYRO_SIGN           1.0f
#define ACC_FWD_AXIS        0       /* gains gravity as the robot leans      */
#define ACC_FWD_SIGN        1.0f
#define ACC_UP_AXIS         2       /* reads about 1 g when upright          */
#define ACC_UP_SIGN        -1.0f    /* -1: board mounted inverted            */

/* ===================== Start-up calibration ==============================
   Sensor offsets are measured by averaging while the robot is held still,
   then the balance angle is measured the same way.                          */
#define CAL_SAMPLES         200
#define CAL_DELAY_MS        5U
#define SETPOINT_SETTLE     100     /* 1 s for the filter to converge first  */
#define SETPOINT_SAMPLES    200     /* 2 s of averaging                      */
#define SETPOINT_DELAY_MS   10U

/* ===================== PID controller ====================================
   Output is a signed percentage of full motor power, -100 to +100.
   These are STARTING points. Tune in the order P, then D, then I.           */
#define PID_KP              8.0f    /* full output at about 12 deg of lean   */
#define PID_KI              0.0f    /* start at 0; add last, once P and D sit */
#define PID_KD              1.2f    /* damping, applied to the gyro rate     */
/* Low-pass on the rate used by the D term. 1.0 means no filtering, smaller
   means smoother. The gyro is quiet but not silent, and D multiplies whatever
   noise it is given, so this is what lets KD be raised without the wheels
   buzzing. 0.3 gives a time constant of about 23 ms.                        */
#define PID_D_LPF_ALPHA     0.3f
#define PID_INTEGRAL_LIMIT  3.0f    /* anti-windup clamp on the running sum  */
#define PID_OUTPUT_LIMIT    100.0f  /* the PWM cannot exceed 100 % duty      */
#define PID_I_DEADBAND      1.0f    /* within this error, bleed the sum away */
#define PID_I_DECAY         0.98f   /* 2 % per step: about a 0.5 s decay     */

/* ===================== Motors ============================================ */
#define MOTORS_ENABLED      1       /* 0 = run the controller with motors off */
#define MOTOR_MIN_DUTY      0.0f    /* friction compensation; see motor.c     */
#define MOTOR_MAX_DUTY      100.0f
/* Maximum change in duty per 10 ms step. This exists to stop the motors being
   asked to reverse completely in one step, but every unit of it is phase lag,
   and phase lag is what makes a balancing loop oscillate. The robot tips in
   roughly 100 ms, so the limit must let the output cross its full range in a
   small fraction of that: 50 per step means 40 ms end to end. Raise it to 200
   to switch the limiter off entirely.                                       */
#define MOTOR_SLEW_PER_STEP 50.0f
#define MOTOR_R_SIGN        1.0f    /* flip to -1.0f if a wheel runs backwards */
#define MOTOR_L_SIGN        1.0f

/* ===================== Safety ============================================
   Past this lean the robot has fallen over. Driving the motors then only
   makes it skid across the bench, so the controller cuts out and resets.     */
#define TILT_CUTOFF_DEG     45.0f

/* ===================== Terminal output ===================================
   0 = readable text, 1 = CSV "acc,gyro,filtered", 2 = CSV plus the PID output */
#define PLOT_MODE           0
#define AXIS_TEST           0       /* 1 = print raw axes instead of angles   */

#endif /* CONFIG_H */
