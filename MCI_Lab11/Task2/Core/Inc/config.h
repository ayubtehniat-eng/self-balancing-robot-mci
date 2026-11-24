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
   The board was calibrated in Task 1 lying flat with its face UP. For Task 2
   it is mounted on the robot flipped 180 degrees, because the long pins are
   on the underside. A 180 degree flip about the X axis reverses two of the
   three axes and leaves the third alone:

       X  (forward)  unchanged    ->  ACC_FWD_SIGN  +1
       Y  (tilt axis) REVERSED    ->  GYRO_SIGN     -1
       Z  (up)        REVERSED    ->  ACC_UP_SIGN   -1

   The gyroscope sign matters more than it looks. The complementary filter
   adds rate * dt to its own estimate, so a reversed rate pushes the angle
   the wrong way and fights the accelerometer instead of leading it. The D
   term then becomes positive feedback rather than damping, which is why
   raising KD made the robot worse rather than steadier.

   To check on the robot: set PLOT_MODE 0, tilt it, and watch the acc and
   gyro columns. They must move the SAME direction. If they move opposite
   ways, flip GYRO_SIGN.                                                     */
#define GYRO_AXIS           1       /* 0 = X, 1 = Y, 2 = Z                   */
#define GYRO_SIGN          -1.0f    /* -1: tilt axis reversed by the flip     */
#define ACC_FWD_AXIS        0       /* gains gravity as the robot leans      */
#define ACC_FWD_SIGN        1.0f    /* +1: forward axis unaffected by the flip*/
#define ACC_UP_AXIS         2       /* reads about 1 g when upright          */
#define ACC_UP_SIGN        -1.0f    /* -1: up axis reversed by the flip       */

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

   These values are derived from a working robot on the same kit. That code
   expressed its gains in raw PWM counts with full scale at 650 out of 999,
   so every gain there must be multiplied by 100/650 to mean the same thing
   here:  Kp 14 -> 2.15,  Kd 1.0 -> 0.154,  I limit 40 -> 6.2.
   Expressed as behaviour, Kp reaches full output at about 46 degrees of lean
   rather than 12, which is far gentler than it first looks.                 */
#define PID_KP              8.0f    /* full output at about 12 deg of lean   */
#define PID_KI              0.0f    /* the working robot balances with Ki=0  */
/* Keep Kd in proportion to Kp. The working robot's own note says to hold
   KD = KP / 14, and that ratio is unit-free so it carries straight across:
      Kp  6 -> Kd 0.43      Kp 10 -> Kd 0.71
      Kp  8 -> Kd 0.57      Kp 12 -> Kd 0.86
   Raising Kp alone makes the robot stiff and prone to overshoot, because the
   damping no longer matches the push. Move them together.                   */
#define PID_KD              0.57f   /* = PID_KP / 14                         */
/* Low-pass on the rate used by the D term. 1.0 means no filtering. With Kd
   this small the gyro noise is barely amplified, so filtering is off to
   match the working configuration. Lower it towards 0.3 only if the wheels
   buzz once Kd has been raised.                                             */
#define PID_D_LPF_ALPHA     1.0f
#define PID_INTEGRAL_LIMIT  6.0f    /* anti-windup clamp on the running sum  */
#define PID_OUTPUT_LIMIT    100.0f  /* the PWM cannot exceed 100 % duty      */
#define PID_I_DEADBAND      1.0f    /* within this error, bleed the sum away */
#define PID_I_DECAY         0.98f   /* 2 % per step: about a 0.5 s decay     */

/* ===================== Motors ============================================ */
#define MOTORS_ENABLED      1       /* 0 = run the controller with motors off */
/* The working robot never drives past 65 % duty. Capping it costs a little
   authority at extreme angles but stops the robot throwing itself across the
   bench on the first correction, which is most of what made ours violent.   */
#define MOTOR_MAX_DUTY      100.0f
/* Friction compensation: these motors do not turn below roughly 14 % duty,
   so the floor is added on top of the demand rather than replacing it, which
   keeps the response continuous through zero.                               */
#define MOTOR_MIN_DUTY      14.0f
/* The working robot has no slew limiting at all. 200 effectively disables it,
   because every unit of slew limiting is phase lag and phase lag is what
   makes a balancing loop oscillate.                                         */
#define MOTOR_SLEW_PER_STEP 200.0f
#define MOTOR_R_SIGN        1.0f    /* flip to -1.0f if a wheel runs backwards */
#define MOTOR_L_SIGN        1.0f

/* ===================== Motor test mode ===================================
   Set this to 1 to take the controller out of the picture completely. The
   sensors are still read and printed, but the motors are simply driven one
   way for a couple of seconds, then the other, over and over.

   This answers one question: do the motors work at all? If the wheels do not
   turn in this mode, the fault is wiring, power or the driver, and no gain
   value will ever help. Set it back to 0 once they do.                      */
#define MOTOR_TEST_MODE     0
#define MOTOR_TEST_DUTY     40.0f   /* percent duty used in the test         */
#define MOTOR_TEST_SECONDS  2U      /* seconds each way                      */

/* ===================== Arming and safety =================================
   With ARMING_ENABLED 0 the controller simply runs: the motors respond to
   tilt from the moment start-up finishes. Simple, and what you want while
   getting the thing working.

   Set it to 1 and the motors stay off until the robot has been held within
   ARM_ANGLE_DEG of the balance point, turning slower than ARM_RATE_DPS, for
   ARM_HOLD_MS. That stops the wheels thrashing while the robot is carried to
   the floor, which is convenient later but gets in the way while debugging.

   TILT_CUTOFF_DEG applies either way: past that lean the robot is lying on
   its side, so the motors stop rather than skidding it across the floor.     */
#define ARMING_ENABLED      0
#define ARM_ANGLE_DEG       10.0f
#define ARM_RATE_DPS        40.0f
#define ARM_HOLD_MS         400U
#define TILT_CUTOFF_DEG     55.0f

/* A board LED mirrors the state, so the robot can be read from across the
   room with no terminal: lit means armed and driving.                        */
#define ARM_LED_ENABLED     1

/* ===================== Terminal output ===================================
   Set PRINT_ENABLED to 0 to switch all serial output off. Note that this does
   NOT speed up the control loop: printing happens in the main loop and the
   TIM6 interrupt preempts it, so the loop still runs every 10 ms either way.
   The lab also requires the angles to be transmitted in real time, so turn it
   back on before the demo.                                                   */
#define PRINT_ENABLED       1
/* 0 = readable text, 1 = CSV "acc,gyro,filtered", 2 = CSV plus the PID output */
#define PLOT_MODE           0
#define AXIS_TEST           0       /* 1 = print raw axes instead of angles   */

#endif /* CONFIG_H */
