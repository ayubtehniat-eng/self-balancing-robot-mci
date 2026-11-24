/**
  ******************************************************************************
  * @file    balance.c
  * @brief   The balance controller.
  *
  * Everything that must happen at a fixed rate lives in Balance_Step(), which
  * the timer ISR calls every DT seconds. A hardware timer drives it so the
  * interval is exact: the filter integrates rate * dt and the controller
  * divides by dt, so any jitter in the interval corrupts both.
  ******************************************************************************
  */
#include "balance.h"
#include "config.h"
#include "imu.h"
#include "tilt.h"
#include "pid.h"
#include "motor.h"
#include "uart_print.h"
#include <math.h>

static IMU_t  imu;
static Tilt_t tilt;
static PID_t  pid;

static float setpoint = 0.0f;     /* the robot's measured balance angle */

/* Written by the ISR, read by the main loop, so they must be volatile */
static volatile float   s_acc_angle  = 0.0f;
static volatile float   s_gyro_angle = 0.0f;
static volatile float   s_angle      = 0.0f;
static volatile float   s_rate       = 0.0f;
static volatile float   s_output     = 0.0f;
static volatile uint8_t s_armed      = 0;
static volatile uint8_t s_print_flag = 0;

/* Measure the balance angle by averaging the filtered angle while the robot
   is held upright. Averaging removes the accelerometer's noise, so the stored
   value is the true resting angle rather than one noisy sample. The balance
   point is rarely 0 degrees, because the battery and board are not centred
   over the wheel axle. */
static float measure_setpoint(void)
{
    float sum = 0.0f;

    /* Let the filter converge first. Averaging immediately would include the
       transient and drag the result away from the resting angle. */
    for (int n = 0; n < SETPOINT_SETTLE; n++)
    {
        IMU_Read(&imu);
        Tilt_Update(&tilt, &imu, DT);
        HAL_Delay(SETPOINT_DELAY_MS);
    }

    for (int n = 0; n < SETPOINT_SAMPLES; n++)
    {
        IMU_Read(&imu);
        Tilt_Update(&tilt, &imu, DT);
        sum += tilt.angle;
        HAL_Delay(SETPOINT_DELAY_MS);
    }

    return sum / SETPOINT_SAMPLES;
}

void Balance_Init(I2C_HandleTypeDef *hi2c, SPI_HandleTypeDef *hspi,
                  TIM_HandleTypeDef *htim_right, TIM_HandleTypeDef *htim_left)
{
    uprintf("\r\nLab 11 Task 2: tilt angle + PID motor control\r\n");

    if (IMU_Init(hi2c, hspi) != HAL_OK)
    {
        uprintf("Sensor ID check failed\r\n");
    }

    uprintf("Calibrating: hold the robot still and upright...\r\n");
    IMU_Calibrate(&imu);

    IMU_Read(&imu);
    Tilt_Init(&tilt, &imu);

    uprintf("Measuring the balance angle, keep holding...\r\n");
    setpoint = measure_setpoint();
    uprintf("Setpoint = %.2f deg\r\n", setpoint);

    PID_Init(&pid, PID_KP, PID_KI, PID_KD);
    Motor_Init(htim_right, htim_left);

    uprintf("PID: Kp=%.2f Ki=%.2f Kd=%.2f, dt=%.3f s, loop=%u Hz\r\n",
            pid.kp, pid.ki, pid.kd, (float)DT, (unsigned)LOOP_HZ);
    uprintf("Ready. Stand the robot up and let go; it arms by itself.\r\n");
}

/* Light a board LED while the controller is armed, so the robot can be read
   from across the room without looking at the terminal. */
static void arm_led(uint8_t on)
{
#if ARM_LED_ENABLED
    HAL_GPIO_WritePin(GPIOE, LD6_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    (void)on;
#endif
}

void Balance_Step(void)
{
    static uint32_t print_count  = 0;

    /* 1. Read both sensors and update the tilt estimate. This happens in
          every mode, so the terminal always shows what the robot thinks. */
    IMU_Read(&imu);
    Tilt_Update(&tilt, &imu, DT);

    float error  = fabsf(tilt.angle - setpoint);
    float output = 0.0f;
    uint8_t driving = 0;

#if MOTOR_TEST_MODE

    /* Hardware check: the controller is bypassed entirely. Drive both motors
       one way, then the other, regardless of what the sensors say. If the
       wheels do not turn here, the fault is wiring, power or the driver. */
    static uint32_t test_tick = 0;
    const uint32_t half_cycle = MOTOR_TEST_SECONDS * LOOP_HZ;

    output = ((test_tick / half_cycle) % 2U) ? -MOTOR_TEST_DUTY : MOTOR_TEST_DUTY;
    test_tick++;

    Motor_SetBoth(output);
    driving = 1;
    (void)error;

#elif ARMING_ENABLED

    /* Wait until the robot is standing up steadily before driving anything */
    static uint32_t steady_count = 0;
    static uint8_t  armed        = 0;
    const uint32_t arm_hold_loops = (ARM_HOLD_MS * LOOP_HZ) / 1000U;

    if (armed)
    {
        if (error > TILT_CUTOFF_DEG)
        {
            armed = 0; steady_count = 0;
            PID_Reset(&pid);
            Motor_Stop();
        }
        else
        {
            output = PID_Update(&pid, setpoint, tilt.angle, tilt.rate, DT);
            Motor_SetBoth(output);
        }
    }
    else
    {
        Motor_Stop();
        if (error < ARM_ANGLE_DEG && fabsf(tilt.rate) < ARM_RATE_DPS)
        {
            if (++steady_count >= arm_hold_loops)
            {
                PID_Reset(&pid);
                armed = 1; steady_count = 0;
            }
        }
        else
        {
            steady_count = 0;
        }
    }
    driving = armed;

#else

    /* Simple mode: the controller just runs. The only guard is the fall
       cutoff, which stops the wheels once the robot is on its side. */
    if (error > TILT_CUTOFF_DEG)
    {
        PID_Reset(&pid);
        Motor_Stop();
    }
    else
    {
        /* 2. The controller uses the filtered angle as its feedback and the
              gyroscope rate for its derivative term. */
        output = PID_Update(&pid, setpoint, tilt.angle, tilt.rate, DT);

        /* 3. Drive both motors from the signed output */
        Motor_SetBoth(output);
        driving = 1;
    }

#endif

    s_armed = driving;
    arm_led(driving);

    s_acc_angle  = tilt.acc_angle;
    s_gyro_angle = tilt.gyro_angle;
    s_angle      = tilt.angle;
    s_rate       = tilt.rate;
    s_output     = output;

#if PRINT_ENABLED
    if (++print_count >= PRINT_EVERY)
    {
        print_count  = 0;
        s_print_flag = 1;
    }
#else
    (void)print_count;
#endif
}

uint8_t Balance_IsArmed(void)
{
    return s_armed;
}

uint8_t Balance_PrintDue(void)
{
    if (s_print_flag)
    {
        s_print_flag = 0;
        return 1;
    }
    return 0;
}

void Balance_Print(void)
{
#if !PRINT_ENABLED
    return;      /* output switched off in config.h */
#else
    /* Copy the shared values with interrupts off, so the ISR cannot update
       them halfway through and leave us printing a mismatched set. */
    __disable_irq();
    float acc_angle  = s_acc_angle;
    float gyro_angle = s_gyro_angle;
    float angle      = s_angle;
    float rate       = s_rate;
    float output     = s_output;
    uint8_t armed    = s_armed;
    __enable_irq();

    /* Some output formats do not use every value; this keeps the compiler
       quiet without changing what is printed. */
    (void)acc_angle; (void)gyro_angle; (void)angle;
    (void)rate; (void)output; (void)armed;

#if AXIS_TEST
    uprintf("Acc %c=%6.2f g  Acc %c=%6.2f g  Gyro %c=%7.1f dps\r\n",
            IMU_AxisName(ACC_FWD_AXIS), ACC_FWD_SIGN * imu.acc_g[ACC_FWD_AXIS],
            IMU_AxisName(ACC_UP_AXIS),  ACC_UP_SIGN  * imu.acc_g[ACC_UP_AXIS],
            IMU_AxisName(GYRO_AXIS),    rate);
#elif PLOT_MODE == 1
    uprintf("%.2f,%.2f,%.2f\r\n", acc_angle, gyro_angle, angle);
#elif PLOT_MODE == 2
    uprintf("%.2f,%.2f,%.2f,%.2f,%u\r\n",
            acc_angle, gyro_angle, angle, output, (unsigned)armed);
#else
    uprintf("acc=%7.2f gyro=%7.2f filt=%7.2f rate=%7.1f | PID=%7.2f%% %s\r\n",
            acc_angle, gyro_angle, angle, rate, output,
            armed ? "ARMED" : "waiting: stand it up");
#endif
#endif
}
