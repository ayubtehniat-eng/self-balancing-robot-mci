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
static volatile uint8_t s_fallen     = 0;
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
    uprintf("Ready.\r\n");
}

void Balance_Step(void)
{
    static uint32_t print_count = 0;

    /* 1. Read both sensors */
    IMU_Read(&imu);

    /* 2. Update the tilt estimate */
    Tilt_Update(&tilt, &imu, DT);

    /* 3. If the robot has fallen past recovery, driving the motors only makes
          it skid, so cut the output and clear the controller's history. */
    float output;

    if (fabsf(tilt.angle - setpoint) > TILT_CUTOFF_DEG)
    {
        PID_Reset(&pid);
        Motor_Stop();
        output     = 0.0f;
        s_fallen   = 1;
    }
    else
    {
        /* 4. The controller uses the filtered angle as its feedback, and the
              gyroscope rate for its derivative term. */
        output = PID_Update(&pid, setpoint, tilt.angle, tilt.rate, DT);

        /* 5. Drive both motors from the signed output */
        Motor_SetBoth(output);
        s_fallen = 0;
    }

    /* 6. Hand the values to the main loop, more slowly than the loop runs */
    s_acc_angle  = tilt.acc_angle;
    s_gyro_angle = tilt.gyro_angle;
    s_angle      = tilt.angle;
    s_rate       = tilt.rate;
    s_output     = output;

    if (++print_count >= PRINT_EVERY)
    {
        print_count  = 0;
        s_print_flag = 1;
    }
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
    /* Copy the shared values with interrupts off, so the ISR cannot update
       them halfway through and leave us printing a mismatched set. */
    __disable_irq();
    float acc_angle  = s_acc_angle;
    float gyro_angle = s_gyro_angle;
    float angle      = s_angle;
    float rate       = s_rate;
    float output     = s_output;
    uint8_t fallen   = s_fallen;
    __enable_irq();

#if AXIS_TEST
    uprintf("Acc %c=%6.2f g  Acc %c=%6.2f g  Gyro %c=%7.1f dps\r\n",
            IMU_AxisName(ACC_FWD_AXIS), ACC_FWD_SIGN * imu.acc_g[ACC_FWD_AXIS],
            IMU_AxisName(ACC_UP_AXIS),  ACC_UP_SIGN  * imu.acc_g[ACC_UP_AXIS],
            IMU_AxisName(GYRO_AXIS),    rate);
#elif PLOT_MODE == 1
    uprintf("%.2f,%.2f,%.2f\r\n", acc_angle, gyro_angle, angle);
#elif PLOT_MODE == 2
    uprintf("%.2f,%.2f,%.2f,%.2f\r\n", acc_angle, gyro_angle, angle, output);
#else
    uprintf("acc=%7.2f gyro=%7.2f filt=%7.2f rate=%7.1f | PID=%7.2f%% %s\r\n",
            acc_angle, gyro_angle, angle, rate, output,
            fallen ? "[FALLEN]" : "");
#endif
}
