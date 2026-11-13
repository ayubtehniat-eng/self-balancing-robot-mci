/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* ---------- Raw sensor data, in real units ---------- */
typedef struct
{
    float acc_g[3];      // acceleration on X, Y, Z in g
    float gyro_dps[3];   // rotation speed on X, Y, Z in deg/s (offset removed)
    float gyro_bias[3];  // gyro offset measured while flat and still
    float acc_bias[3];   // accelerometer offset measured while flat and still
} IMU_t;

/* ---------- The three tilt angle estimates, in degrees ---------- */
typedef struct
{
    float acc_angle;     // from the accelerometer only (correct, but noisy)
    float gyro_angle;    // from the gyroscope only (smooth, but drifts)
    float angle;         // complementary filter result: the one we control with
} Tilt_t;

/* ---------- PID controller state ---------- */
typedef struct
{
    float kp, ki, kd;    // gains
    float integral;      // running sum of error * dt
    float prev_error;    // error from the previous update, for the D term
    float p_term;        // kept separately so each term can be printed
    float i_term;
    float d_term;
    float output;        // final control signal, in percent duty (-100 to +100)
} PID_t;

/* ---------- Which motor ---------- */
typedef enum
{
    MOTOR_RIGHT = 0,
    MOTOR_LEFT  = 1
} Motor_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ===================== Timing: one dt used everywhere ===================== */
#define LOOP_HZ          100U                  // TIM6 interrupt rate
#define DT               (1.0f / LOOP_HZ)      // dt = 0.01 s, used by filter AND PID
#define PRINT_EVERY      5U                    // print every 5th ISR -> 20 Hz

/* ===================== Complementary filter ===================== */
#define ALPHA            0.98f                 // weight on the gyro
#define RAD_TO_DEG       57.2958f              // 180 / pi

/* ===================== Output format ===================== */
#define PLOT_MODE        0    // 0 = labelled text, 1 = CSV "acc,gyro,filt", 2 = CSV + PID output
#define AXIS_TEST        0    // 1 = print raw axes instead of angles

/* ===================== Axis selection (found by tilting the board) =========
   Tilting about the Y axis moves gravity between X and Z, so:
     gyro Y  = how fast we are tipping
     acc X   = the "downhill" axis
     acc Z   = the "up out of the board" axis                                */
#define GYRO_AXIS        1       // 0 = X, 1 = Y, 2 = Z
#define GYRO_SIGN        1.0f    // flip to -1.0f if gyro and accel disagree
#define ACC_FWD_AXIS     0       // axis that gains gravity when leaning forward
#define ACC_FWD_SIGN     1.0f
#define ACC_UP_AXIS      2       // axis that reads about 1 g when flat
#define ACC_UP_SIGN     -1.0f    // -1 because the board is mounted UPSIDE DOWN,
                                 // so the Z axis points down and reads about -1 g

/* ===================== PID controller ===================== */
/* The setpoint is MEASURED at start-up, not hard-coded: the robot is held at
   its balance point and the filtered angle is averaged over this many samples.
   The balance point is rarely exactly 0 deg, because the battery and the board
   are not centred over the wheel axle. */
#define SETPOINT_SETTLE  100     // 100 x 10 ms = 1 s for the filter to settle first
#define SETPOINT_SAMPLES 200     // then 200 x 10 ms = 2 s of averaging
#define SETPOINT_DELAY_MS 10U    // one sample per control period
#define KP               40.0f   // full output at about 2.5 deg of lean
#define KI               0.1f    // slow correction of small leftover error
#define KD               2.0f    // damping, based on how fast the error changes
#define INTEGRAL_LIMIT   200.0f  // anti-windup: with Ki=0.1 this caps the I term at +/-20%
#define OUTPUT_LIMIT     100.0f  // the PWM can never exceed 100% duty
#define I_DEADBAND       1.0f    // within 1 deg of the setpoint, bleed the integral away
#define I_DECAY          0.98f   // 2% per 10 ms step -> about a 0.5 s time constant

/* ===================== Motors ===================== */
#define MOTORS_ENABLED   1       // set to 0 to test the controller with motors off
#define MOTOR_DEADBAND   2.0f    // below this, just stop: too small to be useful
#define MOTOR_MIN_DUTY   20.0f   // a DC motor only starts turning above roughly 20%
#define R_DIR_SIGN       1.0f    // flip to -1.0f if the right wheel turns the wrong way
#define L_DIR_SIGN       1.0f    // flip to -1.0f if the left wheel turns the wrong way

/* ===================== LSM303AGR accelerometer (I2C1) ===================== */
#define ACC_ADDR         (0x19 << 1)  // 7-bit 0x19 -> 0x32 for HAL
#define WHO_AM_I_A       0x0F
#define ACC_ID           0x33
#define CTRL_REG1_A      0x20
#define CTRL_REG1_A_VAL  0x67         // 200 Hz ODR, X/Y/Z enabled
#define CTRL_REG4_A      0x23
#define CTRL_REG4_A_VAL  0x80         // block data update on, +/-2 g
#define OUT_X_L_A        0x28         // first of the 6 data registers
#define I2C_AUTO_INC     0x80         // read consecutive registers in one go
#define I2C_TIMEOUT_MS   10U
#define ACC_G_PER_LSB    (0.0039f / 64.0f)   // 10-bit left-justified, 3.9 mg/count

/* ===================== I3G4250D gyroscope (SPI1) ===================== */
#define WHO_AM_I_G       0x0F
#define GYRO_ID_BLUE     0xD3         // I3G4250D
#define GYRO_ID_GREEN    0xD4         // L3GD20 on the green board
#define CTRL_REG1_G      0x20
#define CTRL_REG1_G_VAL  0x5F         // 200 Hz ODR, powered on, X/Y/Z enabled
#define CTRL_REG4_G      0x23
#define CTRL_REG4_G_VAL  0x10         // +/-500 deg/s
#define OUT_X_L_G        0x28
#define SPI_READ_FLAG    0x80         // command bit 7 = 1 means read
#define SPI_AUTO_INC     0x40         // command bit 6 = 1 means auto-increment
#define GYRO_DPS_PER_LSB 0.0175f      // 17.5 mdps per count at +/-500 deg/s

/* ===================== Calibration ===================== */
#define CAL_SAMPLES      200          // readings averaged to find the offsets
#define CAL_DELAY_MS     5U           // gap between them (sensors run at 200 Hz)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

IMU_t  imu;    // latest sensor readings
Tilt_t tilt;   // latest angle estimates
PID_t  pid;    // controller state

float  g_setpoint = 0.0f;   // measured at start-up: the robot's balance angle

/* Written by the TIM6 ISR, read by the main loop, so they must be volatile */
volatile float   g_acc_angle  = 0.0f;
volatile float   g_gyro_angle = 0.0f;
volatile float   g_angle      = 0.0f;
volatile float   g_output     = 0.0f;
volatile uint8_t g_print_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* =======================================================================
   LAYER 1: UART output
   ======================================================================= */
void myPrintf(const char *fmt, ...)
{
    char buffer[100];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/* =======================================================================
   LAYER 2: talking to the two sensors
   ======================================================================= */

/* ---------- Accelerometer over I2C ---------- */
static HAL_StatusTypeDef acc_write_reg(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c1, ACC_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                             &value, 1, I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef acc_read_regs(uint8_t start_reg, uint8_t *buf, uint16_t len)
{
    if (len > 1)
    {
        start_reg |= I2C_AUTO_INC;   // 0x28 -> 0xA8: read 0x28, 0x29, ... in one transfer
    }
    return HAL_I2C_Mem_Read(&hi2c1, ACC_ADDR, start_reg, I2C_MEMADD_SIZE_8BIT,
                            buf, len, I2C_TIMEOUT_MS);
}

/* ---------- Gyroscope over SPI ---------- */
static void gyro_cs_low(void)
{
    HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);
}

static void gyro_cs_high(void)
{
    HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
}

static void gyro_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { reg, value };      // bit 7 = 0 means write

    gyro_cs_low();
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    gyro_cs_high();
}

static uint8_t gyro_read_reg(uint8_t reg)
{
    uint8_t cmd  = SPI_READ_FLAG | reg;  // bit 7 = 1 means read
    uint8_t data = 0;

    gyro_cs_low();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &data, 1, HAL_MAX_DELAY);
    gyro_cs_high();
    return data;
}

static void gyro_read_regs(uint8_t start_reg, uint8_t *buf, uint16_t len)
{
    uint8_t cmd = SPI_READ_FLAG | SPI_AUTO_INC | start_reg;   // 0x28 -> 0xE8

    gyro_cs_low();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, buf, len, HAL_MAX_DELAY);
    gyro_cs_high();
}

/* Join a low byte and a high byte into one signed 16-bit number */
static int16_t to_int16(uint8_t low, uint8_t high)
{
    return (int16_t)((high << 8) | low);
}

/* =======================================================================
   LAYER 3: IMU, giving readings in g and deg/s
   ======================================================================= */

/* Check both sensors answer, then switch them on */
HAL_StatusTypeDef IMU_Init(void)
{
    uint8_t acc_id  = 0;
    uint8_t gyro_id = 0;

    acc_read_regs(WHO_AM_I_A, &acc_id, 1);
    gyro_id = gyro_read_reg(WHO_AM_I_G);

    uint8_t acc_ok  = (acc_id == ACC_ID);
    uint8_t gyro_ok = (gyro_id == GYRO_ID_BLUE) || (gyro_id == GYRO_ID_GREEN);

    myPrintf("Accel WHO_AM_I = 0x%02X (%s)\r\n", acc_id,  acc_ok  ? "OK" : "MISMATCH");
    myPrintf("Gyro  WHO_AM_I = 0x%02X (%s)\r\n", gyro_id, gyro_ok ? "OK" : "MISMATCH");

    acc_write_reg(CTRL_REG1_A, CTRL_REG1_A_VAL);   // accelerometer on, 200 Hz
    acc_write_reg(CTRL_REG4_A, CTRL_REG4_A_VAL);   // +/-2 g
    gyro_write_reg(CTRL_REG1_G, CTRL_REG1_G_VAL);  // gyroscope on, 200 Hz
    gyro_write_reg(CTRL_REG4_G, CTRL_REG4_G_VAL);  // +/-500 deg/s

    HAL_Delay(100);   // let the sensors produce their first samples
    return (acc_ok && gyro_ok) ? HAL_OK : HAL_ERROR;
}

/* Read both sensors and convert to g and deg/s, with the offsets removed */
void IMU_Read(IMU_t *d)
{
    uint8_t buf[6];

    if (acc_read_regs(OUT_X_L_A, buf, 6) == HAL_OK)
    {
        for (int i = 0; i < 3; i++)
        {
            d->acc_g[i] = to_int16(buf[2 * i], buf[2 * i + 1]) * ACC_G_PER_LSB
                          - d->acc_bias[i];
        }
    }

    gyro_read_regs(OUT_X_L_G, buf, 6);
    for (int i = 0; i < 3; i++)
    {
        d->gyro_dps[i] = to_int16(buf[2 * i], buf[2 * i + 1]) * GYRO_DPS_PER_LSB
                         - d->gyro_bias[i];
    }
}

/* Offset calibration: average CAL_SAMPLES readings while the board is flat and
   still, and store each mean as that axis's offset.
   Flat and still means:
     gyro X, Y, Z  should read 0 deg/s  -> offset = mean
     acc  X, Y     should read 0 g      -> offset = mean
     acc  up axis  should read 1 g      -> offset = mean - 1 g, to keep gravity */
void IMU_Calibrate(IMU_t *d)
{
    float gyro_sum[3] = { 0.0f, 0.0f, 0.0f };
    float acc_sum[3]  = { 0.0f, 0.0f, 0.0f };

    for (int i = 0; i < 3; i++)
    {
        d->gyro_bias[i] = 0.0f;   // clear old offsets so we measure raw values
        d->acc_bias[i]  = 0.0f;
    }

    for (int n = 0; n < CAL_SAMPLES; n++)
    {
        IMU_Read(d);
        for (int i = 0; i < 3; i++)
        {
            gyro_sum[i] += d->gyro_dps[i];
            acc_sum[i]  += d->acc_g[i];
        }
        HAL_Delay(CAL_DELAY_MS);
    }

    for (int i = 0; i < 3; i++)
    {
        d->gyro_bias[i] = gyro_sum[i] / CAL_SAMPLES;   // the mean is the offset
        d->acc_bias[i]  = acc_sum[i]  / CAL_SAMPLES;
    }
    d->acc_bias[ACC_UP_AXIS] -= ACC_UP_SIGN * 1.0f;    // the up axis must still read 1 g

    myPrintf("Gyro offset = %.3f, %.3f, %.3f deg/s\r\n",
             d->gyro_bias[0], d->gyro_bias[1], d->gyro_bias[2]);
    myPrintf("Acc  offset = %.3f, %.3f, %.3f g\r\n",
             d->acc_bias[0], d->acc_bias[1], d->acc_bias[2]);
}

/* =======================================================================
   LAYER 4: tilt angle, using the complementary filter
   ======================================================================= */

/* Tilt angle from gravity alone. When the board tips, gravity is shared
   between the forward axis and the up axis, and atan2 recovers the angle. */
float Tilt_AccAngle(const IMU_t *d)
{
    float acc_fwd = ACC_FWD_SIGN * d->acc_g[ACC_FWD_AXIS];
    float acc_up  = ACC_UP_SIGN  * d->acc_g[ACC_UP_AXIS];

    return atan2f(acc_fwd, acc_up) * RAD_TO_DEG;
}

/* Start every estimate from the accelerometer, so the filter does not have to
   creep up from zero if the board starts out tilted. */
void Tilt_Init(Tilt_t *t, const IMU_t *d)
{
    t->acc_angle  = Tilt_AccAngle(d);
    t->gyro_angle = t->acc_angle;
    t->angle      = t->acc_angle;
}

/* The complementary filter, called once every DT seconds:
   angle = 0.98 * (angle + gyro_rate * dt) + 0.02 * acc_angle
   The 98% share follows the gyro, which is smooth and instant.
   The 2% share pulls gently towards the accelerometer, which cancels drift. */
void Tilt_Update(Tilt_t *t, const IMU_t *d)
{
    float rate = GYRO_SIGN * d->gyro_dps[GYRO_AXIS];   // deg/s about the hinge axis

    t->acc_angle   = Tilt_AccAngle(d);
    t->gyro_angle += rate * DT;                        // gyro only, kept for comparison
    t->angle       = ALPHA * (t->angle + rate * DT)
                     + (1.0f - ALPHA) * t->acc_angle;
}

/* Measure the setpoint: hold the robot at its balance point and average the
   filtered angle over SETPOINT_SAMPLES readings. Averaging removes the
   accelerometer noise, so the stored value is the true resting angle rather
   than whatever one noisy sample happened to say. */
float Measure_Setpoint(void)
{
    float sum = 0.0f;

    /* Let the filter settle first. Averaging straight away would include the
       transient while the estimate is still converging, which drags the
       measured setpoint away from the true resting angle. */
    for (int n = 0; n < SETPOINT_SETTLE; n++)
    {
        IMU_Read(&imu);
        Tilt_Update(&tilt, &imu);
        HAL_Delay(SETPOINT_DELAY_MS);
    }

    for (int n = 0; n < SETPOINT_SAMPLES; n++)
    {
        IMU_Read(&imu);
        Tilt_Update(&tilt, &imu);
        sum += tilt.angle;
        HAL_Delay(SETPOINT_DELAY_MS);
    }

    return sum / SETPOINT_SAMPLES;
}

/* =======================================================================
   LAYER 5: PID controller
   ======================================================================= */
void PID_Init(PID_t *p, float kp, float ki, float kd)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->integral   = 0.0f;
    p->prev_error = 0.0f;
    p->p_term = 0.0f;
    p->i_term = 0.0f;
    p->d_term = 0.0f;
    p->output = 0.0f;
}

/* One controller update. Returns the control signal in percent duty. */
float PID_Update(PID_t *p, float setpoint, float measured, float dt)
{
    float error = setpoint - measured;

    /* P: how far off we are right now */
    p->p_term = p->kp * error;

    /* D: how fast the error is changing, which damps the response */
    p->d_term = p->kd * (error - p->prev_error) / dt;
    p->prev_error = error;

    float out_pd = p->p_term + p->d_term;

    /* ---------------- I, with anti-windup ----------------
       Anti-windup 1 (conditional integration): only accumulate if the result
       would not keep the output saturated. Without this, holding the robot
       tilted drives the sum to its limit and the motors stay at full output
       long after the robot is released. */
    float candidate = p->integral + error * dt;
    float test_out  = out_pd + p->ki * candidate;

    if (fabsf(test_out) < OUTPUT_LIMIT || (test_out * error) < 0.0f)
    {
        p->integral = candidate;
    }

    /* Anti-windup 2 (leak): once we are sitting at the setpoint the error is
       zero, so the sum would never shrink on its own and the motors would keep
       turning. Bleed it away with a time constant of about 0.5 s. */
    if (fabsf(error) < I_DEADBAND)
    {
        p->integral *= I_DECAY;
    }

    /* Hard clamp as a final guard */
    if (p->integral >  INTEGRAL_LIMIT) p->integral =  INTEGRAL_LIMIT;
    if (p->integral < -INTEGRAL_LIMIT) p->integral = -INTEGRAL_LIMIT;
    p->i_term = p->ki * p->integral;

    /* Sum the three terms, then clamp to what the PWM can actually produce */
    float out = out_pd + p->i_term;
    if (out >  OUTPUT_LIMIT) out =  OUTPUT_LIMIT;
    if (out < -OUTPUT_LIMIT) out = -OUTPUT_LIMIT;

    p->output = out;
    return out;
}

/* =======================================================================
   LAYER 6: motor driver
   ======================================================================= */

/* Write the two direction pins of one motor.
   HIGH/LOW turns one way, LOW/HIGH the other, LOW/LOW stops. */
static void motor_set_direction(Motor_t m, GPIO_PinState in1, GPIO_PinState in2)
{
    if (m == MOTOR_RIGHT)
    {
        HAL_GPIO_WritePin(R_IN1_GPIO_Port, R_IN1_Pin, in2);
        HAL_GPIO_WritePin(R_IN2_GPIO_Port, R_IN2_Pin, in1);
    }
    else
    {
        HAL_GPIO_WritePin(L_IN1_GPIO_Port, L_IN1_Pin, in2);
        HAL_GPIO_WritePin(L_IN2_GPIO_Port, L_IN2_Pin, in1);
    }
}

/* Set the PWM duty of one motor. duty = CCR / (ARR + 1), and ARR is 999,
   so a percentage maps straight onto CCR = percent * 10. */
static void motor_set_duty(Motor_t m, float percent)
{
    if (percent < 0.0f)   percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;

    uint32_t ccr = (uint32_t)(percent * 10.0f);

    if (m == MOTOR_RIGHT)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ccr);   // PC6, right motor
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr);   // PA15, left motor
    }
}

/* Start both PWM channels with the motors stopped */
void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);   // right motor
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);   // left motor

    motor_set_duty(MOTOR_RIGHT, 0.0f);
    motor_set_duty(MOTOR_LEFT,  0.0f);
    motor_set_direction(MOTOR_RIGHT, GPIO_PIN_RESET, GPIO_PIN_RESET);
    motor_set_direction(MOTOR_LEFT,  GPIO_PIN_RESET, GPIO_PIN_RESET);
}

/* Drive one motor from a signed number:
     the SIGN picks the direction, the SIZE becomes the PWM duty. */
void Motor_SetSigned(Motor_t m, float signed_percent)
{
    /* Per-motor sign flip, so a wheel wired backwards is a one-character fix */
    signed_percent *= (m == MOTOR_RIGHT) ? R_DIR_SIGN : L_DIR_SIGN;

    float magnitude = fabsf(signed_percent);

    if (magnitude < MOTOR_DEADBAND)
    {
        /* Too small to be useful, so stop rather than make the motor buzz */
        motor_set_direction(m, GPIO_PIN_RESET, GPIO_PIN_RESET);
        motor_set_duty(m, 0.0f);
        return;
    }

    /* A geared DC motor does not turn below roughly 20% duty, so lift small
       commands to that floor. Set MOTOR_MIN_DUTY to 0 to remove this. */
    if (magnitude < MOTOR_MIN_DUTY)
    {
        magnitude = MOTOR_MIN_DUTY;
    }

    if (signed_percent > 0.0f)
    {
        motor_set_direction(m, GPIO_PIN_SET, GPIO_PIN_RESET);   // clockwise
    }
    else
    {
        motor_set_direction(m, GPIO_PIN_RESET, GPIO_PIN_SET);   // anticlockwise
    }

    motor_set_duty(m, magnitude);
}

/* Both motors together, which is what the balancing controller drives */
void Motor_SetBoth(float signed_percent)
{
#if MOTORS_ENABLED
    Motor_SetSigned(MOTOR_RIGHT, signed_percent);
    Motor_SetSigned(MOTOR_LEFT,  signed_percent);
#else
    (void)signed_percent;   // motors off: useful while checking the controller
#endif
}

/* =======================================================================
   LAYER 7: output to the terminal
   ======================================================================= */
void Print_Status(float acc_angle, float gyro_angle, float angle, float output)
{
#if   PLOT_MODE == 1
    /* CSV for plot.py: acc_angle,gyro_angle,filtered_angle */
    myPrintf("%.2f,%.2f,%.2f\r\n", acc_angle, gyro_angle, angle);
#elif PLOT_MODE == 2
    /* CSV with the controller output as a fourth value */
    myPrintf("%.2f,%.2f,%.2f,%.2f\r\n", acc_angle, gyro_angle, angle, output);
#else
    /* Readable, showing which axes each angle came from */
    const char axis_name[3] = { 'X', 'Y', 'Z' };
    myPrintf("Acc(%c,%c)=%7.2f  Gyro %c=%7.2f  Filt=%7.2f  PID=%7.2f %%\r\n",
             axis_name[ACC_FWD_AXIS], axis_name[ACC_UP_AXIS], acc_angle,
             axis_name[GYRO_AXIS], gyro_angle,
             angle, output);
#endif
}

void Print_Axes_Raw(const IMU_t *d)
{
    const char axis_name[3] = { 'X', 'Y', 'Z' };

    myPrintf("Acc %c = %6.2f g, Acc %c = %6.2f g, Gyro %c = %7.2f deg/s\r\n",
             axis_name[ACC_FWD_AXIS], ACC_FWD_SIGN * d->acc_g[ACC_FWD_AXIS],
             axis_name[ACC_UP_AXIS],  ACC_UP_SIGN  * d->acc_g[ACC_UP_AXIS],
             axis_name[GYRO_AXIS],    GYRO_SIGN    * d->gyro_dps[GYRO_AXIS]);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */

  myPrintf("\r\nLab 11 Task 2: tilt angle + PID motor control\r\n");

  /* 1. Check both sensors answer, then wake them */
  if (IMU_Init() != HAL_OK)
  {
      myPrintf("Sensor ID check failed\r\n");
  }

  /* 2. Measure the sensor offsets with the board flat and still */
  myPrintf("Calibrating: keep the board flat and still...\r\n");
  memset(&imu, 0, sizeof(imu));
  IMU_Calibrate(&imu);
  myPrintf("Calibration done\r\n");

  /* 3. Start the angle estimate from the accelerometer */
  IMU_Read(&imu);
  Tilt_Init(&tilt, &imu);

  /* 4. Measure the setpoint: the angle the robot sits at when balanced */
  myPrintf("Hold the robot at its balance point...\r\n");
  g_setpoint = Measure_Setpoint();
  myPrintf("Setpoint measured = %.2f deg\r\n", g_setpoint);

  /* 5. Set up the controller and the motors */
  PID_Init(&pid, KP, KI, KD);
  Motor_Init();
  myPrintf("PID: Kp=%.2f Ki=%.2f Kd=%.2f, setpoint=%.2f deg, dt=%.3f s\r\n",
           pid.kp, pid.ki, pid.kd, g_setpoint, (float)DT);

  /* 6. Start the 100 Hz control loop. Everything above had to finish first,
        because the ISR and the setup code both use the same I2C and SPI. */
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* The main loop only prints. The control loop lives in the TIM6 ISR,
       because UART transmission is slow and must not disturb its timing. */
    if (g_print_flag)
    {
        g_print_flag = 0;

        /* Copy the shared values with interrupts off, so the ISR cannot
           change them halfway through and give a mixed-up set */
        __disable_irq();
        float acc_angle  = g_acc_angle;
        float gyro_angle = g_gyro_angle;
        float angle      = g_angle;
        float output     = g_output;
        __enable_irq();

#if AXIS_TEST
        Print_Axes_Raw(&imu);
#else
        Print_Status(acc_angle, gyro_angle, angle, output);
#endif
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  /* Put SCK in its idle-high state now. SPI mode 3 expects the clock to be
     high before CS goes low, but HAL only enables SPI at the first transfer,
     which makes the very first gyro read come back wrong. */
  __HAL_SPI_ENABLE(&hspi1);
  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 47;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 479;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, R_IN1_Pin|R_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, L_IN2_Pin|L_IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R_IN1_Pin R_IN2_Pin */
  GPIO_InitStruct.Pin = R_IN1_Pin|R_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : L_IN2_Pin L_IN1_Pin */
  GPIO_InitStruct.Pin = L_IN2_Pin|L_IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* The gyro chip select must idle HIGH (not selected). CubeMX generates it
     LOW, which leaves the gyro selected from reset and garbles WHO_AM_I. */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* =======================================================================
   The fixed-rate control loop: TIM6 fires this every 10 ms (100 Hz).
   A hardware timer runs it, so dt is exactly DT no matter how busy the
   main loop is. That matters because the filter integrates gyro * dt and
   the PID divides by dt, so jitter in dt corrupts both.
   Keep this short: no printing and no HAL_Delay in here.
   ======================================================================= */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint32_t print_count = 0;

    if (htim->Instance != TIM6)
    {
        return;   // some other timer, not ours
    }

    /* Step 1: read the IMU */
    IMU_Read(&imu);

    /* Step 2: update the tilt angle with the complementary filter */
    Tilt_Update(&tilt, &imu);

    /* Step 3: the controller uses the filtered angle as its feedback */
    float output = PID_Update(&pid, g_setpoint, tilt.angle, DT);

    /* Step 4: drive both motors. Positive output turns them one way,
       negative the other, and the size sets the speed. */
    Motor_SetBoth(output);

    /* Step 5: hand the values to the main loop and ask it to print,
       at a slower rate than the control loop */
    g_acc_angle  = tilt.acc_angle;
    g_gyro_angle = tilt.gyro_angle;
    g_angle      = tilt.angle;
    g_output     = output;

    if (++print_count >= PRINT_EVERY)
    {
        print_count = 0;
        g_print_flag = 1;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */