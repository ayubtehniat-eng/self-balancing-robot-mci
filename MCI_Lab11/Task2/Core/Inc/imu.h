/**
  ******************************************************************************
  * @file    imu.h
  * @brief   Device driver for the two on-board motion sensors.
  *
  *          LSM303AGR accelerometer  -- I2C1, 7-bit address 0x19
  *          I3G4250D  gyroscope      -- SPI1 mode 3, chip select on PE3
  *
  * The rest of the program never touches a sensor register. It asks this
  * module for acceleration in g and angular rate in degrees per second.
  ******************************************************************************
  */
#ifndef IMU_H
#define IMU_H

#include "main.h"

/* One set of readings, already converted to real units with offsets removed */
typedef struct
{
    float acc_g[3];       /* acceleration on X, Y, Z, in g                   */
    float gyro_dps[3];    /* angular rate on X, Y, Z, in degrees per second  */
    float acc_bias[3];    /* measured offsets, subtracted by IMU_Read()      */
    float gyro_bias[3];
} IMU_t;

/* Check both sensors answer on their bus, then power them up.
   Returns HAL_OK only if both WHO_AM_I registers held the expected value. */
HAL_StatusTypeDef IMU_Init(I2C_HandleTypeDef *hi2c, SPI_HandleTypeDef *hspi);

/* Take one reading from each sensor. Fast enough for the control ISR. */
void IMU_Read(IMU_t *imu);

/* Average many readings while the robot is held still, and store the result
   as the offsets. The up axis keeps 1 g, because gravity is not an error. */
void IMU_Calibrate(IMU_t *imu);

/* Readable names for the axes, for printing */
char IMU_AxisName(int axis);

#endif /* IMU_H */
