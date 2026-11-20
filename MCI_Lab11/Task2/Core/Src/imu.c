/**
  ******************************************************************************
  * @file    imu.c
  * @brief   Device driver for the LSM303AGR accelerometer and I3G4250D gyro.
  ******************************************************************************
  */
#include "imu.h"
#include "config.h"
#include "uart_print.h"
#include <string.h>

/* ---------------- LSM303AGR accelerometer, on I2C1 ---------------- */
#define ACC_ADDR          (0x19 << 1)   /* HAL wants the address pre-shifted */
#define ACC_WHO_AM_I      0x0F
#define ACC_ID            0x33
#define ACC_CTRL_REG1     0x20
#define ACC_CTRL_REG1_VAL 0x67          /* 200 Hz output rate, X/Y/Z enabled */
#define ACC_CTRL_REG4     0x23
#define ACC_CTRL_REG4_VAL 0x80          /* block data update on, +/-2 g      */
#define ACC_OUT_X_L       0x28
#define ACC_AUTO_INC      0x80          /* read consecutive registers        */
#define ACC_TIMEOUT_MS    10U
/* Normal mode gives 10 bits left-justified inside 16, at 3.9 mg per count   */
#define ACC_G_PER_LSB     (0.0039f / 64.0f)

/* ---------------- I3G4250D gyroscope, on SPI1 ---------------- */
#define GYRO_WHO_AM_I      0x0F
#define GYRO_ID_BLUE       0xD3         /* I3G4250D                          */
#define GYRO_ID_GREEN      0xD4         /* L3GD20 on the green board         */
#define GYRO_CTRL_REG1     0x20
#define GYRO_CTRL_REG1_VAL 0x5F         /* 200 Hz, powered on, X/Y/Z enabled */
#define GYRO_CTRL_REG4     0x23
#define GYRO_CTRL_REG4_VAL 0x10         /* +/-500 deg/s                      */
#define GYRO_OUT_X_L       0x28
#define GYRO_READ_FLAG     0x80         /* command bit 7 = read              */
#define GYRO_AUTO_INC      0x40         /* command bit 6 = auto-increment    */
#define GYRO_DPS_PER_LSB   0.0175f      /* 17.5 mdps per count at +/-500     */

static I2C_HandleTypeDef *acc_bus  = NULL;
static SPI_HandleTypeDef *gyro_bus = NULL;

/* ===================== Low-level bus access ============================== */

static HAL_StatusTypeDef acc_write_reg(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(acc_bus, ACC_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                             &value, 1, ACC_TIMEOUT_MS);
}

static HAL_StatusTypeDef acc_read_regs(uint8_t start_reg, uint8_t *buf, uint16_t len)
{
    if (len > 1)
    {
        start_reg |= ACC_AUTO_INC;   /* 0x28 -> 0xA8: one transfer, six bytes */
    }
    return HAL_I2C_Mem_Read(acc_bus, ACC_ADDR, start_reg, I2C_MEMADD_SIZE_8BIT,
                            buf, len, ACC_TIMEOUT_MS);
}

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
    uint8_t tx[2] = { reg, value };   /* bit 7 clear means write */

    gyro_cs_low();
    HAL_SPI_Transmit(gyro_bus, tx, 2, HAL_MAX_DELAY);
    gyro_cs_high();
}

static uint8_t gyro_read_reg(uint8_t reg)
{
    uint8_t cmd  = GYRO_READ_FLAG | reg;
    uint8_t data = 0;

    gyro_cs_low();
    HAL_SPI_Transmit(gyro_bus, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(gyro_bus, &data, 1, HAL_MAX_DELAY);
    gyro_cs_high();

    return data;
}

static void gyro_read_regs(uint8_t start_reg, uint8_t *buf, uint16_t len)
{
    uint8_t cmd = GYRO_READ_FLAG | GYRO_AUTO_INC | start_reg;   /* 0x28 -> 0xE8 */

    gyro_cs_low();
    HAL_SPI_Transmit(gyro_bus, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(gyro_bus, buf, len, HAL_MAX_DELAY);
    gyro_cs_high();
}

/* Each axis arrives as a low byte and a high byte, in two's complement */
static int16_t to_int16(uint8_t low, uint8_t high)
{
    return (int16_t)(((uint16_t)high << 8) | low);
}

/* ===================== Public interface ================================== */

HAL_StatusTypeDef IMU_Init(I2C_HandleTypeDef *hi2c, SPI_HandleTypeDef *hspi)
{
    uint8_t acc_id  = 0;
    uint8_t gyro_id = 0;

    acc_bus  = hi2c;
    gyro_bus = hspi;

    /* Identify both sensors before configuring them. A wrong ID means the
       bus settings or the wiring are wrong, and nothing after this is valid. */
    acc_read_regs(ACC_WHO_AM_I, &acc_id, 1);
    gyro_id = gyro_read_reg(GYRO_WHO_AM_I);

    uint8_t acc_ok  = (acc_id == ACC_ID);
    uint8_t gyro_ok = (gyro_id == GYRO_ID_BLUE) || (gyro_id == GYRO_ID_GREEN);

    uprintf("Accel WHO_AM_I = 0x%02X (%s)\r\n", acc_id,  acc_ok  ? "OK" : "MISMATCH");
    uprintf("Gyro  WHO_AM_I = 0x%02X (%s)\r\n", gyro_id, gyro_ok ? "OK" : "MISMATCH");

    /* Both sensors power up asleep; these writes start them sampling.
       200 Hz is twice the control loop, so every read returns fresh data. */
    acc_write_reg(ACC_CTRL_REG1, ACC_CTRL_REG1_VAL);
    acc_write_reg(ACC_CTRL_REG4, ACC_CTRL_REG4_VAL);
    gyro_write_reg(GYRO_CTRL_REG1, GYRO_CTRL_REG1_VAL);
    gyro_write_reg(GYRO_CTRL_REG4, GYRO_CTRL_REG4_VAL);

    HAL_Delay(100);   /* let the first samples appear */

    return (acc_ok && gyro_ok) ? HAL_OK : HAL_ERROR;
}

void IMU_Read(IMU_t *imu)
{
    uint8_t buf[6];

    /* If the I2C read fails the previous values are kept, which is safer than
       feeding the controller a buffer full of rubbish. */
    if (acc_read_regs(ACC_OUT_X_L, buf, 6) == HAL_OK)
    {
        for (int i = 0; i < 3; i++)
        {
            imu->acc_g[i] = to_int16(buf[2 * i], buf[2 * i + 1]) * ACC_G_PER_LSB
                            - imu->acc_bias[i];
        }
    }

    gyro_read_regs(GYRO_OUT_X_L, buf, 6);
    for (int i = 0; i < 3; i++)
    {
        imu->gyro_dps[i] = to_int16(buf[2 * i], buf[2 * i + 1]) * GYRO_DPS_PER_LSB
                           - imu->gyro_bias[i];
    }
}

void IMU_Calibrate(IMU_t *imu)
{
    float acc_sum[3]  = { 0.0f, 0.0f, 0.0f };
    float gyro_sum[3] = { 0.0f, 0.0f, 0.0f };

    /* Clear the old offsets so this measures the raw values */
    memset(imu->acc_bias,  0, sizeof(imu->acc_bias));
    memset(imu->gyro_bias, 0, sizeof(imu->gyro_bias));

    for (int n = 0; n < CAL_SAMPLES; n++)
    {
        IMU_Read(imu);
        for (int i = 0; i < 3; i++)
        {
            acc_sum[i]  += imu->acc_g[i];
            gyro_sum[i] += imu->gyro_dps[i];
        }
        HAL_Delay(CAL_DELAY_MS);
    }

    for (int i = 0; i < 3; i++)
    {
        imu->acc_bias[i]  = acc_sum[i]  / CAL_SAMPLES;
        imu->gyro_bias[i] = gyro_sum[i] / CAL_SAMPLES;
    }

    /* Held still, every gyro axis should read zero, so its whole average is
       error. The accelerometer is different: the up axis should still read
       1 g afterwards, because gravity is signal and not an offset. */
    imu->acc_bias[ACC_UP_AXIS] -= ACC_UP_SIGN * 1.0f;

    uprintf("Gyro offset = %.3f, %.3f, %.3f deg/s\r\n",
            imu->gyro_bias[0], imu->gyro_bias[1], imu->gyro_bias[2]);
    uprintf("Acc  offset = %.3f, %.3f, %.3f g\r\n",
            imu->acc_bias[0], imu->acc_bias[1], imu->acc_bias[2]);
}

char IMU_AxisName(int axis)
{
    const char names[3] = { 'X', 'Y', 'Z' };
    return (axis >= 0 && axis < 3) ? names[axis] : '?';
}
