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

typedef struct
{
    float acc_g[3];      // acceleration on X, Y, Z in g
    float gyro_dps[3];   // rotation speed on X, Y, Z in deg/s (bias removed)
    float gyro_bias[3];  // what the gyro reads when still (its error)
    float acc_bias[3];   // accelerometer offset measured while flat and still
} IMU_t;

typedef struct
{
    float acc_angle;     // angle from accelerometer only (noisy)
    float gyro_angle;    // angle from gyro only (drifts)
    float angle;         // complementary filter result (the good one)
} Tilt_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Timing: one dt used everywhere */
#define LOOP_MS          10U                    // loop period: 10 ms -> 100 Hz
#define DT               (LOOP_MS / 1000.0f)    // dt in seconds = 0.01
#define PRINT_EVERY      5U                     // print every 5th loop -> 20 Hz

/*Complementary filter */
#define ALPHA            0.98f                  // weight on the gyro
#define RAD_TO_DEG       57.2958f               // 180 / pi

/* Axis selection (confirmed by the axis test) */
#define AXIS_TEST        0      // 1 = print raw axes to find them, 0 = normal run
#define GYRO_AXIS        1       // 0 = X, 1 = Y, 2 = Z  (manual's pseudocode uses Y)
#define GYRO_SIGN        1.0f    // flip to -1.0f if gyro and accel disagree
#define ACC_FWD_AXIS     0       // axis that gains gravity when leaning forward
#define ACC_FWD_SIGN     1.0f
#define ACC_UP_AXIS      2       // axis that reads about 1 g when upright
#define ACC_UP_SIGN      1.0f

/* LSM303AGR accelerometer (I2C1) */
#define ACC_ADDR         (0x19 << 1)  // 7-bit 0x19 -> 0x32 for HAL
#define WHO_AM_I_A       0x0F
#define ACC_ID           0x33
#define CTRL_REG1_A      0x20
#define CTRL_REG1_A_VAL  0x67         // 200 Hz, X/Y/Z on
#define CTRL_REG4_A      0x23
#define CTRL_REG4_A_VAL  0x80         // block data update, +/-2 g
#define OUT_X_L_A        0x28         // first of 6 data registers
#define I2C_AUTO_INC     0x80         // read registers one after another
#define I2C_TIMEOUT_MS   10U
#define ACC_G_PER_LSB    (0.0039f / 64.0f)

/*I3G4250D gyroscope (SPI1)*/
#define WHO_AM_I_G       0x0F
#define GYRO_ID_BLUE     0xD3         // I3G4250D
#define GYRO_ID_GREEN    0xD4         // green board
#define CTRL_REG1_G      0x20
#define CTRL_REG1_G_VAL  0x5F         // 200 Hz, power on, X/Y/Z on
#define CTRL_REG4_G      0x23
#define CTRL_REG4_G_VAL  0x10         // +/-500 deg/s
#define OUT_X_L_G        0x28
#define SPI_READ_FLAG    0x80         // bit 7 = 1 means read
#define SPI_AUTO_INC     0x40         // bit 6 = 1 means auto-increment
#define GYRO_DPS_PER_LSB 0.0175f      // 17.5 mdps per count at +/-500
/* Calibration: board must be flat and still */
#define CAL_SAMPLES      500          // samples averaged for the offsets
#define CAL_DELAY_MS     5U           // gap between samples (sensors run at 200 Hz)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
IMU_t  imu;    // latest sensor readings
Tilt_t tilt;   // current angles
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void myPrintf(const char *fmt, ...)
{
    char buffer[1000];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);   // build the text
    va_end(args);
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}
// Accelerometer read function:I2C
static HAL_StatusTypeDef acc_write_reg(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c1, ACC_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                             &value, 1, I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef acc_read_regs(uint8_t start_reg, uint8_t *buf, uint16_t len)
{
    if (len > 1)
    {
        start_reg |= I2C_AUTO_INC;   // 0x28 -> 0xA8: read 0x28, 0x29, ... in one go
    }
    return HAL_I2C_Mem_Read(&hi2c1, ACC_ADDR, start_reg, I2C_MEMADD_SIZE_8BIT,
                            buf, len, I2C_TIMEOUT_MS);
}

/* Gyroscope: SPI */
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
    uint8_t tx[2] = { reg, value };      // bit 7 = 0 -> write

    gyro_cs_low();
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    gyro_cs_high();
}

static uint8_t gyro_read_reg(uint8_t reg)
{
    uint8_t cmd  = SPI_READ_FLAG | reg;  // bit 7 = 1 -> read
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

/* Combine a low and high byte into one signed 16-bit number */
static int16_t to_int16(uint8_t low, uint8_t high)
{
    return (int16_t)((high << 8) | low);
}

/*IMU Layer*/
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

    acc_write_reg(CTRL_REG1_A, CTRL_REG1_A_VAL);   // accel on, 200 Hz
    acc_write_reg(CTRL_REG4_A, CTRL_REG4_A_VAL);   // +/-2 g
    gyro_write_reg(CTRL_REG1_G, CTRL_REG1_G_VAL);  // gyro on, 200 Hz
    gyro_write_reg(CTRL_REG4_G, CTRL_REG4_G_VAL);  // +/-500 deg/s

    HAL_Delay(100);   // give the sensors time to produce first samples
    return (acc_ok && gyro_ok) ? HAL_OK : HAL_ERROR;
}

/* Read both sensors and convert to g and deg/s */
void IMU_Read(IMU_t *d)
{
    uint8_t buf[6];

    if (acc_read_regs(OUT_X_L_A, buf, 6) == HAL_OK)   // FIX: actually read the accelerometer
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

/* Offset calibration: take CAL_SAMPLES readings while the board is flat
   and still, find the mean of each axis, and store it as the offset.
   Flat and still means:
     gyro X, Y, Z should read 0 deg/s  -> offset = mean
     acc  X, Y    should read 0 g      -> offset = mean
     acc  Z       should read 1 g      -> offset = mean - 1 g (keep gravity) */
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
        HAL_Delay(CAL_DELAY_MS);   // wait for the next sample
    }

    for (int i = 0; i < 3; i++)
    {
        d->gyro_bias[i] = gyro_sum[i] / CAL_SAMPLES;   // mean = offset
        d->acc_bias[i]  = acc_sum[i]  / CAL_SAMPLES;
    }
    d->acc_bias[ACC_UP_AXIS] -= ACC_UP_SIGN * 1.0f;   // up axis must still read 1 g

    myPrintf("Gyro offset = %.3f, %.3f, %.3f deg/s\r\n",
             d->gyro_bias[0], d->gyro_bias[1], d->gyro_bias[2]);
    myPrintf("Acc  offset = %.3f, %.3f, %.3f g\r\n",
             d->acc_bias[0], d->acc_bias[1], d->acc_bias[2]);
}

/*Tilt Layer: complementary filter */
float Tilt_AccAngle(const IMU_t *d)
{
    float acc_fwd = ACC_FWD_SIGN * d->acc_g[ACC_FWD_AXIS];
    float acc_up  = ACC_UP_SIGN  * d->acc_g[ACC_UP_AXIS];

    return atan2f(acc_fwd, acc_up) * RAD_TO_DEG;   // forward lean = positive angle
}
void Tilt_Init(Tilt_t *t, IMU_t *d)
{
    t->acc_angle = Tilt_AccAngle(d);
    t->gyro_angle = t->acc_angle;
    t->angle      = t->acc_angle;
}

void Tilt_Update(Tilt_t *t, const IMU_t *d)
{
    t->acc_angle = Tilt_AccAngle(d);
    t->gyro_angle += GYRO_SIGN * d->gyro_dps[GYRO_AXIS] * DT;
    t->angle = ALPHA * (t->angle + GYRO_SIGN * d->gyro_dps[GYRO_AXIS] * DT)
               + (1.0f - ALPHA) * t->acc_angle;
}

/* Output */
#define PLOT_MODE  1   // 1 = plain CSV for plot.py, 0 = labelled text for screen

void Print_Angles(const Tilt_t *t)
{
    const char axis_name[3] = { 'X', 'Y', 'Z' };

#if PLOT_MODE
    /* CSV for plot.py: acc_angle,gyro_angle,filtered_angle */
    myPrintf("%.2f,%.2f,%.2f\r\n", t->acc_angle, t->gyro_angle, t->angle);
#else
    /* Readable: shows which axes each angle comes from */
    myPrintf("Acc(%c,%c) = %7.2f deg, Gyro %c = %7.2f deg, Filtered = %7.2f deg\r\n",
             axis_name[ACC_FWD_AXIS], axis_name[ACC_UP_AXIS], t->acc_angle,
             axis_name[GYRO_AXIS], t->gyro_angle,
             t->angle);
#endif
}

/* Axis test: print only the axes used for the tilt angle */
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
  MX_USB_PCD_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  myPrintf("\r\nLab 11 Task 1: tilt angle with a complementary filter\r\n");

  if (IMU_Init() != HAL_OK)
  {
      myPrintf("Sensor ID check failed\r\n");
  }

  myPrintf("Calibrating: keep the board flat and still...\r\n");
  memset(&imu, 0, sizeof(imu));   // start with zero offsets
  IMU_Calibrate(&imu);
  myPrintf("Calibration done\r\n");

  IMU_Read(&imu);          // first reading
  Tilt_Init(&tilt, &imu);  // start the angles from it

  uint32_t last_tick   = HAL_GetTick();   // time of the last update (ms)
  uint32_t print_count = 0;               // counts loops between prints
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    while ((HAL_GetTick() - last_tick) < LOOP_MS)
    {
        // wait for the next loop
    }
    last_tick += LOOP_MS;
    IMU_Read(&imu);
    Tilt_Update(&tilt, &imu);
    if (++print_count >= PRINT_EVERY)
    {
        print_count = 0;
        if (AXIS_TEST)
        {
            Print_Axes_Raw(&imu);
        }
        else
        {
            Print_Angles(&tilt);
        }
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
  __HAL_SPI_ENABLE(&hspi1);
  /* USER CODE END SPI1_Init 2 */

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

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* Gyro chip select idles HIGH (not selected) until we talk to it */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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