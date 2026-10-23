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
    int16_t x, y, z;
} Vec3i_t;                 // three raw sensor counts

typedef struct
{
    float x, y, z;
} Vec3f_t;                 // three values in physical units

typedef struct
{
    Vec3i_t acc_raw;       // raw accelerometer counts (16-bit, left-justified)
    Vec3i_t gyro_raw;      // raw gyroscope counts
    Vec3f_t acc;           // acceleration in g, offset removed
    Vec3f_t gyro;          // angular rate in deg/s, offset removed
    Vec3f_t acc_offset;    // accelerometer bias found by Offset_LSM()
    Vec3f_t gyro_offset;   // gyroscope bias found by Offset_LSM()
    float roll;            // tilt about X from the accelerometer, degrees
    float pitch;           // tilt about Y from the accelerometer, degrees
} LSM_Data_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ---- LSM303AGR accelerometer on I2C1 (PB6/PB7) ---- */
#define ACC_ADDR_W        0x32    // 7-bit 0x19 << 1, R/W = 0 (write)
#define ACC_ADDR_R        0x33    // 7-bit 0x19 << 1, R/W = 1 (read)
#define WHO_AM_I_A        0x0F
#define ACC_ID            0x33
#define CTRL_REG1_A       0x20
#define CTRL_REG1_A_VAL   0x67    // 0110 0111: 200 Hz, normal power, X/Y/Z on
#define CTRL_REG4_A       0x23
#define CTRL_REG4_A_VAL   0x00    // +/-2 g, normal mode (10-bit data)
#define OUT_X_L_A         0x28    // first of 6 output registers (0x28-0x2D)
#define I2C_AUTO_INC      0x80    // MSB of register address -> auto-increment
#define ACC_MG_PER_LSB    3.9f    // normal mode, +/-2 g (datasheet)
#define I2C_TIMEOUT_MS    100

/* ---- I3G4250D gyroscope on SPI1 (from Lab 8) ---- */
#define SPI_READ_FLAG     0x80    // bit 7 = 1 -> read
#define SPI_AUTO_INC      0x40    // bit 6 = 1 -> auto-increment
#define WHO_AM_I_G        0x0F
#define GYRO_ID           0xD3    // I3G4250D (blue board)
#define CTRL_REG1_G       0x20
#define CTRL_REG1_G_VAL   0x8F    // power on, X/Y/Z on (Lab 8 Task 3 value)
#define CTRL_REG4_G       0x23
#define CTRL_REG4_G_VAL   0x00    // +/-245 dps
#define OUT_X_L_G         0x28    // first of 6 output registers (0x28-0x2D)
#define GYRO_DPS_PER_LSB  0.00875f  // 8.75 mdps per count at +/-245 dps

/* ---- Processing ---- */
#define RAD_TO_DEG        57.2958f
#define OFFSET_SAMPLES    20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
LSM_Data_t lsm;            // all sensor data lives here
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ===================== UART ===================== */
void myPrintf(const char *fmt, ...)
{
    char buffer[100];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/* ============ Accelerometer: I2C register access ============ */
static HAL_StatusTypeDef acc_write_reg(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c1, ACC_ADDR_W, reg, I2C_MEMADD_SIZE_8BIT,
                             &value, 1, I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef acc_read_regs(uint8_t start_reg, uint8_t *buf, uint16_t len)
{
    if (len > 1)
    {
        start_reg |= I2C_AUTO_INC;   // 0x28 -> 0xA8: read 0x28, 0x29, ... in one go
    }
    return HAL_I2C_Mem_Read(&hi2c1, ACC_ADDR_R, start_reg, I2C_MEMADD_SIZE_8BIT,
                            buf, len, I2C_TIMEOUT_MS);
}

/* ============ Gyroscope: SPI register access (Lab 8) ============ */
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
    uint8_t tx[2] = { reg, value };            // bit 7 = 0 -> write

    gyro_cs_low();
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    gyro_cs_high();
}

static uint8_t gyro_read_reg(uint8_t reg)
{
    uint8_t cmd  = SPI_READ_FLAG | reg;
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

/* ===================== Helpers ===================== */
static int16_t to_int16(uint8_t low, uint8_t high)
{
    return (int16_t)((high << 8) | low);      // raw = (high << 8) | low
}

static float acc_to_g(int16_t raw)
{
    /* Normal mode gives 10-bit data, left-justified in 16 bits:
       drop the 6 unused bits (/64), then 3.9 mg per count, then mg -> g */
    return (raw / 64.0f) * ACC_MG_PER_LSB / 1000.0f;
}

static float gyro_to_dps(int16_t raw)
{
    return raw * GYRO_DPS_PER_LSB;
}

/* Read all six raw values from both sensors into the struct */
static void read_raw(LSM_Data_t *d)
{
    uint8_t buf[6];

    if (acc_read_regs(OUT_X_L_A, buf, 6) == HAL_OK)
    {
        d->acc_raw.x = to_int16(buf[0], buf[1]);
        d->acc_raw.y = to_int16(buf[2], buf[3]);
        d->acc_raw.z = to_int16(buf[4], buf[5]);
    }

    gyro_read_regs(OUT_X_L_G, buf, 6);
    d->gyro_raw.x = to_int16(buf[0], buf[1]);
    d->gyro_raw.y = to_int16(buf[2], buf[3]);
    d->gyro_raw.z = to_int16(buf[4], buf[5]);
}

/* ===================== The four lab functions ===================== */

/* Init_LSM: check IDs, wake the accelerometer and the gyroscope */
void Init_LSM(LSM_Data_t *d)
{
    uint8_t id = 0;

    memset(d, 0, sizeof(*d));

    acc_read_regs(WHO_AM_I_A, &id, 1);
    myPrintf("Accel WHO_AM_I = 0x%02X (%s)\r\n", id, (id == ACC_ID) ? "OK" : "MISMATCH");

    acc_write_reg(CTRL_REG1_A, CTRL_REG1_A_VAL);   // wake up: 200 Hz, X/Y/Z on
    acc_write_reg(CTRL_REG4_A, CTRL_REG4_A_VAL);   // +/-2 g, normal mode

    id = gyro_read_reg(WHO_AM_I_G);
    myPrintf("Gyro  WHO_AM_I = 0x%02X (%s)\r\n", id, (id == GYRO_ID) ? "OK" : "MISMATCH");

    gyro_write_reg(CTRL_REG1_G, CTRL_REG1_G_VAL);  // power on, X/Y/Z on
    gyro_write_reg(CTRL_REG4_G, CTRL_REG4_G_VAL);  // +/-245 dps

    HAL_Delay(100);                                // wait for first valid samples
}

/* Offset_LSM: average stationary readings to find each sensor's bias */
void Offset_LSM(LSM_Data_t *d)
{
    Vec3f_t acc_sum  = {0};
    Vec3f_t gyro_sum = {0};

    for (int i = 0; i < OFFSET_SAMPLES; i++)
    {
        read_raw(d);

        acc_sum.x  += acc_to_g(d->acc_raw.x);
        acc_sum.y  += acc_to_g(d->acc_raw.y);
        acc_sum.z  += acc_to_g(d->acc_raw.z);

        gyro_sum.x += gyro_to_dps(d->gyro_raw.x);
        gyro_sum.y += gyro_to_dps(d->gyro_raw.y);
        gyro_sum.z += gyro_to_dps(d->gyro_raw.z);

        HAL_Delay(10);                             // let new samples arrive
    }

    /* Flat and still: X and Y should read 0 g, Z should read 1 g (gravity).
       So the Z offset is whatever is left over after removing gravity. */
    float mean_z = acc_sum.z / OFFSET_SAMPLES;

    d->acc_offset.x = acc_sum.x / OFFSET_SAMPLES;
    d->acc_offset.y = acc_sum.y / OFFSET_SAMPLES;
    d->acc_offset.z = mean_z - ((mean_z >= 0.0f) ? 1.0f : -1.0f);

    /* Still: every gyro axis should read 0 deg/s, so the whole mean is bias */
    d->gyro_offset.x = gyro_sum.x / OFFSET_SAMPLES;
    d->gyro_offset.y = gyro_sum.y / OFFSET_SAMPLES;
    d->gyro_offset.z = gyro_sum.z / OFFSET_SAMPLES;

    myPrintf("acc offset (g):    %.3f, %.3f, %.3f\r\n",
             d->acc_offset.x, d->acc_offset.y, d->acc_offset.z);
    myPrintf("gyro offset (dps): %.3f, %.3f, %.3f\r\n",
             d->gyro_offset.x, d->gyro_offset.y, d->gyro_offset.z);
}

/* Read_LSM: read, scale to units, remove offsets, compute tilt angles */
void Read_LSM(LSM_Data_t *d)
{
    read_raw(d);

    d->acc.x  = acc_to_g(d->acc_raw.x) - d->acc_offset.x;
    d->acc.y  = acc_to_g(d->acc_raw.y) - d->acc_offset.y;
    d->acc.z  = acc_to_g(d->acc_raw.z) - d->acc_offset.z;

    d->gyro.x = gyro_to_dps(d->gyro_raw.x) - d->gyro_offset.x;
    d->gyro.y = gyro_to_dps(d->gyro_raw.y) - d->gyro_offset.y;
    d->gyro.z = gyro_to_dps(d->gyro_raw.z) - d->gyro_offset.z;

    /* Tilt from the direction of gravity; atan2 gives radians -> degrees */
    d->roll  = atan2f(d->acc.y, d->acc.z) * RAD_TO_DEG;
    d->pitch = atan2f(-d->acc.x, sqrtf(d->acc.y * d->acc.y + d->acc.z * d->acc.z))
               * RAD_TO_DEG;
}

/* Print_LSM: one comma-separated line per reading */
void Print_LSM(const LSM_Data_t *d)
{
    /* ax, ay, az in g */
    myPrintf("%.2f,%.2f,%.2f\r\n", d->acc.x, d->acc.y, d->acc.z);
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
  MX_USART2_UART_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */
  gyro_cs_high();                    // PE3 starts low: deselect the gyro first
  HAL_Delay(100);                    // let both sensors finish booting

  myPrintf("\r\nLab 9 Task 2: LSM303AGR accel + I3G4250D gyro\r\n");
  Init_LSM(&lsm);

  myPrintf("Calibrating, keep the board flat and still...\r\n");
  Offset_LSM(&lsm);

  myPrintf("ax,ay,az\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Read_LSM(&lsm);
    Print_LSM(&lsm);
    HAL_Delay(100);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
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
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  /* The values above are the board's defaults, which don't suit the gyro.
     Re-apply the Lab 8 settings: 8-bit frames, SPI mode 3 (CPOL high,
     CPHA 2nd edge), 48 MHz / 8 = 6 MHz (gyro max is 10 MHz), no NSS pulse.
     If you set SPI1 to these values in CubeMX, this block can be deleted. */
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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