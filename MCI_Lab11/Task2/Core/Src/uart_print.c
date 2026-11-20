/**
  ******************************************************************************
  * @file    uart_print.c
  * @brief   printf-style text output over a UART.
  *
  * The transmit is blocking, which is fine in the main loop but would wreck
  * the timing of the control loop, so nothing here is called from the ISR.
  ******************************************************************************
  */
#include "uart_print.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

static UART_HandleTypeDef *print_uart = NULL;

void UartPrint_Init(UART_HandleTypeDef *huart)
{
    print_uart = huart;
}

void uprintf(const char *fmt, ...)
{
    char buffer[120];
    va_list args;

    if (print_uart == NULL)
    {
        return;                      /* not set up yet: do nothing */
    }

    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    HAL_UART_Transmit(print_uart, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}
