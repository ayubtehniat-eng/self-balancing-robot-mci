/**
  ******************************************************************************
  * @file    uart_print.h
  * @brief   printf-style text output over a UART.
  ******************************************************************************
  */
#ifndef UART_PRINT_H
#define UART_PRINT_H

#include "main.h"

/* Remember which UART to send on. Call once, before any uprintf(). */
void UartPrint_Init(UART_HandleTypeDef *huart);

/* Same usage as printf. Blocking, so never call this from inside an ISR. */
void uprintf(const char *fmt, ...);

#endif /* UART_PRINT_H */
