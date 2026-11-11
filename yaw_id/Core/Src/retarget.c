/**
  ******************************************************************************
  * @file    retarget.c
  * @brief   Disable ARM semihosting and route printf/puts to a UART.
  *
  * Why this file exists
  * --------------------
  * With the full ARM C library (MicroLIB unchecked) the C runtime calls
  * _initio() during __rt_lib_init, which does freopen(":tt", ...) on the
  * stdio streams. _sys_open() then issues a semihosting call, i.e. the
  * Thumb instruction  BKPT 0xAB. If the debugger does not service
  * semihosting, the CPU halts there before main() is ever reached.
  *
  * __use_no_semihosting makes the linker pick the *_hlt library variants,
  * so no BKPT 0xAB is ever executed. _sys_exit and _ttywrch are then
  * mandatory symbols, provided below.
  *
  * Toolchain: ARM Compiler 6 (armclang) / Keil MDK
  * Add this file to a project group in uVision (e.g. Application/User/Core).
  ******************************************************************************
  */

#include <stdio.h>
#include "usart.h"

/* No semihosting code may be linked in. */
__asm(".global __use_no_semihosting\n\t");

/* Required by __use_no_semihosting. */
void _sys_exit(int x) { (void)x; }
void _ttywrch(int ch) { (void)ch; }

/* Stream objects the library expects to find. */
FILE __stdout;
FILE __stderr;

/* ---- pick a UART that is NOT used by any protocol ---- */
/* Yaw-ID: UART8 is binary VOFA; diagnostic text uses unused UART3. */
/* UART3 has no board-to-board task in this standalone target. */
#define LOG_UART huart3

int fputc(int ch, FILE *f)
{
    (void)f;
    HAL_UART_Transmit(&LOG_UART, (unsigned char *)&ch, 1, 0xFFFF);
    return ch;
}

int fgetc(FILE *f)
{
    (void)f;
    return -1;
}
