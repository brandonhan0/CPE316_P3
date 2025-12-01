#ifndef UART_H

#define UART_H

#include "stm32l4xx_hal.h" // so we can use HAL functions
#include <stdint.h>

void UART2_INIT(uint32_t pclk1_hz, uint32_t baud);
void UART_write(const char *s);

#endif
