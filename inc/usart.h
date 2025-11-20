#ifndef USART_H

#define USART_H

#include "stm32l4xx_hal.h" // so we can use HAL functions
#include <stdint.h>

void USART_init(uint32_t pclk_hz, uint32_t baud);
void USART_write(const char *s);

#endif
