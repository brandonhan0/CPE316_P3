#ifndef USART_H

#define USART_H

#include "stm32l4xx_hal.h" // so we can use HAL functions
#include <stdint.h>

void USART2_INIT(void);
void USART_write(const char *s);

#endif
