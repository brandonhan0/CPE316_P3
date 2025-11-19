#ifndef DAC_H

#define DAC_H

#include "stm32l4xx_hal.h" // so we can use HAL functions
#include <stdint.h>

void DAC_write(uint16_t data);
uint16_t DAC_volt_conv(int mv);
void DAC_init(void);

#endif
