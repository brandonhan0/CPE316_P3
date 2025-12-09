#ifndef ADC_H

#define ADC_H

#include "stm32l476xx.h"
#include "stm32l4xx_hal.h" // so we can use HAL functions
#include <stdint.h>

void ADC_init(void);
uint16_t ADC1_read(void);
int adc_to_mV(uint16_t data);

#endif
