#ifndef PWM_H
#define PWM_H

#include "stm32l4xx_hal.h" // so we can use HAL functions
#include <stdint.h>

void PWM_init(void);
uint16_t PWM_set_duty(float duty);

#endif
