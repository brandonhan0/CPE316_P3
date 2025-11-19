#ifndef I2C_H

#define I2C_H

#include "stm32l4xx_hal.h" // so we can use HAL functions
#include <stdint.h>

void I2C_init(void);
HAL_StatusTypeDef I2C1_read(void);
HAL_StatusTypeDef I2C1_write(void);

#endif
