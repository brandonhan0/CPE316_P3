#ifndef I2C_H

#define I2C_H

#include "stm32l4xx_hal.h" // so we can use HAL functions
#include <stdint.h>

void I2C_init(void);
HAL_StatusTypeDef I2C1_read(uint16_t device_address, uint8_t register_address, uint8_t *data, uint16_t size);
HAL_StatusTypeDef I2C1_write(uint16_t device_address, uint8_t register_address, uint8_t *data, uint16_t size);

#endif
