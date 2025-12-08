#ifndef I2C_H

#define I2C_H

#include "stm32l4xx_hal.h" // so we can use HAL functions
#include <stdint.h>

static void I2C3_Init(void);
static void I2C3_Read(uint8_t addr7, uint8_t *data, uint8_t len)
static void I2C3_Write(uint8_t addr7, const uint8_t *data, uint8_t len);

#endif
