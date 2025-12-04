#ifndef I2C_H

#define I2C_H

#include "stm32l4xx_hal.h" // so we can use HAL functions
#include <stdint.h>

typedef enum {
    I2C_OK = 0,
    I2C_ERROR,
    I2C_TIMEOUT_ERR
} I2C_Status;

#define I2C_TIMEOUT  (100000U)

void I2C_init(void);
I2C_Status I2C1_read(uint16_t device_address, uint8_t register_address, uint8_t *data, uint16_t size);
I2C_Status I2C1_write(uint16_t device_address, uint8_t register_address, uint8_t *data, uint16_t size);

#endif
