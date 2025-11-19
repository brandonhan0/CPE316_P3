#include "i2c.h"

// for i2c reads we need to add the device address which would be sensor address, the register address which would be the analog signal

void I2C1_init(void){
	RCC->APB1ENR1 |= (1 << 21);
	RCC->AHB2ENR  |= (RCC_AHB2ENR_GPIOBEN);

	GPIOB->MODER &= ~((3 << 16) | (3 << 18)); // clear
	GPIOB->MODER |=  ((2 << 16) | (2 << 18));
	GPIOB->OTYPER |= ((1 << 8) | (1 << 9)); // want open drain for i2c

	GPIOB->PUPDR &= ~((3 << 16) | (3 << 18));
	GPIOB->PUPDR |=  ((1 << 16) | (1 << 18));

	GPIOB->OSPEEDR &= ~((3 << 16) | (3 << 18));
	GPIOB->OSPEEDR |=  ((3 << 16) | (3 << 18)); // supa fast
	GPIOB->AFR[1] &= ~((0xF << 0) | (0xF << 4)); // clear bits
	GPIOB->AFR[1] |=  ((0x4 << 0) | (0x4 << 4)); // AF4 = I2C1
}

HAL_StatusTypeDef I2C1_read(uint16_t device_address, uint8_t register_address, uint8_t *data, uint16_t size){
    HAL_StatusTypeDef ret;
    ret = HAL_I2C_Master_Transmit(&hi2c1, device_address, &register_address, 1, HAL_MAX_DELAY); // sends address we wanna read from
    if (ret != HAL_OK){
        return ret; // if address is fake :(
    }
    ret = HAL_I2C_Master_Receive(&hi2c1, device_address, data, size, HAL_MAX_DELAY); // reads data from that device
    return ret;
}

HAL_StatusTypeDef I2C1_write(uint16_t device_address, uint8_t register_address, uint8_t *data, uint16_t size){
    HAL_StatusTypeDef ret;
    uint8_t buffer[32]; // adjust if needed
    if (Size + 1 > sizeof(buffer))
        return HAL_ERROR; // prevents buffer overflow
    buffer[0] = register_address; // first byte is register address followed by data
    memcpy(&buffer[1], data, size);
    ret = HAL_I2C_Master_Transmit(&hi2c1, register_address, buffer, size + 1, HAL_MAX_DELAY);
    return ret;
}
