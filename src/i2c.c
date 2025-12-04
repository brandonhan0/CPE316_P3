#include "i2c.h"

// for i2c reads we need to add the device address which would be sensor address, the register address which would be the analog signal

void i2c3_init(void){
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
	GPIOB->AFR[1] |=  ((0x4 << 0) | (0x4 << 4)); // AF4 = i2c3

	I2C3->CR1 &= ~I2C_CR1_PE;
    I2C3->TIMINGR = 0x00303D5B; // need to change this clock i gotta look it up based on our APB clock were gonna run at 4MHz
    I2C3->CR1 &= ~(I2C_CR1_ANFOFF);
    I2C3->CR1 &= ~(I2C_CR1_DNF);
    I2C3->CR1 |= I2C_CR1_PE;
}

static I2C_Status I2C3_WaitFlagSet(volatile uint32_t *reg, uint32_t flag){
    uint32_t timeout = I2C_TIMEOUT;
    while(((*reg) & flag) == 0U) {
        if (--timeout == 0U) {
            return I2C_TIMEOUT_ERR;
        }
    }
    return I2C_OK;
}

static I2C_Status I2C3_WaitFlagClear(volatile uint32_t *reg, uint32_t flag){
    uint32_t timeout = I2C_TIMEOUT;
    while(((*reg) & flag) != 0U) {
        if (--timeout == 0U) {
            return I2C_TIMEOUT_ERR;
        }
    }
    return I2C_OK;
}

I2C_Status I2C3_write(uint16_t device_address, uint8_t register_address, uint8_t *data, uint16_t size){ // writes data, doesnt really get anything back
	
    I2C_Status ret;
    if (size == 0) {
        return I2C_OK;
    }
    if (size > 255) {
        return I2C_ERROR;
    }
	
    ret = I2C3_WaitFlagClear(&i2c3->ISR, I2C_ISR_BUSY); // wait until bus is free
    if (ret != I2C_OK) return ret;
	// clear flags
    if (i2c3->ISR & I2C_ISR_NACKF) {
        i2c3->ICR = I2C_ICR_NACKCF;
    }
    if (i2c3->ISR & I2C_ISR_STOPF) {
        i2c3->ICR = I2C_ICR_STOPCF;
    }
    i2c3->CR2 = 0;
    i2c3->CR2 |= ((uint32_t)(device_address & 0x7FU) << 1); // 7 bit address
    i2c3->CR2 |= ((uint32_t)(size + 1) << I2C_CR2_NBYTES_Pos); // write direction
    i2c3->CR2 &= ~I2C_CR2_RD_WRN; // bytes
    i2c3->CR2 |= I2C_CR2_AUTOEND; // auto end

    i2c3->CR2 |= I2C_CR2_START;

    ret = I2C3_WaitFlagSet(&i2c3->ISR, I2C_ISR_TXIS); // send register address
    if (ret != I2C_OK) return ret;
    i2c3->TXDR = register_address;
 
    for (uint16_t i = 0; i < size; i++){ // send data bits
        ret = i2c3_WaitFlagSet(&i2c3->ISR, I2C_ISR_TXIS);
        if (ret != I2C_OK) return ret;
        i2c3->TXDR = data[i];
    }
    ret = I2C3_WaitFlagSet(&i2c3->ISR, I2C_ISR_STOPF); // wait
    if (ret != I2C_OK) return ret;

    i2c3->ICR = I2C_ICR_STOPCF;

    if (i2c3->ISR & I2C_ISR_NACKF) {
        i2c3->ICR = I2C_ICR_NACKCF;
        return I2C_ERROR;
    }

    return I2C_OK;
}

I2C_Status I2C3_read(uint16_t device_address, uint8_t register_address, uint8_t *data, uint16_t size){
	
    I2C_Status ret;
    if (size == 0){
        return I2C_OK;
    }
    if (size > 255){
        return I2C_ERROR;
    }
    ret = I2C3_WaitFlagClear(&I2C3->ISR, I2C_ISR_BUSY);
    if (ret != I2C_OK) return ret;

    // write first
    i2c3->CR2 = 0;
    i2c3->CR2 |= ((uint32_t)(device_address & 0x7FU) << 1);
    i2c3->CR2 |= (1U << I2C_CR2_NBYTES_Pos);
    i2c3->CR2 &= ~I2C_CR2_RD_WRN;
    i2c3->CR2 &= ~I2C_CR2_AUTOEND;
    i2c3->CR2 |= I2C_CR2_START;
	
    ret = i2c3_WaitFlagSet(&i2c3->ISR, I2C_ISR_TXIS);
    if (ret != I2C_OK) return ret;
    i2c3->TXDR = register_address;

    ret = i2c3_WaitFlagSet(&i2c3->ISR, I2C_ISR_TC);
    if (ret != I2C_OK) return ret;

    // then we read
    i2c3->CR2 &= ~(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RD_WRN);
    i2c3->CR2 |= ((uint32_t)(device_address & 0x7FU) << 1);
    i2c3->CR2 |= ((uint32_t)size << I2C_CR2_NBYTES_Pos);
    i2c3->CR2 |= I2C_CR2_RD_WRN;
    i2c3->CR2 |= I2C_CR2_AUTOEND;

    i2c3->CR2 |= I2C_CR2_START;

    for (uint16_t i = 0; i < size; i++){ // actual reading happens here
        ret = i2c3_WaitFlagSet(&i2c3->ISR, I2C_ISR_RXNE);
        if (ret != I2C_OK) return ret;
        data[i] = (uint8_t)i2c3->RXDR;
    }

    ret = i2c3_WaitFlagSet(&i2c3->ISR, I2C_ISR_STOPF);
    if (ret != I2C_OK) return ret;

    i2c3->ICR = I2C_ICR_STOPCF;
	
    if (i2c3->ISR & I2C_ISR_NACKF) {
        i2c3->ICR = I2C_ICR_NACKCF;
        return I2C_ERROR;
    }

    return I2C_OK;
}


