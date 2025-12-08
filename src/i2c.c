#include "i2c.h"

static void I2C3_Init(void)
{
	//------------------- GPIO_INIT --------------------//

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN; 	// Enable GPIOC clock

    GPIOC->MODER &= ~(0x3 | (0x3 << 2));	// Set PC0 to SCL and PC1 to SDA
    GPIOC->MODER |=  (0x2 | (0x2 << 2));

    GPIOC->OTYPER |= (0x1 | (0x1 << 1));	// Open-drain

    GPIOC->OSPEEDR |= (0x3 | (0x3 << 2));	// High speed

    GPIOC->PUPDR &= ~(0x3 | (0x3 << 2));	// No internal pull-ups (temp sensor has internal pull-ups)

    GPIOC->AFR[0] &= ~(0xF | (0xF << 4));	// Select AF4 (I2C) for PC0, PC1 in AFRL
    GPIOC->AFR[0] |=  (0x4 | (0x4  << 4));

    //------------------- I2C3_INIT ----------------------//

    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C3EN;		// Enable I2C3 clock on APB1

    RCC->APB1RSTR1 |=  RCC_APB1RSTR1_I2C3RST;	// Reset I2C3, then release from reset
    RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_I2C3RST;

    I2C3->CR1 &= ~I2C_CR1_PE;					// Disable I2C3 before changing timing

    I2C3->TIMINGR = 0x00503d58u;    			//

    I2C3->CR1 |= I2C_CR1_PE;					// Enable I2C3 peripheral
}

static void I2C3_Write(uint8_t addr7, const uint8_t *data, uint8_t len)
{
    uint32_t tmp;

    // Wait until bus is free
    while (I2C3->ISR & I2C_ISR_BUSY) { }

    // Configure CR2: address, write, NBYTES, AUTOEND.
    tmp  = I2C3->CR2;
    tmp &= ~(I2C_CR2_SADD | I2C_CR2_RD_WRN | I2C_CR2_NBYTES | I2C_CR2_AUTOEND);
    tmp |= ((uint32_t)addr7 << 1);                  // 7-bit address in SADD[7:1]
    tmp |= ((uint32_t)len << I2C_CR2_NBYTES_Pos);   // number of bytes
    tmp |= I2C_CR2_AUTOEND;                         // automatic STOP after NBYTES
    // RD_WRN = 0 for write
    I2C3->CR2 = tmp;

    // Generate START
    I2C3->CR2 |= I2C_CR2_START;

    for (uint8_t i = 0; i < len; i++)
    {
        // Wait until TXIS (transmit interrupt status) set = TXDR empty.
        while (!(I2C3->ISR & I2C_ISR_TXIS)) { }
        I2C3->TXDR = data[i];
    }

    // Wait for STOP flag then clear it.
    while (!(I2C3->ISR & I2C_ISR_STOPF)) { }
    I2C3->ICR = I2C_ICR_STOPCF;
}

static void I2C3_Read(uint8_t addr7, uint8_t *data, uint8_t len)
{
    uint32_t tmp;

    // Wait until bus free.
    while (I2C3->ISR & I2C_ISR_BUSY) { }

    // Configure CR2: address, read, NBYTES, AUTOEND.
    tmp  = I2C3->CR2;
    tmp &= ~(I2C_CR2_SADD | I2C_CR2_RD_WRN | I2C_CR2_NBYTES | I2C_CR2_AUTOEND);
    tmp |= ((uint32_t)addr7 << 1);
    tmp |= ((uint32_t)len << I2C_CR2_NBYTES_Pos);
    tmp |= I2C_CR2_AUTOEND | I2C_CR2_RD_WRN;  // read transfer
    I2C3->CR2 = tmp;

    // Generate START
    I2C3->CR2 |= I2C_CR2_START;

    for (uint8_t i = 0; i < len; i++)
    {
        // Wait until RXNE (receive data register not empty).
        while (!(I2C3->ISR & I2C_ISR_RXNE)) { }
        data[i] = (uint8_t)I2C3->RXDR;
    }

    // Wait for STOP then clear flag
    while (!(I2C3->ISR & I2C_ISR_STOPF)) { }
    I2C3->ICR = I2C_ICR_STOPCF;
}

static void SHT3x_StartSingleShot(void)
{
    uint8_t cmd[2];

    cmd[0] = (uint8_t)(SHT3X_CMD_SINGLE_SHOT_HIGH_NO_STRETCH >> 8); // MSB
    cmd[1] = (uint8_t)(SHT3X_CMD_SINGLE_SHOT_HIGH_NO_STRETCH & 0xFF); // LSB

    I2C3_Write(SHT3X_I2C_ADDR, cmd, 2);

    // Datasheet: minimum 1 ms between commands, measurement time depends on repeatability.
    // For high repeatability, a conservative delay ~15 ms is typical.
    // TODO: implement a delay_ms() using SysTick or a timer.
    HAL_Delay(15);
}

static int SHT3x_ReadRaw(uint16_t *rawT, uint16_t *rawRH)
{
    uint8_t buf[6];

    // Read 6 bytes: T_MSB, T_LSB, CRC_T, RH_MSB, RH_LSB, CRC_RH
    I2C3_Read(SHT3X_I2C_ADDR, buf, 6);

    // TODO: verify CRC for temperature and humidity using datasheet CRC-8 (poly 0x31, init 0xFF).

    *rawT  = ((uint16_t)buf[0] << 8) | buf[1];
    *rawRH = ((uint16_t)buf[3] << 8) | buf[4];

    return 0; // return non-zero on CRC or I2C errors once you add checks
}

static float SHT3x_Convert_Temperature_F(uint16_t rawT)
{
    // T [°F] = -49 + 315 * ST / (2^16 - 1)
    return -49.0f + 315.0f * ((float)rawT / 65535.0f);
}

static float SHT3x_Convert_RH(uint16_t rawRH)
{
    // RH [%] = 100 * SRH / (2^16 - 1)
    return 100.0f * ((float)rawRH / 65535.0f);
}
