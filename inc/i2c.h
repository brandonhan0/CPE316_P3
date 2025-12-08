/*
 * i2c.c
 *
 *  Created on: Dec 8, 2025
 *      Author: Keega
 */

#ifndef INC_I2C_C_
#define INC_I2C_C_

#include "stm32l476xx.h"
#include "stdio.h"
#include <string.h>

#define SHT3X_I2C_ADDR      0x44u      // AD pin = GND
#define I2C3_SCL_PIN        0u         // PC0
#define I2C3_SDA_PIN        1u         // PC1
#define SHT3X_CMD_SINGLE_SHOT_HIGH_NO_STRETCH   0x2400u   // from datasheet Table 9

static void I2C3_Init(void);
static void I2C3_Write(uint8_t addr7, const uint8_t *data, uint8_t len);
static void I2C3_Read(uint8_t addr7, uint8_t *data, uint8_t len);
static void SHT3x_StartSingleShot(void);
static int SHT3x_ReadRaw(uint16_t *rawT, uint16_t *rawRH);
static float SHT3x_Convert_Temperature_F(uint16_t);
static float SHT3x_Convert_RH(uint16_t);

#endif /* INC_I2C_C_ */
