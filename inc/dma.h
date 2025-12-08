#ifndef DMA_H
#define DMA_H

#include "stm32l4xx_hal.h" // so we can use HAL functions
#include <stdint.h>
#include "stm32l4xx.h" 

#define N_SAMPLES  64

volatile uint16_t adc_buf[N_SAMPLES];
volatile uint8_t  i2c_buf[N_SAMPLES * 2];
uint8_t usart_tx_buf[N_SAMPLES * 4];
volatile uint8_t adc_ready = 0;
volatile uint8_t i2c_ready = 0;
volatile uint8_t tx_busy   = 0;

void adc_dma_init(void);
void i2c3_dma_init(void);
void usart2_dma_init(void);
void pack_samples(void);
void usart2_dma_send(int len);
void DMA1_Channel7_IRQHandler(void);


#endif