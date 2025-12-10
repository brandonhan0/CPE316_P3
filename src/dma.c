#include "dma.h"

#define N_SAMPLES  1

// ADC buffer
volatile uint16_t adc_buf[N_SAMPLES];

// I2C3 RX buffer
volatile uint8_t  i2c_buf[N_SAMPLES * 6];

// USART TX: 8 bytes per combined sample -> 2 for ADC, 6 for I2C
uint8_t usart_tx_buf[N_SAMPLES * 8];

volatile uint8_t tx_busy   = 0;

void adc_dma_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
		DMA1_CSELR->CSELR &=
			~((0xFu << 0)  |   // ch1
			  (0xFu << 8)  |   // ch3
			  (0xFu << 24));   // ch7
		DMA1_CSELR->CSELR |=
			(0u << 0)  |       // ch1 -> ADC1 (0000)
			(3u << 8)  |       // ch3 -> I2C3_RX (0011)
			(2u << 24);        // ch7 -> USART2_TX (0010)

    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    DMA1_Channel1->CPAR  = (uint32_t)&ADC1->DR;
    DMA1_Channel1->CMAR  = (uint32_t)adc_buf;
    DMA1_Channel1->CNDTR = N_SAMPLES;

    DMA1_Channel1->CCR  = 0;
    DMA1_Channel1->CCR |=
        DMA_CCR_MINC    |
		DMA_CCR_MSIZE_0 |
		DMA_CCR_PSIZE_0 |
		DMA_CCR_TCIE    |
        DMA_CCR_PL_1;
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

void i2c3_dma_init(void)
{
    // DMA1 clock already enabled in the previous init

    // Disable channel during config
    DMA1_Channel3->CCR &= ~DMA_CCR_EN;



    // Peripheral address: I2C3 RX data register
    DMA1_Channel3->CPAR  = (uint32_t)&I2C3->RXDR;

    // Memory address: our I2C buffer (bytes)
    DMA1_Channel3->CMAR  = (uint32_t)i2c_buf;

    // Number of BYTES to receive (2 per sample)
    DMA1_Channel3->CNDTR = N_SAMPLES * 6;

    // No msize or psize bc 8-bit transfers by default
    DMA1_Channel3->CCR = 0; // clear
    DMA1_Channel3->CCR =
        DMA_CCR_MINC    |       // increment memory for each received byte
        DMA_CCR_TCIE    |       // enable transfer-complete interrupt
        DMA_CCR_PL_0;           // priority

    // Enable DMA1 Channel 3 IRQ in NVIC
    NVIC_EnableIRQ(DMA1_Channel3_IRQn);

    // Enable channel
    DMA1_Channel3->CCR |= DMA_CCR_EN;

    // Enable I2C3 RX DMA in peripheral
    I2C3->CR1 |= I2C_CR1_RXDMAEN;
}

void usart2_dma_init(void)
{
    // DMA1 clock still already enabled

    // Disable channel during config
    DMA1_Channel7->CCR &= ~DMA_CCR_EN;

    // Base config for DMA1_Channel7 (USART2_TX)
    DMA1_Channel7->CCR = 0; // clear first
    DMA1_Channel7->CCR =
        DMA_CCR_MINC  |        // increment memory
        DMA_CCR_DIR   |        // memory -> peripheral
        DMA_CCR_TCIE;          // transfer-complete interrupt

    // Peripheral address set to USART2 TX data register
    DMA1_Channel7->CPAR = (uint32_t) &USART2->TDR;

    // Enable DMA1 Channel 7 IRQ in NVIC
    NVIC_EnableIRQ(DMA1_Channel7_IRQn);

    // Enable TX DMA in USART2
    USART2->CR3 |= USART_CR3_DMAT;
}

void pack_samples(void)
{
    int idx = 0;

    for (int i = 0; i < N_SAMPLES; i++) {
        // ADC 16-bit
        uint16_t a = adc_buf[i];
        usart_tx_buf[idx++] = (uint8_t)(a & 0xFF);         // ADC low
        usart_tx_buf[idx++] = (uint8_t)((a >> 8) & 0xFF);  // ADC high

        // I2C 6-byte sample
        for (int j = 0; j < 6; j++) {
            usart_tx_buf[idx++] = i2c_buf[6 * i + j];
        }
    }



    // Re-enable I2C data collection
    DMA1_Channel3->CNDTR = N_SAMPLES * 6;
    DMA1_Channel3->CCR  |= DMA_CCR_EN;

}

void usart2_dma_send(int len)
{
    // Don't start a new DMA send if one already in progress
    if (tx_busy) return;
    tx_busy = 1;

    // Disable channel before reloading CNDTR/CMAR
    DMA1_Channel7->CCR &= ~DMA_CCR_EN;

    // Memory address set to already packed USART buffer
    DMA1_Channel7->CMAR  = (uint32_t)usart_tx_buf;

    // Number of bytes to send
    DMA1_Channel7->CNDTR = len;

    // Re-enable channel to start transfer
    DMA1_Channel7->CCR |= DMA_CCR_EN;
}

uint16_t ADC1_get_latest_sample(void) {
    return adc_buf[0];   // or last index if you use N_SAMPLES > 1
}

// Interrupt for when DMA 1 Channel 7 finishes transfering
void DMA1_Channel7_IRQHandler(void)
{
    if (DMA1->ISR & DMA_ISR_TCIF7) {
        DMA1->IFCR = DMA_IFCR_CTCIF7;   // clear TC flag

        // Stop DMA channel
        DMA1_Channel7->CCR &= ~DMA_CCR_EN;

        tx_busy = 0; // new DMA send ready


    }
}
