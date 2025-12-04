#include "dma.h"

#include "stm32l4xx.h" 

#define N_SAMPLES  64

// ADC buffer
volatile uint16_t adc_buf[N_SAMPLES];

// I2C3 RX buffer
volatile uint8_t  i2c_buf[N_SAMPLES * 2];

// USART TX: 4 bytes per combined sample (2 for ADC, 2 for I2C)
uint8_t usart_tx_buf[N_SAMPLES * 4];

volatile uint8_t adc_ready = 0;
volatile uint8_t i2c_ready = 0;
volatile uint8_t tx_busy   = 0;

void adc_dma_init(void)
{
    // Enable DMA1 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    // Disable channel during config
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;

    // Peripheral address: ADC1 data register
    DMA1_Channel1->CPAR  = (uint32_t)&ADC1->DR;

    // Memory address: our ADC buffer
    DMA1_Channel1->CMAR  = (uint32_t)adc_buf;

    // Number of 16-bit samples
    DMA1_Channel1->CNDTR = N_SAMPLES;

    // Configure channel:
    DMA1_Channel1->CCR =
        DMA_CCR_MINC    |       // increment memory
        DMA_CCR_MSIZE_0 |       // memory size = 16-bit
        DMA_CCR_PSIZE_0 |       // peripheral size = 16-bit
        DMA_CCR_TCIE    |       // transfer-complete interrupt
        DMA_CCR_PL_1;           // high priority (optional)

    // Enable IRQ in NVIC (one-time)
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    // Enable channel
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    // Enable ADC DMA requests (ADC already configured)
    ADC1->CFGR |= ADC_CFGR_DMAEN;
    // Optionally: ADC1->CFGR |= ADC_CFGR_DMACFG; // circular vs normal depending on your mode
}

void i2c3_dma_init(void)
{
    // Enable DMA1 clock (if not already)
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    // Disable channel during config
    DMA1_Channel3->CCR &= ~DMA_CCR_EN;

    // Peripheral address: I2C3 RX data register
    DMA1_Channel3->CPAR  = (uint32_t)&I2C3->RXDR;

    // Memory address: our I2C buffer (bytes)
    DMA1_Channel3->CMAR  = (uint32_t)i2c_buf;

    // Number of BYTES to receive (2 per sample)
    DMA1_Channel3->CNDTR = N_SAMPLES * 2;

    // Configure channel (8-bit transfers by default: MSIZE=00, PSIZE=00)
    DMA1_Channel3->CCR =
        DMA_CCR_MINC    |       // increment memory for each received byte
        DMA_CCR_TCIE    |       // transfer-complete interrupt
        DMA_CCR_PL_0;           // priority (adjust as needed)

    // Enable IRQ in NVIC (one-time)
    NVIC_EnableIRQ(DMA1_Channel3_IRQn);

    // Enable channel
    DMA1_Channel3->CCR |= DMA_CCR_EN;

    // Enable I2C3 RX DMA in peripheral
    I2C3->CR1 |= I2C_CR1_RXDMAEN;

    // NOTE: you still need to start an I2C read transaction elsewhere SJLKFJSLKDFJLSKDJFLk
    // (set NBYTES, slave address, START, etc.) so that data actually arrives.
}

void usart2_dma_init(void)
{
    // Enable DMA1 clock (if not already)
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    // Base config for DMA1_Channel7 (USART2_TX)
    DMA1_Channel7->CCR = 0; // clear first
    DMA1_Channel7->CCR =
        DMA_CCR_MINC  |        // increment memory
        DMA_CCR_DIR   |        // memory -> peripheral
        DMA_CCR_TCIE;          // transfer-complete interrupt

    // Peripheral address: USART2 TX data register (constant)
    DMA1_Channel7->CPAR = (uint32_t)&USART2->TDR;

    // Enable IRQ in NVIC (one-time)
    NVIC_EnableIRQ(DMA1_Channel7_IRQn);

    // Enable TX DMA in USART2 (one-time)
    USART2->CR3 |= USART_CR3_DMAT;
}

void pack_samples(void)
{
    int idx = 0;

    for (int i = 0; i < N_SAMPLES; i++) {
        // ----- ADC (16-bit) -----
        uint16_t a = adc_buf[i];

        usart_tx_buf[idx++] = (uint8_t)(a & 0xFF);        // ADC low byte
        usart_tx_buf[idx++] = (uint8_t)((a >> 8) & 0xFF); // ADC high byte

        // ----- I2C3 (2 bytes -> 16-bit) -----
        uint8_t low  = i2c_buf[2 * i];
        uint8_t high = i2c_buf[2 * i + 1];
        // Optional reconstruct: uint16_t b = ((uint16_t)high << 8) | low;

        usart_tx_buf[idx++] = low;   // I2C low byte
        usart_tx_buf[idx++] = high;  // I2C high byte
    }
}

void usart2_dma_send(int len)
{
    // Don't start a new DMA send if one is already in progress
    if (tx_busy)
        return;

    tx_busy = 1;

    // Disable channel before reloading CNDTR/CMAR
    DMA1_Channel7->CCR &= ~DMA_CCR_EN;

    // Memory address: packed TX buffer
    DMA1_Channel7->CMAR  = (uint32_t)usart_tx_buf;

    // Number of bytes to send
    DMA1_Channel7->CNDTR = len;

    // Re-enable channel to start transfer
    DMA1_Channel7->CCR |= DMA_CCR_EN;
}

// USART2 TX DMA (DMA1_Channel7)
void DMA1_Channel7_IRQHandler(void)
{
    if (DMA1->ISR & DMA_ISR_TCIF7) {
        DMA1->IFCR = DMA_IFCR_CTCIF7;   // clear TC flag

        // Stop DMA channel
        DMA1_Channel7->CCR &= ~DMA_CCR_EN;

        tx_busy = 0;
    }
}