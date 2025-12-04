#include "uart.h"

void UART2_INIT(uint32_t pclk1_hz, uint32_t baud){
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

    RCC->CCIPR &= ~RCC_CCIPR_USART2SEL; // uart clock source


    GPIOA->MODER &= ~((3 << 4) | (3 << 6));
    GPIOA->MODER |=  ((2 << 4) | (2 << 6));
    GPIOA->AFR[0] &= ~((0xF << 8) | (0xF << 12));
    GPIOA->AFR[0] |=  ((7   << 8) | (7   << 12));

    GPIOA->PUPDR   &= ~((3 << 4) | (3 << 6));
    GPIOA->PUPDR   |=  ((1 << 4) | (1 << 6)); // pull-up

    USART2->CR1 &= ~USART_CR1_UE;

    // default
    USART2->CR1 = 0;
    USART2->CR2 = 0;
    USART2->CR3 = 0;

    uint32_t usartdiv = (pclk1_hz + (baud / 2)) / baud;
    USART2->BRR = usartdiv;
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;
    USART2->CR1 |= USART_CR1_UE;
}

void UART_write_char(char c){
	while ((USART2->ISR & USART_ISR_TXE) == 0){}
    USART2->TDR = (uint8_t)c;
}

void UART_write(const char *s){
    while (*s){
        UART_write_char(*s++);
    }
}
