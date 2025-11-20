#include "usart.h"


static inline uint32_t usart_brr(uint32_t pclk_hz, uint32_t baud){
    return (pclk_hz + (baud/2U)) / baud; // rounded
}

void USART_init(uint32_t pclk_hz, uint32_t baud){

	RCC->AHB2ENR  |= (RCC_AHB2ENR_GPIOAEN);
	RCC->APB1ENR1 |= (1 << 17);

	GPIOA->MODER &= ~((3 << 4) | (3 << 6)); // clear
	GPIOA->MODER |=  ((2 << 4) | (2 << 6)); // AF

	GPIOA->OTYPER &= ~((1 << 2) | (1 << 3));

	GPIOA->PUPDR &= ~((3 << 4) | (3 << 6));

	GPIOA->OSPEEDR &= ~((3 << 4) | (3 << 6));
	GPIOA->OSPEEDR |=  ((3 << 4) | (3 << 6));

	GPIOA->AFR[0] &= ~((0xF << 8) | (0xF << 12)); // clear
	GPIOA->AFR[0] |=  ((0x7 << 8) | (0x7 << 12)); // AF7
	
	USART2->BRR  = usart_brr(pclk_hz, baud);
	USART2->CR1 &= ~USART_CR1_UE; // cant have this on during config
	USART2->CR1 = 0; // default config
    USART2->CR2 = 0; // stop bit
    USART2->CR3 = 0; // no hw flow
    USART2->CR1 = (1 << 3) | (1 << 2); // should enable tx we dont need rx maybe for programming idk its on now though
    
    USART2->CR1 |= (1 << 0); // en b1
}

void USART_write(const char *s){
    const uint8_t *p = (const uint8_t *)s;
    while (*p){
        while ((USART2->ISR & USART_ISR_TXE) == 0){}
        USART2->TDR = *p++;
    }
    while ((USART2->ISR & USART_ISR_TC) == 0){}
}

