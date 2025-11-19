#include "usart.h"

void USART_PINS_init(void){

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
	
}

void USART_write(char* data){ // i would rather use HAL here bruh aint nobody wanna do memory shit
    HAL_UART_Transmit(&huart2, (uint8_t*)data, strlen(data), HAL_MAX_DELAY);
}
