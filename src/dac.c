#include "dac.h"



void DAC_init(void){
/*
PA4 - SPI_1_NSS low
PA5 - SPI_1_SCK clk
PA6 - SPI_1_MISO dont care
PA7 - SPI_1_MOSI master out slave in
*/

    // initialize SPI1 pins

	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL4 | GPIO_AFRL_AFSEL5 | GPIO_AFRL_AFSEL6 | GPIO_AFRL_AFSEL7); // mask af section
	GPIOA->AFR[0] |= ((5 << GPIO_AFRL_AFSEL4_Pos) | (5 << GPIO_AFRL_AFSEL5_Pos) | (5 << GPIO_AFRL_AFSEL6_Pos) | (5 << GPIO_AFRL_AFSEL7_Pos)); // select spi_1 (AF5)
    GPIOA->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE6 | GPIO_MODER_MODE7); //mask
    GPIOA->MODER |= (GPIO_MODER_MODE4_1 | GPIO_MODER_MODE5_1 | GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1); // enable alternate functions

    // configure spi
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // enable spi1 clock
    SPI1->CR1 = (SPI_CR1_MSTR);
    SPI1->CR2 = (SPI_CR2_SSOE | SPI_CR2_NSSP | (0xF << SPI_CR2_DS_Pos)); // enable nss output
    SPI1->CR1 |= (SPI_CR1_SPE);
}

uint16_t DAC_volt_conv(int mv){ // convert a voltage value into a 12-bit value to control the DAC
    if (mv>3300)mv=3300;
    if(mv<0) mv = 0;
    return (uint16_t)((uint32_t)4095*mv/3300);
}

void DAC_write(uint16_t data){
//    uint16_t control = 0x3000; // control bits for DAC
    uint16_t out = ((3<<12) | (data & 0x0FFF)); // expects data
    SPI1->DR = out;
}




