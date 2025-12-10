#include "adc.h"

void ADC_init(void){
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    GPIOA->MODER &= ~(3u << 0);   // clear PA0 mode
    GPIOA->MODER |=  (3u << 0);   // analog mode
    GPIOA->PUPDR &= ~(3u << 0);   // no pull
    GPIOA->ASCR |= (1u << 0);
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;

    ADC123_COMMON->CCR &= ~ADC_CCR_CKMODE;
    ADC123_COMMON->CCR |=  ADC_CCR_CKMODE_0; // HCLK/1

    ADC1->CR &= ~ADC_CR_ADEN;
    ADC1->CR &= ~ADC_CR_DEEPPWD;
    ADC1->CR |=  ADC_CR_ADVREGEN; // enable regulator
    HAL_Delay(5);

    // single-ended calibration
    ADC1->CR &= ~ADC_CR_ADCALDIF;
    ADC1->CR |=  ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL) { }

    // channel 5 single-ended
    ADC1->DIFSEL &= ~ADC_DIFSEL_DIFSEL_5;

    // sampling time for ch5
    ADC1->SMPR1 &= ~(7u << 15);
    ADC1->SMPR1 |=  (7u << 15); // long sample time
    ADC1->SQR1 &= ~(ADC_SQR1_L_Msk | ADC_SQR1_SQ1_Msk);
    ADC1->SQR1 |=  (5u << ADC_SQR1_SQ1_Pos);
    ADC1->CFGR = 0;   // NO continuous
    ADC1->CFGR |= ADC_CFGR_DMAEN;  // DMA enabled new

    ADC1->ISR |= ADC_ISR_ADRDY;
    ADC1->CR  |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY)) {} // new
}

int adc_to_mV(uint16_t data){
	return (3300 * (int)data + 2047) / 4095;
}
