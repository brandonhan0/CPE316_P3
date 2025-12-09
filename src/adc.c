#include "adc.h"

/*
 *
 *
 * THIS WORKS FINE DONT CHANGE
 *
 *
 *
 * */

void ADC_init(void){
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);
	GPIOA->MODER &= ~(3 << 0); // clear bits for PA0 channel 5
	GPIOA->MODER |=  (3 << 0); // analog mode
	GPIOA->PUPDR &= ~(3 << 0); // no pull

	RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN; // adc clock

    ADC123_COMMON->CCR &= ~ADC_CCR_CKMODE;
    ADC123_COMMON->CCR |=  ADC_CCR_CKMODE_0;

    ADC1->CR &= ~ADC_CR_ADEN;
    ADC1->CR &= ~ADC_CR_DEEPPWD; // need to exit deep power mode first to start ADC operations
    if(ADC123_COMMON) ADC1->CR |=  ADC_CR_ADVREGEN;
    else ADC1->CR |= ADC_CR_ADVREGEN;
    HAL_Delay(5);

    ADC1->CR &= ~ADC_CR_ADCALDIF; // for single ended coversation thats what we want
    ADC1->CR |=  ADC_CR_ADCAL; // start calibration bit
    while (ADC1->CR & ADC_CR_ADCAL){} // while calibrating hold here

    ADC1->DIFSEL &= ~(ADC_DIFSEL_DIFSEL_5);

    ADC1->SMPR1 &= ~(7 << 15); // clear smpr
    ADC1->SMPR1 |= (4 << 15);// for like 47.5 cc adc sampling

    ADC1->SQR1 &= ~(ADC_SQR1_L_Msk | ADC_SQR1_SQ1_Msk);  // 1 conversion, clear SQ1
    ADC1->SQR1 |=  (5u << ADC_SQR1_SQ1_Pos);

    ADC1->ISR |= ADC_ISR_ADRDY;
    ADC1->CFGR |= ADC_CFGR_DMAEN; // for dma
    ADC1->CR  |= ADC_CR_ADEN; // enable adc

    uint32_t timeout = 1000000;
    while ((!(ADC1->ISR & ADC_ISR_ADRDY)) && --timeout);
    if (timeout == 0) {
        return; // error
    }
    ADC1->CR |= ADC_CR_ADSTART; // for dma
    ADC1->ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR;
}


uint16_t ADC1_read(void){ // what were going to use in irq
    ADC1->ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR; // clear flags
    ADC1->CR  |= ADC_CR_ADSTART; // start conversion
    uint32_t timeout = 1;
    while ((ADC1->ISR & ADC_ISR_EOC) == 0) {
        if (--timeout == 0) {
            return 0xFFF0; // error value, or handle somehow
        }
    }
    ADC1->DR;
    return (uint16_t)ADC1->DR;
}

int adc_to_mV(uint16_t data){
	return (3300 * (int)data + 2047) / 4095;
}
