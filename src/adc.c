#include "adc.h"

ADC_HandleTypeDef hadc1;

void ADC_init(void){
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);
	GPIOA->MODER &= ~(3 << 0); // clear bits for PA0 channel 5
	GPIOA->MODER |=  (3 << 0); // analog mode
	GPIOA->PUPDR &= ~(3 << 0); // no pull
	
	RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN; // adc clock
	
	
    ADC1->CR &= ~ADC_CR_DEEPPWD; // need to exit deep power mode first to start ADC operations
    ADC1->CR |=  ADC_CR_ADVREGEN;
    HAL_Delay(25);
    
    ADC1->DIFSEL &= ~(1U << 5); // for single input adc channel 5
    ADC1->CR &= ~ADC_CR_ADCALDIF; // for single ended coversation thats what we want
    ADC1->CR |=  ADC_CR_ADCAL; // start calibration bit
    while (ADC1->CR & ADC_CR_ADCAL){} // while calibrating hold here
    
    ADC1->SMPR1 &= ~(7 << 15); // clear smpr
    ADC1->SMPR1 |= (4 << 15);// for like 47.5 cc adc sampling 
    
    ADC1->SQR1 = (ADC1->SQR1 & ~(ADC_SQR1_L_Msk | ADC_SQR1_SQ1_Msk))| (5 << ADC_SQR1_SQ1_Pos); // 5 bits to determine channel reading

    ADC1->ISR = ADC_ISR_ADRDY; // clear isr
    ADC1->CR  |= ADC_CR_ADEN; // enable adc
    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0){} // wait for init

    ADC1->ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR; // clear
}

uint16_t ADC1_read(void){ // what were going to use in irq

    ADC1->ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR; // clear
    ADC1->CR  |= ADC_CR_ADSTART; // start conversion
    while ((ADC1->ISR & ADC_ISR_EOC) == 0){} // wait for conversion
    return (uint16_t)ADC1->DR; // returns 12 bit value
}

int adc_to_mV(uint16_t data){
	int value = 3300*(data/4095);
	return value;
}
