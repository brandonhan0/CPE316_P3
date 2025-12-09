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
//	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);
//	GPIOA->MODER &= ~(3 << 0); // clear bits for PA0 channel 5
//	GPIOA->MODER |=  (3 << 0); // analog mode
//	GPIOA->PUPDR &= ~(3 << 0); // no pull
//
//	RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN; // adc clock
//
//    ADC123_COMMON->CCR &= ~ADC_CCR_CKMODE;
//    ADC123_COMMON->CCR |=  ADC_CCR_CKMODE_0;
//
//    ADC1->CR &= ~ADC_CR_ADEN;
//    ADC1->CR &= ~ADC_CR_DEEPPWD; // need to exit deep power mode first to start ADC operations
//    if(ADC123_COMMON) ADC1->CR |=  ADC_CR_ADVREGEN;
//    else ADC1->CR |= ADC_CR_ADVREGEN;
//    HAL_Delay(5);
//
//    ADC1->CR &= ~ADC_CR_ADCALDIF; // for single ended coversation thats what we want
//    ADC1->CR |=  ADC_CR_ADCAL; // start calibration bit
//    while (ADC1->CR & ADC_CR_ADCAL){} // while calibrating hold here
//
//    ADC1->DIFSEL &= ~(ADC_DIFSEL_DIFSEL_5);
//
//    ADC1->SMPR1 &= ~(7 << 15); // clear smpr
//    ADC1->SMPR1 |= (4 << 15);// for like 47.5 cc adc sampling
//
//    ADC1->SQR1 &= ~(ADC_SQR1_L_Msk | ADC_SQR1_SQ1_Msk);  // 1 conversion, clear SQ1
//    ADC1->SQR1 |=  (5u << ADC_SQR1_SQ1_Pos);
//
//    //ADC1->CFGR |= ADC_CFGR_CONT; // for dma
//    ADC1->ISR |= ADC_ISR_ADRDY;
//    ADC1->CR  |= ADC_CR_ADEN; // enable adc
//
//
//
//// disable en in dma setup
//
//
//
//    uint32_t timeout = 1000000;
//    while ((!(ADC1->ISR & ADC_ISR_ADRDY)) && --timeout);
//    if (timeout == 0) {
//        return; // error
//    }
//    //ADC1->CR |= ADC_CR_ADSTART; // for dma
//    ADC1->ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR;


    // --- GPIO: PA0 = ADC1_IN5 ---
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    GPIOA->MODER &= ~(3u << 0);   // clear PA0 mode
    GPIOA->MODER |=  (3u << 0);   // analog mode
    GPIOA->PUPDR &= ~(3u << 0);   // no pull

    // --- ADC clock ---
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;

    ADC123_COMMON->CCR &= ~ADC_CCR_CKMODE;
    ADC123_COMMON->CCR |=  ADC_CCR_CKMODE_0; // HCLK/1

    // --- ADC power-up + calibration ---
    ADC1->CR &= ~ADC_CR_ADEN;          // make sure disabled
    ADC1->CR &= ~ADC_CR_DEEPPWD;       // leave deep power-down
    ADC1->CR |=  ADC_CR_ADVREGEN;      // enable regulator
    HAL_Delay(5);

    // single-ended calibration
    ADC1->CR &= ~ADC_CR_ADCALDIF;
    ADC1->CR |=  ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL) { }

    // channel 5 single-ended
    ADC1->DIFSEL &= ~ADC_DIFSEL_DIFSEL_5;

    // sampling time for ch5
    ADC1->SMPR1 &= ~(7u << 15);
    ADC1->SMPR1 |=  (4u << 15);  // e.g. 47.5 cycles

    // regular sequence: 1 conversion, SQ1 = ch5
    ADC1->SQR1 &= ~(ADC_SQR1_L_Msk | ADC_SQR1_SQ1_Msk);
    ADC1->SQR1 |=  (5u << ADC_SQR1_SQ1_Pos);

    // single conversion mode, DMA enabled
    ADC1->CFGR &= ~ADC_CFGR_CONT;   // NO continuous
    ADC1->CFGR |=  ADC_CFGR_DMAEN;  // DMA enabled

    // enable ADC
    ADC1->ISR |= ADC_ISR_ADRDY;
    ADC1->CR  |= ADC_CR_ADEN;

    // *** DO NOT start conversion here ***
    // ADC1->CR |= ADC_CR_ADSTART;
}


//uint16_t ADC1_read(void){ // what were going to use in irq
//    ADC1->ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR; a// clear flags
//    ADC1->CR  |= ADC_CR_ADSTART; // start conversion
//    uint32_t timeout = 1;
//    while ((ADC1->ISR & ADC_ISR_EOC) == 0) {
//        if (--timeout == 0) {
//            return 0xFFF0; // error value, or handle somehow
//        }
//    }
//    ADC1->DR;
//    return (uint16_t)ADC1->DR;
//}

int adc_to_mV(uint16_t data){
	return (3300 * (int)data + 2047) / 4095;
}
