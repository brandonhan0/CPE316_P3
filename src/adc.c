#include "adc.h"

ADC_HandleTypeDef hadc1;

void ADC_PINS_init(void){
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);
	GPIOA->MODER &= ~(3 << 0); // clear bits for PA0 channel 5
	GPIOA->MODER |=  (3 << 0); // analog mode
	GPIOA->PUPDR &= ~(0b11 << (0 * 2)); // no pull
}

uint16_t ADC1_read(void){ // what were going to use in irq

    uint16_t value = 0;
    HAL_ADC_Start(&hadc1); // start conversion
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK){
        value = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1); // end conversion
    return value; // this is gonan be like the 12b dac value or wtv we convert to mV
}
