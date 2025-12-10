#include "pwm.h"


/*
 *
 *
 * THIS WORKS FINE DONT CHANGE
 *
 *
 *
 * */


void PWM_init(void){

    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;

    // PA6 -> TIM3_CH1 (AF2)
    GPIOA->MODER &= ~(3U << 12);
    GPIOA->MODER |=  (2U << 12);
    GPIOA->PUPDR &= ~(3U << 12);

    GPIOA->AFR[0] &= ~(0xFU << 24);
    GPIOA->AFR[0] |=  (2U  << 24); // AF2 = TIM3

    TIM3->PSC = 159; // 1khz
    TIM3->ARR = 99;
    TIM3->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM3->CCMR1 |=  (6U << TIM_CCMR1_OC1M_Pos); // PWM mode 1
    TIM3->CCMR1 |=  TIM_CCMR1_OC1PE;

    TIM3->CCER  |=  TIM_CCER_CC1E;

    TIM3->CCR1 = 0;

    TIM3->CR1 |= TIM_CR1_ARPE;
    TIM3->EGR |= TIM_EGR_UG;
    TIM3->CR1 |= TIM_CR1_CEN;
}

void PWM_set_duty(float duty){
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    uint32_t arr = TIM3->ARR;
    uint32_t ccr = (uint32_t)(duty * (float)(arr + 1));
    if (ccr > arr) ccr = arr;

    TIM3->CCR1 = ccr;
}
