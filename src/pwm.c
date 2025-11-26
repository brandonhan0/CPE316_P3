#include "stm32f4xx.h"
#include "pwm.h"

void PWM_init(void){
  
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;


    GPIOA->MODER &= ~(3 << 12);
    GPIOA->MODER |=  (2 << 12);// alternate func
    GPIOA->PUPDR &= ~(3 << 12);
    GPIOA->AFR[0] &= ~(15 << 24);
    GPIOA->AFR[0] |=  (2 << 24); // AF2 = TIM3

    TIM3->PSC = 4; // 4MHz
    TIM3->ARR = 99; //1kHz
  
    TIM3->CCMR1 &= ~(TIM_CCMR1_OC1M);
    TIM3->CCMR1 |=  (6 << TIM_CCMR1_OC1M_Pos); // PWM mode 1
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE;

    TIM3->CCER |= TIM_CCER_CC1E;
    TIM3->CCR1 = 0; // start 0% duty do this ltr

    TIM3->CR1 |= TIM_CR1_ARPE;
    TIM3->EGR |= TIM_EGR_UG;
    TIM3->CR1 |= TIM_CR1_CEN;
}

 
void pwm_set_duty(float duty){ // set duty like 0.5f 
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;
    uint32_t arr = TIM3->ARR;
    uint32_t ccr = (uint32_t)(duty * (float)(arr + 1));
    if (ccr > arr) ccr = arr;
    TIM3->CCR1 = ccr;
}
