#include <platform.h>
#include <timer.h>

void TM_PWM_Init(void) {
    TIM_OCInitTypeDef TIM_OCStruct;
    
    /* Common settings */
    
    /* PWM mode 2 = Clear on compare match */
    /* PWM mode 1 = Set on compare match */
    TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
    
/*
    To get proper duty cycle, you have simple equation
    
    pulse_length = ((TIM_Period + 1) * DutyCycle) / 100 - 1
    
    where DutyCycle is in percent, between 0 and 100%
    
    25% duty cycle:     pulse_length = ((8399 + 1) * 25) / 100 - 1 = 2099
    50% duty cycle:     pulse_length = ((8399 + 1) * 50) / 100 - 1 = 4199
    75% duty cycle:     pulse_length = ((8399 + 1) * 75) / 100 - 1 = 6299
    100% duty cycle:    pulse_length = ((8399 + 1) * 100) / 100 - 1 = 8399
    
    Remember: if pulse_length is larger than TIM_Period, you will have output HIGH all the time
*/
    TIM_OCStruct.TIM_Pulse = 2099; /* 25% duty cycle */
    TIM_OC1Init(TIM4, &TIM_OCStruct);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    
    TIM_OCStruct.TIM_Pulse = 4199; /* 50% duty cycle */
    TIM_OC2Init(TIM4, &TIM_OCStruct);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
    
    TIM_OCStruct.TIM_Pulse = 6299; /* 75% duty cycle */
    TIM_OC3Init(TIM4, &TIM_OCStruct);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
    
    TIM_OCStruct.TIM_Pulse = 8399; /* 100% duty cycle */
    TIM_OC4Init(TIM4, &TIM_OCStruct);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
}