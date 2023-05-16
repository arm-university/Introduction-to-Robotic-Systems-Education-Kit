#include <platform.h>
#include <timer.h>

static void (*timer_callback)(void) = 0;

//Timer 2
void timer_init(uint32_t period) {
	// Initialise the timer with the specified period in clock
	// cycles. Should not start running the timer yet, however.
	
	// The timer should be able to tick anywhere from every
	// microsecond to every second. In the unlikely event the
	// the hardware is unable to divide down to a second, a
	// software divider should be implemented.
	

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //timer2 freq is 16 MHz, 1x prescale on AHB, APB1
  TIM2->ARR = period;
	TIM2->PSC = 0;
	
//	uint32_t tick_us = (SystemCoreClock)/1e6;
	//	timer_period = period; 
	//	tick_us = tick_us*period;	
}

void timer_enable(void) {
	// Enable the timer.
	//SysTick_Config(timer_period);
	TIM2->CR1 |= TIM_CR1_CEN;
}

void timer_disable(void) {
	// Disable the timer.
	TIM2->CR1 &= ~TIM_CR1_CEN;
	//SysTick->CTRL  &= ~SysTick_CTRL_ENABLE_Msk; 
}

void timer_set_callback(void (*callback)(void)) {
	// Set up and enable the interrupt.
	
	// The callback function should be stored in an internal
	// static function pointer.
	
	// The callback function should be executed periodically,
	// according to the period specified by the previous call
	// of timer_init.
	TIM2->CR1 |= TIM_CR1_URS;
	
	timer_callback = callback;
	TIM2->DIER |= TIM_DIER_UIE;
	NVIC_SetPriority(TIM2_IRQn,3);
  NVIC_ClearPendingIRQ(TIM2_IRQn);
	NVIC_EnableIRQ(TIM2_IRQn);
	
}

void PWM_init(uint32_t PWM_period)
{
	//Disable channel 1. 
	TIM2->CCER &= (uint16_t)~TIM_CCER_CC3E;
	//Reset the Output Compare Mode Bits
	TIM2->CCMR2&= (uint16_t)~TIM_CCMR2_OC3M;
	TIM2->CCMR2&= (uint16_t)~TIM_CCMR2_CC3S;
	//Reset the Output Polarity level
  TIM2->CCER &= (uint16_t)~TIM_CCER_CC3P;
		
	//Capture/Compare Output enable
	TIM2->CCER|= TIM_CCER_CC3E_Msk;
	
	//PWM mode 1
	TIM2->CCMR2|=0x6<<TIM_CCMR2_OC3M_Pos; 

	//set polarity to HIGH
	TIM2->CCER&=~TIM_CCER_CC3P_Msk;
	TIM2->CCR3= PWM_period;
	
	//reset the preload register
	TIM2->CCMR2&=(uint16_t)(~TIM_CCMR2_OC3PE);
	
	//enable the preload register
	TIM2->CCMR2|=(uint16_t)(TIM_CCMR2_OC3PE);
	TIM2->CCMR2|=TIM_CCMR2_OC3PE_Msk;

	TIM2->EGR |= ((uint16_t)0x0001);
	TIM2->CR1 |= TIM_CR1_ARPE;
}

void systick_init (uint32_t period_ms)
{
		uint32_t tick_ms;
		tick_ms = SystemCoreClock*(period_ms/1000.0);
		SysTick_Config(tick_ms);
		NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
}

void systick_set_callback(void (*callback)(void)) {
	// Set up and enable the interrupt.

	// The callback function should be stored in an internal
	// static function pointer.

	// The callback function should be executed periodically,
	// according to the period specified by the previous call
	// of timer_init.
	timer_callback = callback;

}

void SysTick_Handler(void)
{
	timer_callback();
}



void TIM2_IRQHandler(void)
{
	if((TIM2->SR)&1){
	  timer_callback();
		TIM2->SR &= ~0x1;		
		NVIC_ClearPendingIRQ(TIM2_IRQn);	
	}
}
// *******************************ARM University Program Copyright © ARM Ltd 2014*************************************   
