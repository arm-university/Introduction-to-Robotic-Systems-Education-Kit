#include <platform.h>	
#include <timer.h>
#include <gpio.h>


static double PWM_frequency = 0.001;																				// Clock is in MHz. Variable to multiply clock to derive KHz
volatile double dutycycle = 0.0;																						// holds updated dutycycle
uint32_t timer_period;																											// holds computed time period
uint32_t trigger_point;																											// holds computed trigger point from updated dutycycle 
int dutycycle_change_direction = 0;																					// 0 for increasing pulse width ratio and 1 for decreasing it


void systick_callback_isr(void) {
	
	if (dutycycle < 1 && !dutycycle_change_direction)
	{
		dutycycle += 0.05;																											// increase by 5% duty cycle 
	}
	else if (dutycycle > 0 && dutycycle_change_direction)
	{
		dutycycle -= 0.05;																											// decrease by 5% duty cycle 
	}
	else
	{
		dutycycle_change_direction = !dutycycle_change_direction;								// alternate direction of dutycycle change
	}
	
	trigger_point = (uint32_t)(timer_period * dutycycle) - 1;									// computer new trigger point for the capture/compare register
	TIM2->CCR3 = trigger_point;																								// load the timer capture/compare register with new trigger point

}


int main(void) {
	
	timer_period = SystemCoreClock / (uint32_t)1e6 / (uint32_t)PWM_frequency - 1;			// compute PWM timer period in KHz from system clock
	
	gpio_set_mode(P_PWM, AF);																													// set gpio pin to alternative function
	
	gpio_AF_config(P_PWM, GPIO_AF_TIM2);																							// configure the alternative function to the timer
	
	timer_init(timer_period);																													// set the timer/PWM frequency
	trigger_point = (uint32_t)(timer_period * dutycycle) - 1;													// calculate the PWM period according to the duty cycle
	PWM_init(trigger_point);																													// initiate the PWM function
	
	systick_init (50);																																// set systick interrupt for every 50ms
	systick_set_callback(systick_callback_isr);
	
	timer_enable();																																		// enable timer
	
	while(1) 
	{
		__WFI();																																				// low power standby state, interrupt needed to exit this state
	}
}
