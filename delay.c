#include "delay.h"
#include "stm32f10x.h"

volatile unsigned int TimingDelay; // __IO -- volatile

void delay_10ms(unsigned int delay)
{
     TIM2->PSC = (SystemCoreClock / 10000) - 1; /* 7200 */
     TIM2->ARR = delay;
     TIM2->EGR |= TIM_EGR_UG;
     TIM2->CR1 |= TIM_CR1_CEN | TIM_CR1_OPM;
     while ((TIM2->CR1 & TIM_CR1_CEN) != 0);
}

void delay_us(unsigned int delay)
{
     TIM2->PSC = (SystemCoreClock / 1000000)  - 1; /* 72 */
     TIM2->ARR = delay;
     TIM2->EGR |= TIM_EGR_UG;
     TIM2->CR1 |= TIM_CR1_CEN | TIM_CR1_OPM;
     while ((TIM2->CR1 & TIM_CR1_CEN) != 0);
}

// SysTick interrupt handler
void SysTick_Handler() {
	if (TimingDelay != 0) { TimingDelay--; }
}

// Do delay for mSecs milliseconds
void Delay_ms(uint16_t mSecs) {
	SysTick_Config(SystemCoreClock / DELAY_TICK_FREQUENCY_MS);
	TimingDelay = mSecs+1;
	while (TimingDelay != 0);
}

// Do delay for nSecs microseconds
void Delay_us(uint16_t uSecs) {
	SysTick_Config(SystemCoreClock / DELAY_TICK_FREQUENCY_US);
	TimingDelay = uSecs+1;
	while (TimingDelay != 0);
}
	
