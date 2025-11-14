#include <assert.h>
#include <stm32g4xx_hal.h>
#include "pcanpro_timestamp.h"

void pcan_timestamp_init( void )
{
	// STM32G431的TIM2计数器为32位，能覆盖从微秒~毫秒级别的范围
	
    __HAL_RCC_TIM2_CLK_ENABLE();
    TIM2->CR1   = 0;
    TIM2->CR2   = 0;
    TIM2->SMCR  = 0;
    TIM2->DIER  = 0;
    TIM2->CCMR1 = 0;
    TIM2->CCMR2 = 0;
    TIM2->CCER  = 0;
    TIM2->PSC   = (SystemCoreClock / 1000000) - 1; // 1 MHz
    TIM2->ARR   = 0xFFFFFFFF;
    TIM2->CR1  |= TIM_CR1_CEN;
    TIM2->EGR   = TIM_EGR_UG;
}

uint32_t pcan_timestamp_millis( void )
{
  return  HAL_GetTick();
}

uint32_t pcan_timestamp_us( void )
{
  return TIM2->CNT;
}
