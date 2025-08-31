#include "main.h"

static inline void DWT_Delay_Init(void);
static inline void delay_us(uint32_t us);

void HC_SR04_Init(void)
{
	DWT_Delay_Init();
}

void HC_SR04_Trigger(void)
{
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
	  delay_us(10);
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);
}

uint16_t HC_SR04_Distance_Calculate(uint32_t pulse_us)
{
	return (pulse_us+1) * 340/20000U;
}

static inline void DWT_Delay_Init(void){
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline void delay_us(uint32_t us){
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock/1000000U);
    while ((DWT->CYCCNT - start) < ticks);
}
