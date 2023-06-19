/************************************************************/
/*                       Clk_Config.c                       */
/************************************************************/


#ifndef __MAIN_H
#define __MAIN_H
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_system.h" // FLASH related decarations
#endif /* __MAIN_H */

//void Speaker_Config(void) { //NB PB10 connected to speaker
//	LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_10, LL_GPIO_MODE_OUTPUT); 					//set PB10 as output
//	LL_GPIO_SetPinOutputType (GPIOB, LL_GPIO_PIN_10, LL_GPIO_OUTPUT_PUSHPULL); //output type is push-pull
//	LL_GPIO_SetPinPull (GPIOB, LL_GPIO_PIN_10, LL_GPIO_PULL_NO); 							//set PB10 as NO pull
//}
/**
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 84000000
  *            HCLK(Hz)                       = 84000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 4
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale2 mode
  *            Flash Latency(WS)              = 2
  */
void SystemClock_Config(void)
{
  /* Enable HSE oscillator */
  LL_RCC_HSE_EnableBypass();
  LL_RCC_HSE_Enable();
  while(LL_RCC_HSE_IsReady() != 1)
  {
  };

  /* Set FLASH latency */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 336, LL_RCC_PLLP_DIV_4);
  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };

  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  };

  /* Set APB1 & APB2 prescaler */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

//	 /* Set systick to 1 us */
//  SysTick_Config(84000000 / 1000000);
//	
////		 /* Set systick to 1 ms */
////  SysTick_Config(84000000 / 1000);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  SystemCoreClock = 84000000;
}
