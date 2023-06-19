/************************************************************/
/*                      Time_Delays.c                       */
/************************************************************/

#ifndef __MAIN_H
#define __MAIN_H
#include "stm32f4xx_ll_rcc.h"
#endif /* __MAIN_H */
#include <stdint.h>
#include "Time_Delays.h"
static volatile uint32_t ticks;
 
void SysTick_Config_MCE2(Time_Units t_u) {
	switch (t_u) {
		case (us): SysTick_Config(84000000 / 1000000);
		break;
		case (ms): SysTick_Config(84000000 / 1000);
		break;
	}
}

void SysTick_Handler(void) {
  ticks++;
}

//*****************************************************************
// Time delays in ms
// To use et systick to 1 ms in SystemClock Configure
//  

uint32_t millis(void) {
  return ticks;
}

void delay_ms(uint32_t t) {
  uint32_t start, end;
  start = millis();
  end = start + t;
  if (start < end) { 
    while ((millis() >= start) && (millis() < end)) { 
      // do nothing 
    } 
  } else { 
    while ((millis() >= start) || (millis() < end)) {
      // do nothing
    };
  }
}
//*****************************************************************
// Time delays in us
uint32_t micros(void) {
  return ticks;
}
 
void delay_us(uint32_t t) {
  uint32_t start, end;
  start = micros();
  end = start + t;
  if (start < end) { 
    while ((micros() >= start) && (micros() < end)) { 
      // do nothing 
    } 
  } else { 
    while ((micros() >= start) || (micros() < end)) {
      // do nothing
    };
  }
}

