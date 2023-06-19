/************************************************************/
/*                      Time_Delays.h                       */
/************************************************************/

#include <stdint.h>

typedef enum {us, ms} Time_Units;

static volatile uint32_t ticks;

void SysTick_Config_MCE2(Time_Units t_u);

void SysTick_Handler(void);
	
uint32_t millis(void);
void delay_ms(uint32_t t);
	
uint32_t micros(void);
void delay_us(uint32_t t);

