// # includes
// function prototypes

static volatile uint32_t count;
 
void SysTick_Handler(void) {
  count++;
}
 
// return the system clock as milliseconds
 
uint32_t millis(void) {
  return count;
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

// main...