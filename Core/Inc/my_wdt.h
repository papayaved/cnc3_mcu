#ifndef INC_MY_WDT_H_
#define INC_MY_WDT_H_

#include "my_types.h"

#define PC_WDT_PERIOD (1000)

void soft_wdt_reset();
BOOL soft_wdt();
void soft_wdt_task();
void soft_wdt_clear();
void soft_wdt_enable(BOOL value);
BOOL soft_wdt_isEnabled();

BOOL hard_wdt();
void hard_wdt_check();

#endif /* INC_MY_WDT_H_ */
