#ifndef INC_ALARM_SM_H_
#define INC_ALARM_SM_H_

#include "defines.h"
#include "my_types.h"

void alarm_reset();
void alarm_sm(uint16_t limsw_flags);

BOOL alarm_isAlarm();
BOOL alarm_isIdle();
BOOL alarm_drumPermit();

void alarm_onFailure();
extern uint8_t cnc_ena;

#endif /* INC_ALARM_SM_H_ */
