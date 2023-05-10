#ifndef INC_SEM_LED_H_
#define INC_SEM_LED_H_

#include "my_types.h"

//typedef union {
//	uint16_t data;
//	struct {
//		uint16_t value:3;
//		uint16_t :5;
//		uint16_t enable:1;
//		uint16_t :7;
//	} field;
//} semaphore_t;

typedef enum {SEM_OFF = 0, SEM_RED = 1, SEM_YELLOW = 2, SEM_GREEN = 4} SEM_COLOR_ENU;

void sem_reset();
void sem_enable(BOOL value);
BOOL sem_enabled();
void sem_set(SEM_COLOR_ENU value);
SEM_COLOR_ENU sem_get();
void sem_task();

void sem_next();

#endif /* INC_SEM_LED_H_ */
