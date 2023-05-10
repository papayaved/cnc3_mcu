#ifndef INC_KEY_TASK_H_
#define INC_KEY_TASK_H_

#include "my_types.h"

#define KEY_PERIOD (10) // 10 msec - 100 Hz. Keys' polling frequency
#define KEY_CNT_MAX (1) // response 20 ms
#define KEY_VEL_SPEED (49) // (KEY_VEL_SPEED + 1) * KEY_PERIOD - auto drum velocity INC/DEC period

#define KEY_NUM (5)
#define KEY_MASK (0x1F)

//#define KEY_PRINT

//typedef union {
//	uint8_t data;
//	struct {
//		uint8_t inc:1;
//		uint8_t dec:1;
//		uint8_t drum:1;
//		uint8_t pump:1;
//		uint8_t wire:1;
//	} key;
//} keys_t;

typedef union {
	uint8_t data;
	struct {
		uint8_t inc:1;
		uint8_t dec:1;
		uint8_t pump:1;
		uint8_t drum:1;
		uint8_t wire:1;
	} key;
} keys_t;

typedef struct {
	uint8_t cnt;
	BOOL state;
} key_counter_t;

void key_task();
void key_task_reset();

#endif /* INC_KEY_TASK_H_ */
