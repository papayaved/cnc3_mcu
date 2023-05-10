#ifndef INC_DEBUG_FIFO_H_
#define INC_DEBUG_FIFO_H_

#include "my_types.h"

//#define DEBUG_FIFO

void debug_task();
int debug_fifo_add(uint8_t data);
BOOL debug_fifo_empty();
int debug_fifo_free();
uint8_t debug_fifo_get();
void debug_fifo_clear();

#endif /* INC_DEBUG_FIFO_H_ */
