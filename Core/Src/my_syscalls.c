#include "stm32h7xx.h"
#include "defines.h"
#include "debug_fifo.h"

int _write(int file, char *ptr, int len) {
	/* Implement your write code here, this is used by puts and printf for example */
#ifndef RESEASE
#ifndef DEBUG_FIFO
	for(int i = 0; i < len; i++)
		ITM_SendChar(*ptr++);
#else
	for(int i = 0; i < len; i++)
		if (*ptr == '\n') {
			debug_fifo_add('\r');
			debug_fifo_add('\n');
			ptr++;
		}
		else
			debug_fifo_add(*ptr++);
#endif
#endif

	return len;
}
