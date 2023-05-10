#include "debug_fifo.h"
#include "usart.h"

#define DEBUG_BUF_SIZE (1024 * 16)

static struct {
	int wraddr, rdaddr, length;
	uint8_t buf[DEBUG_BUF_SIZE];
} debug_fifo;

static const uint8_t full_str[] = "\r\nFULL\r\n";

void debug_task() {
#ifdef DEBUG_FIFO
//	if (!debug_fifo_empty())
//		ITM_SendChar( debug_fifo_get() );

	static BOOL tx_req;
	static uint8_t byte;

	HAL_StatusTypeDef status;

	if (debug_tx_rdy) {
		if (!tx_req && !debug_fifo_empty()) {
			byte = debug_fifo_get();
			tx_req = TRUE;
		}

		if (tx_req) {
			HAL_UART_StateTypeDef state = HAL_UART_GetState(&huart1);

			if (state == HAL_UART_STATE_READY) {
				debug_tx_rdy = FALSE;
				status = HAL_UART_Transmit_IT(&huart1, &byte, 1);

				if (status == HAL_OK)
					tx_req = FALSE;
				else
					debug_tx_rdy = TRUE;
			}
		}
	}
#endif
}

// FIFO
int debug_fifo_size() { return DEBUG_BUF_SIZE; }
BOOL debug_fifo_full() { return debug_fifo.length == DEBUG_BUF_SIZE; }
BOOL debug_fifo_empty() { return debug_fifo.length == 0; }

int debug_fifo_length() { return debug_fifo.length; }
int debug_fifo_free() { return DEBUG_BUF_SIZE - debug_fifo.length; }

int debug_fifo_add(uint8_t data) {
	if (debug_fifo_free() > (int)sizeof(full_str) - 1) {
		debug_fifo.buf[debug_fifo.wraddr++] = data;
		debug_fifo.length++;

		if (debug_fifo.wraddr == DEBUG_BUF_SIZE)
			debug_fifo.wraddr = 0;

		return 1;
	}
	else if (debug_fifo_free() == sizeof(full_str) - 1) {
		for (int i = 0; i < (int)sizeof(full_str) - 1; i++) {
			debug_fifo.buf[debug_fifo.wraddr++] = full_str[i];

			if (debug_fifo.wraddr == DEBUG_BUF_SIZE)
				debug_fifo.wraddr = 0;
		}

		debug_fifo.length += sizeof(full_str) - 1;

		return 0;
	}
	else
		return 0;
}

uint8_t debug_fifo_get() {
	if (debug_fifo.length != 0) {
		debug_fifo.length--;

		uint8_t data = debug_fifo.buf[debug_fifo.rdaddr++];

		if (debug_fifo.rdaddr == DEBUG_BUF_SIZE)
			debug_fifo.rdaddr = 0;

		return data;
	}
	else
		return debug_fifo.buf[debug_fifo.rdaddr];
}

void debug_fifo_clear() {
	memset(&debug_fifo, 0, sizeof(debug_fifo));
}
