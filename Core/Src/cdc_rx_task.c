#include <stdio.h>
#include "stm32h7xx_hal.h"

#include "my_types.h"
#include "rx_buf.h"

#define CDC_TIMEOUT (1000) // ms, timeout between USB packets that contain one communication packet

static COMMAND_T getCommand() { return (COMMAND_T)(rx_fifo_get(0)>>4); } // current command in FIFO
static uint8_t getLength() { return rx_fifo_get(4); } // packet size

/*
 * Old task that parsed Rx FIFO but it lost a burst write data
 */
void cdc_rx_task() {
	static uint32_t tic;
	static size_t size_reg;
	static BOOL timer_ena;
	BOOL timer_req = FALSE;

	if (rx_buf_empty()) {
		size_t size = rx_fifo_size();

		if (size) {
			timer_req = size == size_reg; // FIFO is not empty and is not changed

			if (size >= 9) {
				COMMAND_T cmd = getCommand();
				size_t len = getLength();

				switch (cmd) {
				case CMD_WRITE: case CMD_BURST_WRITE:
					len += 9;
					if (size >= len) {
						timer_req = FALSE;
						if (rx_buf_move(len) == 0)
							rx_fifo_flush(); // it's necessary?
					}
					break;

				case CMD_READ: case CMD_READ_FIFO:
					if (size >= 9) {
						timer_req = FALSE;
						if (rx_buf_move(9) == 0)
							rx_fifo_flush();
					}
					break;

				case CMD_BURST_READ:
					if (size >= 13) {
						timer_req = FALSE;
						if (rx_buf_move(13) == 0)
							rx_fifo_flush();
					}
					break;

				default:
					rx_fifo_flush();
					break;
				}
			}
		}

		size_reg = size;
	}

	if (timer_req) {
		if (!timer_ena) {
			timer_ena = TRUE;
			tic = HAL_GetTick();
		} else {
			uint32_t toc = HAL_GetTick();

			if (toc - tic >= CDC_TIMEOUT) {
//				for (int i = 0; i < size; i++) {
//					uint8_t c = rx_fifo_get(i);
//					printf("%02x(%c) ", c, c);
//				}
//
//				printf("\n");
				rx_fifo_flush();
			}
		}
	} else {
		timer_ena = FALSE;
	}
}

static BOOL timer_req = FALSE, timer_clr = FALSE; // enable request and clear timeout timer

/*
 * The function parses the Rx FIFO buffer and forms a data packet
 */
void cdc_rx_irq() {
	timer_req = FALSE;
	timer_clr = TRUE;

	size_t size = rx_fifo_size();
//	printf("F%d\n", size);

	if (rx_buf_empty() && size) {
		timer_req = TRUE; // it will be reseted when timer starts

		if (size >= 9) {
			COMMAND_T cmd = getCommand();
			size_t len = getLength();

			switch (cmd) {
			case CMD_WRITE: case CMD_BURST_WRITE:
				len += 9;
				if (size >= len) {
					if (rx_buf_move(len) == 0) {
						rx_fifo_flush(); // it's necessary?
						timer_req = FALSE;
					}
				}
				break;

			case CMD_READ: case CMD_READ_FIFO:
				if (size >= 9) {
					if (rx_buf_move(9) == 0) {
						rx_fifo_flush();
						timer_req = FALSE;
					}
				}
				break;

			case CMD_BURST_READ:
				if (size >= 13) {
					if (rx_buf_move(13) == 0) {
						rx_fifo_flush();
						timer_req = FALSE;
					}
				}
				break;

			default:
				rx_fifo_flush();
				timer_req = FALSE;
				break;
			}
		}
	}
}

/*
 * This task implements a timeout timer for receiving one data packet.
 * In case of timeout, the task clears the Rx FIFO buffer
 */
void cdc_rx_timer_task() {
	static uint32_t tic; // last tick
	static BOOL timer_ena;

	if (timer_clr) {
		timer_ena = FALSE;
		timer_clr = FALSE;
	}

	if (timer_req) {
		timer_ena = TRUE;
		tic = HAL_GetTick();
		timer_req = FALSE;
	}

	if (timer_ena) {
		uint32_t toc = HAL_GetTick();

		if (toc - tic >= CDC_TIMEOUT) {
//			size_t size = rx_fifo_size();
//			printf("TO %d\n", size);

//			for (int i = 0; i < size; i++) {
//				uint8_t c = rx_fifo_get(i);
//				printf("%02x(%c) ", c, c);
//			}
//			printf("\n");

			rx_fifo_flush(); // Rx error, clear old data
			timer_ena = FALSE;
		}
	}
}
