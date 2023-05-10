#ifndef USB_COM_H_
#define USB_COM_H_

#include <stdint.h>
#include "my_types.h"

#define DATA_SIZE (255U)
#define CMD_SIZE (4U + 1U + DATA_SIZE + 4U)

typedef enum {CMD_IDLE, CMD_READ, CMD_WRITE, CMD_READ_FIFO, CMD_BURST_READ, CMD_BURST_WRITE, CMD_ERROR = 0xF} COMMAND_T;

typedef struct {
	uint8_t buf[16 * CMD_SIZE];
//	uint8_t buf[8];
	size_t rdaddr, wraddr;
	BOOL overflow;
} rx_fifo_t;

typedef struct {
	uint8_t array[CMD_SIZE];
	size_t size;
} rx_buffer_t;

extern volatile rx_fifo_t rx_fifo;
extern volatile rx_buffer_t rx_buf;

size_t rx_fifo_add(const uint8_t* restrict const Buf, const size_t Len);
size_t rx_fifo_size();
BOOL rx_fifo_empty();
size_t rx_fifo_remain();
BOOL rx_fifo_full();
BOOL rx_fifo_enough(size_t len);
uint8_t rx_fifo_get(size_t i);
BOOL rx_fifo_overflow();
size_t rx_fifo_rdack(size_t len);
void rx_fifo_flush();

size_t rx_buf_move(size_t len);
size_t rx_buf_size();
BOOL rx_buf_empty();
uint8_t rx_buf_get(size_t i);
void rx_buf_rdack();

COMMAND_T rx_buf_cmd();
uint32_t rx_buf_addr();
uint8_t rx_buf_data_size();

void rx_fifo_tb();

#endif /* USB_COM_H_ */
