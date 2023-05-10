#include "rx_buf.h"

#include <string.h>
#include <stdio.h>

#include "aux_func.h"

volatile rx_fifo_t rx_fifo;
volatile rx_buffer_t rx_buf;

/*return
 *number of written data
 */
size_t rx_fifo_add(const uint8_t* restrict const Buf, const size_t Len) {
	if (rx_fifo.wraddr >= sizeof(rx_fifo.buf)) // error
		rx_fifo.wraddr = rx_fifo.rdaddr;

	if (Len <= rx_fifo_remain()) {
		if (rx_fifo.wraddr + Len < sizeof(rx_fifo.buf)) {
			memcpy((uint8_t*)&rx_fifo.buf[rx_fifo.wraddr], Buf, Len);
			rx_fifo.wraddr += Len;
		}
		else {
			size_t rem = sizeof(rx_fifo.buf) - rx_fifo.wraddr;
			memcpy((uint8_t*)&rx_fifo.buf[rx_fifo.wraddr], Buf, rem);

			rx_fifo.wraddr = Len - rem;
			if (rx_fifo.wraddr)
				memcpy((uint8_t*)rx_fifo.buf, &Buf[rem], rx_fifo.wraddr);
		}
		return Len;
	}
	else {
		rx_fifo.overflow = TRUE;
		return 0;
	}
}

void rx_fifo_flush() { rx_fifo.rdaddr = rx_fifo.wraddr; }

size_t rx_fifo_size() {
	size_t wraddr = rx_fifo.wraddr;

	if (wraddr >= sizeof(rx_fifo.buf)) // error
		return 0;
	else if (rx_fifo.rdaddr == wraddr)
		return 0;
	else if (wraddr > rx_fifo.rdaddr)
		return wraddr - rx_fifo.rdaddr;
	else
		return sizeof(rx_fifo.buf) - rx_fifo.rdaddr + wraddr;
}

BOOL rx_fifo_empty() { return rx_fifo_size() == 0; }
BOOL rx_fifo_full() { return rx_fifo_size() >= sizeof(rx_fifo.buf) - 1; }

size_t rx_fifo_remain() {
	size_t size = rx_fifo_size();
	return size < sizeof(rx_fifo.buf) - 1 ? sizeof(rx_fifo.buf) - 1 - size : 0;
}
BOOL rx_fifo_enough(size_t len) { return len <= rx_fifo_remain(); }

uint8_t rx_fifo_get(size_t i) {
	if (i < sizeof(rx_fifo.buf)) {
		i += rx_fifo.rdaddr;

		if (i >= sizeof(rx_fifo.buf))
			i -= sizeof(rx_fifo.buf);

		return rx_fifo.buf[i];
	}
	return 0;
}

BOOL rx_fifo_overflow() { return rx_fifo.overflow; }

size_t rx_fifo_rdack(size_t len) {
	if (len <= rx_fifo_size()) {
		size_t rdaddr = rx_fifo.rdaddr;

		rdaddr += len;

		if (rdaddr >= sizeof(rx_fifo.buf))
			rdaddr -= sizeof(rx_fifo.buf);

		rx_fifo.rdaddr = rdaddr;

		return len;
	}
	return 0;
}

//////////////////////////////////////////////////////////
size_t rx_buf_move(size_t len) {
	size_t fifo_size, len1, len2;
	size_t wraddr = rx_fifo.wraddr;

	if (rx_fifo.rdaddr != wraddr) {
		if (wraddr > rx_fifo.rdaddr)
			fifo_size = wraddr - rx_fifo.rdaddr;
		else
			fifo_size = sizeof(rx_fifo.buf) - rx_fifo.rdaddr + wraddr;
	}
	else
		fifo_size = 0;

	if (fifo_size && len && len <= fifo_size && len <= sizeof(rx_buf.array)) {
		if (wraddr > rx_fifo.rdaddr) {
			memcpy((uint8_t*)rx_buf.array, (uint8_t*)&rx_fifo.buf[rx_fifo.rdaddr], len);
			rx_fifo_rdack(len);
			rx_buf.size = len;
		}
		else {
			fifo_size = sizeof(rx_fifo.buf) - rx_fifo.rdaddr;
			len1 = len > fifo_size ? fifo_size : len;
			memcpy((uint8_t*)rx_buf.array, (uint8_t*)&rx_fifo.buf[rx_fifo.rdaddr], len1);
			len2 = len - len1;

			if (len2)
				memcpy((uint8_t*)&rx_buf.array[len1], (uint8_t*)rx_fifo.buf, len2);

			rx_fifo_rdack(len);
			rx_buf.size = len;
		}
	}
	else
		rx_buf.size = 0;

	return rx_buf.size;
}

size_t rx_buf_size() { return rx_buf.size; }
BOOL rx_buf_empty() { return !rx_buf.size; }
uint8_t rx_buf_get(size_t i) { return i < sizeof(rx_buf.array) ? rx_buf.array[i] : 0; }
void rx_buf_rdack() { rx_buf.size = 0; }

COMMAND_T rx_buf_cmd() { return (rx_buf.size) ? (COMMAND_T)(rx_buf.array[0]>>4) : CMD_IDLE; }
uint32_t rx_buf_addr() { return (rx_buf.size >= 4) ? read_u32_rev((uint8_t*)rx_buf.array, rx_buf.size, 0) & 0x0FFFFFFF : ~0U; }
uint8_t rx_buf_data_size() { return (rx_buf.size >= 5) ? rx_buf.array[4] : 0; }

void rx_fifo_tb() {
//	static uint8_t buf[64];
	static uint8_t buf[6];
	const int len = sizeof(buf);
	BOOL OK;

	printf("RX FIFO test\n");

	for (int i = 0; i < len; i++)
		buf[i] = i + 1;

	rx_fifo_add(buf, len);

	OK = rx_fifo_size() == len;
	if (!OK)
		printf("Error size: %d\n", rx_fifo_size());

	size_t size = rx_fifo_size();
	for (int i = 0; i < size; i++) {
		OK &= rx_fifo_get(i) == buf[i];
		if (!OK) {
			printf("Data error: %x %x\n", rx_fifo_get(i), buf[i]);
		}
	}

	rx_fifo_rdack(size);

	OK &= rx_fifo_empty();
	if (!OK)
		printf("Not empty 1\n");

	int i = 0;
	while (!rx_fifo_full()) {
		rx_fifo_add((uint8_t*)&i, 1);
		i++;
	}

	size = rx_fifo_size();
	OK &= size == sizeof(rx_fifo.buf) - 1;

	for (int i = 0; i < size; i++) {
		OK &= rx_fifo_get(i) == (uint8_t)i;
		if (!OK) {
			printf("Data error: %x %x\n", rx_fifo_get(i), i & 0xFF);
			break;
		}
	}

	rx_fifo_rdack(size);

	OK &= rx_fifo_empty();
	if (!OK)
		printf("Not empty 2\n");

	if (OK)
		printf("OK\n");


	printf("Test RX BUF\n");

	for (int i = 0; i < len; i++)
		buf[i] = i + 10;

	rx_fifo_add(buf, len);

	size = rx_fifo_size();
	if (rx_buf_empty() && size <= sizeof(rx_buf.array))
		rx_buf_move(size);

	size = rx_buf_size();
	for (int i = 0; i < size; i++) {
		OK &= rx_buf_get(i) == buf[i];
		if (!OK) {
			printf("Data error: %x %x\n", rx_buf_get(i), buf[i]);
		}
	}

	rx_buf_rdack();

	OK &= rx_buf_empty();
	if (!OK)
		printf("Not empty 3\n");

	if (OK)
		printf("OK\n");
}
