#include <stdio.h>
#include "imit_fifo.h"

static struct {
	int wraddr, rdaddr, count;
	motor_t buf[IMIT_BUF_SIZE];
} fifo;

BOOL imit_fifo_full() { return fifo.count == IMIT_BUF_SIZE; }
BOOL imit_fifo_empty() { return fifo.count == 0; }

int imit_fifo_count() { return fifo.count; }
int imit_fifo_free() { return IMIT_BUF_SIZE - fifo.count; }

int imit_fifo_add(const motor_t* const data) {
	if (fifo.count < IMIT_BUF_SIZE) {
		fifo.buf[fifo.wraddr] = *data;

		if (fifo.wraddr < IMIT_BUF_SIZE - 1)
			fifo.wraddr++;
		else
			fifo.wraddr = 0;

		fifo.count++;

		return 0;
	}
	else
		return -1;
}

motor_t* imit_fifo_q() {
	return &fifo.buf[fifo.rdaddr];
}

int imit_fifo_rdack() {
	if (fifo.count != 0) {
		if (fifo.rdaddr < IMIT_BUF_SIZE - 1)
			fifo.rdaddr++;
		else
			fifo.rdaddr = 0;

		fifo.count--;

		return 0;
	}

	return -1;
}

extern volatile BOOL imit_ena;

void imit_fifo_clear() {
	imit_ena = FALSE;
	memset(&fifo, 0, sizeof(fifo));
}

void motor_print(const motor_t* m) {
	static int x, y;

	switch (m->mode) {
	case 0:
		if (m->axis == AXIS_X) {
			x += m->N;
			printf("\t%d X:%d T:%d [%d, %d]\n", m->num, (int)m->N, (int)m->T, x, y);
		}
		else if (m->axis == AXIS_Y) {
			y += m->N;
			printf("\t%d Y:%d T:%d [%d, %d]\n", m->num, (int)m->N, (int)m->T, x, y);
		}
		else if (!m->last)
			printf("\tAxis ERR %d\n", m->axis);

		if (m->last)
			printf("\tEnd\n");

		break;
	default:
		printf("\tMode ERR\n");
		break;
	}
}
