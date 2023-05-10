#ifndef INC_IMIT_FIFO_H_
#define INC_IMIT_FIFO_H_

#include "my_types.h"
#include "line.h"
#include "rx_buf.h"

#define IMIT_BUF_SIZE ((DATA_SIZE / 12) * 12 * 2) // double buffering
//#define IMIT_FIFO_ENABLE
//#define SUM_IMIT_STEPS

typedef struct {
	uint32_t num:24;
	uint32_t axis:3;
	uint32_t sync:1;
	uint32_t mode:2;
	uint32_t last:1;
	uint32_t valid:1;
	int32_t N:32;
	uint32_t T:32;
} motor_t;

typedef motor_t motors_array_t[2];

inline int imit_fifo_size() { return IMIT_BUF_SIZE; }

BOOL imit_fifo_full();
BOOL imit_fifo_empty();

int imit_fifo_count();
int imit_fifo_free();

int imit_fifo_add(const motor_t* const data);
motor_t* imit_fifo_q();
int imit_fifo_rdack();

void imit_fifo_clear();

void motor_print(const motor_t* m);

#endif /* INC_IMIT_FIFO_H_ */
