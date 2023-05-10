#ifndef INC_STEP_DIR_H_
#define INC_STEP_DIR_H_

#include "my_types.h"

//#define T_MIN (1000) // clocks per step - steps at 72kHz
#define T_MIN (2)
#define T_MAX (0x7FFFFFFF)
#define T_MIN_DEFAULT (FPGA_CLOCK / 300) // clocks per step. Steps frequency is 300 Hz
#define T_MAX_DEFAULT (T_MAX)

BOOL step_ready();
BOOL step_isStopped();

void step_block();
void step_unblock();
void step_setImitEna(BOOL value);
BOOL step_getImitEna();
void read_imit_fifo_task();

void step_writePause(uint32_t str_num, uint32_t T);
void step_writeTaskId(uint32_t str_num);
void step_write(uint32_t str_num, BOOL frame_sync, AXIS_T axis, BOOL fin, int32_t N, uint32_t T);
BOOL step_write_reg(unsigned index, double ts, int32_t N);
void step_write_req(uint32_t str_num);
void step_writeXYUV(uint32_t str_num, double ts, int32_t Nx, int32_t Ny, int32_t Nu, int32_t Nv);
//void step_writeXY(uint32_t str_num, uint32_t ts, int32_t Nx, int32_t Ny);
void step_setPos(int str_num, const point_t* const pt, const point_t* const uv_pt);
context_t step_getContext();
void step_clear();

void step_setTmin(uint32_t T);
void step_setTmax(uint32_t T);
uint32_t step_Tmin();
uint32_t step_Tmax();

#endif /* INC_STEP_DIR_H_ */
