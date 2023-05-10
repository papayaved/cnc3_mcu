#include "step_dir.h"

#include <stdlib.h>
#include <math.h>

#include "fpga.h"
#include "imit_fifo.h"
#include "cnc_task.h"
#include "encoder.h"
#include "cnc_param.h"

volatile BOOL imit_ena;
volatile uint32_t T_min = T_MIN_DEFAULT, T_max = T_MAX_DEFAULT;

void step_abort() { fpga_motorAbort(); }

void step_reset() {
	imit_ena = FALSE;
	T_min = T_MIN_DEFAULT;
	T_max = T_MAX_DEFAULT;
	step_abort();
}

BOOL step_ready() {
#ifdef IMIT_FIFO_ENABLE
	return !imit_fifo_full() && fpga_getReady();
#else
	return fpga_getWrRdy();
#endif
}

BOOL step_isStopped() {
	uint16_t res = fpga_getRun();
	return res == 0;
}

//void step_block() { fpga_setRunEnable(FALSE); }
//void step_unblock() { fpga_setRunEnable(TRUE); }

void step_setImitEna(BOOL value) { if (!cnc_run()) imit_ena = value != 0; }
BOOL step_getImitEna() {
#ifdef IMIT_FIFO_ENABLE
	return imit_ena;
#else
	return FALSE;
#endif
}

void read_imit_fifo_task() {
#ifdef IMIT_FIFO_ENABLE
	static motor_t* m;

	if (!imit_fifo_empty()) {
		m = imit_fifo_q();
		imit_fifo_rdack();
		motor_print(m);
	}
#endif
}

// T - pause ticks, default tick - 0.1 ms
void step_writePause(uint32_t str_num, uint32_t T) {
	fpga_setTaskID(str_num);
	fpga_step(0, 0, T);
	fpga_write_u16(MTR_WRREQ>>1, 1<<0);
}

void step_writeTaskId(uint32_t str_num) {
	fpga_setTaskID(str_num);
	fpga_step(0, 0, 0);
	fpga_write_u16(MTR_WRREQ>>1, 1<<0);
}

void step_write(uint32_t str_num, BOOL frame_sync, AXIS_T axis, BOOL fin, int32_t N, uint32_t T) {
	if (!fin && N != 0)
		if (axis < MOTORS) {
			fpga_setTaskID(str_num);
			fpga_step(axis, N, T);
			fpga_write_u16(MTR_WRREQ>>1, 1<<axis);
		}

#ifdef IMIT_FIFO_ENABLE
	motor_t m;
	m.num = str_num;
	m.axis = axis;
	m.sync = frame_sync;
	m.mode = 0;
	m.last = fin;
	m.N = N;
	m.T = T;

	if (imit_ena && (N != 0 || fin)) {
#ifdef SUM_IMIT_STEPS
		static motor_t m_reg;
		static BOOL init;

		if (init) {
			if (fin) {
				m_reg.last = fin;
				imit_fifo_add(&m_reg);
				init = FALSE;
			}
			else if (m_reg.axis == axis && m_reg.num == num && m_reg.sync == frame_sync && m_reg.T == T)
				m_reg.N += N;
			else {
				imit_fifo_add(&m_reg);
				m_reg = m;
			}
		}
		else {
			m_reg = m;
			init = TRUE;
		}
#else
		imit_fifo_add(&m);
#endif
	}
#endif

//	if (axis < MOTORS)
//		motors[axis] = m;

//	if (axis == AXIS_X) {
//		pos.x += N;
//		path.x += abs(N);
//	}
//	else if (axis == AXIS_Y) {
//		pos.y += N;
//		path.y += abs(N);
//	}
}

BOOL step_write_reg(unsigned i, double ts, int32_t N) {
	if (i < MTR_NUM && N) {
		unsigned T = (unsigned)round( abs(ts / N) );
		T = MAX(T, T_min);
		T = MIN(T, T_max);

		fpga_step((uint8_t)i, N, T);
		return TRUE;
	}

	return FALSE;
}

//void step_write_req(uint32_t str_num) {
//	fpga_setTaskID(str_num);
//	fpga_write_u16(MTR_WRREQ>>1, 1);
//}

void step_writeXYUV(uint32_t str_num, double ts, int32_t Nx, int32_t Ny, int32_t Nu, int32_t Nv) {
	uint8_t wrreq = step_write_reg(0, ts, Nx) ? 1 : 0;

	if (step_write_reg(1, ts, Ny)) wrreq |= 2;
	if (step_write_reg(2, ts, Nu)) wrreq |= 4;
	if (step_write_reg(3, ts, Nv)) wrreq |= 8;

	if (wrreq) {
		fpga_setTaskID(str_num);
		fpga_write_u16(MTR_WRREQ>>1, wrreq);
	}
}

//void step_writeXY(uint32_t str_num, uint32_t ts, int32_t Nx, int32_t Ny) {
//	step_writeXYUV(str_num, ts, Nx, Ny, 0, 0);
//}

//motors_array_t* alt_step_read() { return &motors; }

// stop FPGA and set position. G92
void step_setPos(int str_num, const point_t* const pt, const point_t* const uv_pt) {
	fpga_setSoftPermit(FALSE);
	step_writeTaskId(str_num);
	fpga_setPos(0, pt->x);
	fpga_setPos(1, pt->y);
	fpga_setPos(2, uv_pt->x);
	fpga_setPos(3, uv_pt->y);
	fpga_setSoftPermit(TRUE);
}

// stop FPGA and read context
context_t step_getContext() {
	context_t ctx;
	fpga_setSoftPermit(FALSE);
	ctx.str_num = fpga_getTaskID();
	ctx.pt.x = fpga_getPos(0);
	ctx.pt.y = fpga_getPos(1);
	ctx.uv_pt.x = fpga_getPos(2);
	ctx.uv_pt.y = fpga_getPos(3);
	fpga_setSoftPermit(TRUE);
	return ctx;
}

void step_clear() {
	fpga_setSoftPermit(FALSE);

	for (int i = 0; i < 4; i++) {
		fpga_setPos(i, 0);
	}

	enc_setXY(0, 0);

	fpga_setSoftPermit(TRUE);
}

// Set the maximum motor's speed, in FPGA ticks
void step_setTmin(uint32_t T) {
	T = MAX(T, T_MIN);
	T_min = MIN(T, T_max);
}

void step_setTmax(uint32_t T) {
	T = MIN(T, T_MAX);
	T_max = MAX(T, T_min);
}

uint32_t step_Tmin() { return T_min; }
uint32_t step_Tmax() { return T_max; }

//static double F_reg = 0; // mm / clock
//static int32_t nx_reg, ny_reg, nu_reg, nv_reg;
//
//void step_writeXYUV_A(uint32_t str_num, double ts, int32_t nx, int32_t ny, int32_t nu, int32_t nv, BOOL reset) {
//	static double dx, dy, d, F, dF, acc;
//
//	if (reset) {
//		F_reg = 0;
//		nx_reg = 0;
//		ny_reg = 0;
//	}
//
//	dx = steps_to_mm(nx, cnc_scale_x());
//	dy = steps_to_mm(ny, cnc_scale_y());
//	d = sqrt(dx*dx + dy*dy); // mm
//
//	F = d / ts; // mm / clocks
//	dF = F - F_reg;
//	acc = dF / ts; // mm / clock^2
//
//	if (acc >= 0) {
//		if (acc > cnc_acc())
//			ts = dF / cnc_acc();
//
//		F_reg = F;
//		nx_reg = nx;
//		ny_reg = ny;
//
//		step_writeXYUV(str_num, ts, nx, ny, nu, nv);
//		return;
//	}
//	else
//		dF = cnc_dec();
//
//}
//
//void step_brake(uint32_t str_num, int32_t* const p_nx, int32_t* const p_ny, int32_t* const p_nu, int32_t* const p_nv) {
//	static int32_t nx, ny, nu, nv;
//	static double ts, F, d, dx_reg, dy_reg, d_reg, k;
//
//	ts = F_reg / cnc_dec();
//	F = 0.5 * F_reg;
//	d = F * ts;
//
//	dx_reg = steps_to_mm(nx_reg, cnc_scale_x());
//	dy_reg = steps_to_mm(ny_reg, cnc_scale_y());
//	d_reg = sqrt(dx_reg * dx_reg + dy_reg * dy_reg); // mm
//
//	k = d / d_reg;
//
//	nx = k * nx_reg;
//	ny = k * ny_reg;
//	nu = k * nu_reg;
//	nv = k * nv_reg;
//
//	step_writeXYUV(str_num, ts, nx, ny, nu, nv);
//
//	F_reg = 0;
//	nx_reg = ny_reg = nu_reg = nv_reg = 0;
//
//	if (p_nx) *p_nx = nx;
//	if (p_ny) *p_ny = ny;
//	if (p_nu) *p_nu = nu;
//	if (p_nv) *p_nv = nv;
//}
