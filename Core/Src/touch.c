#include "touch.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "fpga.h"
#include "step_dir.h"
#include "cnc_param.h"
#include "feedback.h"
#include "defines.h"
#include "cnc_func.h"
#include "fpga_gpo.h"

typedef enum {ST_IDLE, ST_FWD, ST_FWD_FINE0, ST_FWD_FINE1, ST_RB, ST_WAIT} touch_state_t;
static touch_state_t state, state_old;

struct {
	BOOL valid;
	point_t pt;
} touch_res;

point_t touch_result() { return touch_res.pt; }
int32_t touch_res_x() { return touch_res.pt.x; }
int32_t touch_res_y() { return touch_res.pt.y; }
BOOL touch_valid() { return touch_res.valid; }

typedef struct {
	uint8_t touches;
	double dx, dy, F, F_fine, fine_share;
} touch_task_t;

static touch_task_t task;
static int touch_cnt;

uint8_t touch() { return (uint8_t)touch_cnt; }
uint8_t touches() { return task.touches; }
int touch_getState() { return (int)state; }

void touch_init(double x, double y, uint8_t touches, double F, double F_fine, double fine_share) {
	task.dx = x;
	task.dy = y;
	task.touches = touches;
	task.F = F;
	task.F_fine = F_fine;
	task.fine_share = fine_share;
}

void touch_reset() {
	state = state_old = ST_IDLE;
	task.touches = 0;
	fpga_setCenterMode(FPGA_CENTER_MODE_OFF);
	fpga_clearCentered();
}

static int touch_error() {
	touch_reset();
	return -1;
}

// return: 0 - OK, 1 - finish, result < 0 - error
int touch_next(float* const T) {
	static int nx0, ny0, nx1, ny1, nx, ny, ret;
	static double k = 0, dx = 0, dy = 0, R = 0, R0 = 0, ts = 0;

	ret = 0;
	state_old = state;

	switch (state) {
	default:
		ret = touch_error();
		break;

	case ST_IDLE:
		if (task.touches) {
			touch_res.valid = FALSE;
			touch_res.pt.x = touch_res.pt.y = 0;

			k = 1.0 - task.fine_share;
			dx = task.dx * k;
			dy = task.dy * k;
			nx = mm_to_steps(dx, cnc_scaleX());
			ny = mm_to_steps(dy, cnc_scaleY());

			if (nx || ny) {
				touch_cnt = 1;

				fpga_globalSnapshot();
				nx0 = fpga_getPos(0);
				ny0 = fpga_getPos(1);

				R = sqrt(dx*dx + dy*dy); // mm
				*T = 1.0 / task.F; // clocks / mm
				ts = (*T) * R; // clocks

				fpga_setCenterMode(FPGA_CENTER_MODE_FORWARD);
				step_writeXYUV(0, ts, nx, ny, 0, 0);

#ifdef PRINT
				uint16_t run = fpga_getRun();
				uint8_t wrreq = run & 0xff;
				run >>= 8;

				printf("CNT:%d X:%d(%d) Y:%d(%d) fwd\n", (int)touch_cnt, (int)nx, (int)(nx ? round(ts/nx) : 0), (int)ny, (int)(ny ? round(ts/ny) : 0));
				printf("RD wrreq:%x run:%x\n", (int)wrreq, (int)run);
#endif

				state = ST_FWD;
			}
		}
		else
			ret = touch_error();

		break;

	case ST_FWD:
		if (step_isStopped()) {
			if (fpga_getCentered())
				state = ST_FWD_FINE1;
			else { // Fine speed
				dx = task.dx * task.fine_share;
				dy = task.dy * task.fine_share;
				nx = mm_to_steps(dx, cnc_scaleX());
				ny = mm_to_steps(dy, cnc_scaleY());

				if (nx || ny) {
					R = sqrt(dx*dx + dy*dy); // mm
					*T = 1.0 / task.F_fine; // clocks / mm
					ts = (*T) * R; // clocks

					fpga_setCenterMode(FPGA_CENTER_MODE_FORWARD);
					step_writeXYUV(0, ts, nx, ny, 0, 0);
#ifdef PRINT
					printf("CNT:%d X:%d(%d) Y:%d(%d) ffwd\n", (int)touch_cnt, (int)nx, (int)(nx ? round(ts/nx) : 0), (int)ny, (int)(ny ? round(ts/ny) : 0));
#endif
					state = ST_FWD_FINE0;
				}
				else
					ret = touch_error();
			}
		}
		break;

	case ST_FWD_FINE0:
		if (step_isStopped()) {
			if (fpga_getCentered())
				state = ST_FWD_FINE1;
				// next case
			else {
				ret = touch_error();
				break;
			}
		}
		else
			break;

	case ST_FWD_FINE1:
		fpga_globalSnapshot();
		nx1 = fpga_getPos(0);
		ny1 = fpga_getPos(1);

		touch_res.pt.x += nx1;
		touch_res.pt.y += ny1;

		if (touch_cnt < task.touches) {
			touch_cnt++;

			R = sqrt(task.dx*task.dx + task.dy*task.dy);
			k = TOUCH_RB / R;

			dx = -task.dx * k;
			dy = -task.dy * k;

			nx = mm_to_steps(dx, cnc_scaleX());
			ny = mm_to_steps(dy, cnc_scaleY());

			if (nx || ny) {
				*T = 1.0 / task.F_fine; // clocks / mm
				ts = (*T) * TOUCH_RB; // clocks

				fpga_setCenterMode(FPGA_CENTER_MODE_OFF);
				step_writeXYUV(0, ts, nx, ny, 0, 0);
				state = ST_RB;
#ifdef PRINT
				printf("CNT:%d X:%d(%d) Y:%d(%d) rev\n", (int)touch_cnt, (int)nx, (int)(nx ? round(ts/nx) : 0), (int)ny, (int)(ny ? round(ts/ny) : 0));
#endif
			}
			else
				ret = touch_error();
		}
		else {
			touch_res.pt.x /= task.touches;
			touch_res.pt.y /= task.touches;
			touch_res.valid = TRUE;

#ifdef PRINT
			printf("Touch %d %d\n", (int)touch_res.pt.x, (int)touch_res.pt.y);
#endif

			nx = touch_res.pt.x - nx1;
			ny = touch_res.pt.y - ny1;

			if (nx || ny) {
				dx = steps_to_mm(nx, cnc_scaleX());
				dy = steps_to_mm(ny, cnc_scaleY());
				R = sqrt(dx*dx + dy*dy);

				*T = 1.0 / task.F_fine; // clocks / mm
				ts = (*T) * R; // clocks

				fpga_setCenterMode(FPGA_CENTER_MODE_OFF);
				step_writeXYUV(0, ts, nx, ny, 0, 0);
				state = ST_WAIT;
#ifdef PRINT
				printf("CNT:%d X:%d(%d) Y:%d(%d) rev\n", (int)touch_cnt, (int)nx, (int)(nx ? round(ts/nx) : 0), (int)ny, (int)(ny ? round(ts/ny) : 0));
#endif
			}
			else {
				task.touches = 0;
				state = ST_IDLE;
				ret = 1;
			}
		}
		break;

	case ST_RB:
		if (step_isStopped()) {
			nx = nx1 - nx0;
			ny = ny1 - ny0;

			dx = steps_to_mm(nx, cnc_scaleX());
			dy = steps_to_mm(ny, cnc_scaleY());

			R = sqrt(dx*dx + dy*dy);
			R0 = sqrt(task.dx * task.dx + task.dy * task.dy);

			k = R / R0;
			k = k > 1.0 ? 0.0 : 1.0 - k;

			dx = task.dx * k;
			dy = task.dy * k;

			nx = mm_to_steps(dx, cnc_scaleX());
			ny = mm_to_steps(dy, cnc_scaleY());

			if (nx || ny) {
				R = R0 * k; // mm
				*T = 1.0 / task.F_fine; // clocks / mm
				ts = (*T) * R; // clocks

				fpga_setCenterMode(FPGA_CENTER_MODE_FORWARD);
				step_writeXYUV(0, ts, nx, ny, 0, 0);
				state = ST_FWD_FINE0;
#ifdef PRINT
				printf("CNT:%d X:%d(%d) Y:%d(%d) ffwd\n", (int)touch_cnt, (int)nx, (int)(nx ? round(ts/nx) : 0), (int)ny, (int)(ny ? round(ts/ny) : 0));
#endif
			}
			else
				ret = touch_error();
		}
		break;

	case ST_WAIT:
		if (step_isStopped()) {
			task.touches = 0;
			state = ST_IDLE;
			ret = 1;
		}
		break;
	}

#ifdef PRINT
	if (state != state_old) {
		switch (state) {
		case ST_IDLE:		printf("\t\tST_IDLE\n"); break;
		case ST_FWD:		printf("\t\tST_FWD\n"); break;
		case ST_FWD_FINE0:	printf("\t\tST_FWD_F0\n"); break;
		case ST_FWD_FINE1:	printf("\t\tST_FWD_F1\n"); break;
		case ST_RB:			printf("\t\tST_RB\n"); break;
		case ST_WAIT:		printf("\t\tST_WAIT\n"); break;
		default:			printf("\t\tstate: %d\n", (int)state); break;
		}
	}
#endif

	return ret;
}
