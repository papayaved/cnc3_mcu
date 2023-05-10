#include "cnc_func.h"
#include "defines.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <ctype.h>
#include <fpga_gpo.h>
#include <math.h>
#include <limits.h>

int32_t double_to_int32(double value) {
	value = round(value);

	if (value > (int32_t)INT32_MAX)
		return (int32_t)INT32_MAX;

	if (value < (int32_t)INT32_MIN)
		return (int32_t)INT32_MIN;

	return (int32_t)value;
}

// scale - steps / mm
double fmm_to_steps(double value, double scale) {
	return value * scale;
}

// scale - steps / mm
int32_t mm_to_steps(double value, double scale) {
	return double_to_int32( value * scale );
}

// scale - steps / mm
double fsteps_to_mm(double value, double scale) {
	return value / scale;
}

// scale - steps / mm
double steps_to_mm(int32_t value, double scale) {
	return (double)value / scale;
}

double steps_to_mm2(int32_t nx, int32_t ny, const scale_t* const scale_xy) {
	double x = steps_to_mm(nx, scale_xy->x);
	double y = steps_to_mm(ny, scale_xy->y);
	return sqrt(x*x + y*y);
}

fpoint_t steps_to_fpoint_mm(const point_t* const mtr_pt, const scale_t* const scale) {
	fpoint_t res = { steps_to_mm(mtr_pt->x, scale->x), steps_to_mm(mtr_pt->y, scale->y) };
	return res;
}

point_t fpoint_mm_to_steps(const fpoint_t* const pt_mm, const scale_t* const scale) {
	point_t res = { mm_to_steps(pt_mm->x, scale->x), mm_to_steps(pt_mm->y, scale->y) };
	return res;
}

/* F - mm / min
 * result - clock / mm
 */
double speed_to_period(double F) {
	if (F < F_MIN_MMM) {
		return T_MAX_TICK;
	}
	else if (F > F_MAX_MMM) {
		return T_MIN_TICK;
	}

//	if (F < F_MIN_MMM || F > F_MAX_MMM) {
////		printf("T:%d (default)\n", (int)T_DEFAULT_TICK);
//		return T_MIN_TICK;
//	}

	double res = (FPGA_CLOCK * 60.0) / F;
//	printf("T:%d (default)\n", (int)res);
	return res; // ticks / mm
}

/* T - clock / mm
 * result - mm / min
 */
double period_to_speed(double T) { return (FPGA_CLOCK * 60.0) / T; }

/* T - clock / mm
 * result - um / sec
 */
float period_to_ums(float T) { return (FPGA_CLOCK * 1000) / T; }

/* F - mm / min
 * result - mm / clock
 */
double mmm_to_mmclock(double F) {
	if (F < F_MIN_MMM)
		F = F_MIN_MMM;
	else if (F > F_MAX_MMM)
		F = F_MAX_MMM;

	return F * (1.0 / (FPGA_CLOCK * 60.0));
}

/* F - mm / clock
 * result - mm / min
 */
double mmclock_to_mmm(double F) { return F * (FPGA_CLOCK * 60.0); }

double mmm_to_ums(double mmm) { return mmm * (1000 / 60); }

double iflength(int32_t nx, int32_t ny) {
	double x = nx;
	double y = ny;
	return sqrt(x*x + y*y);
}

// mm/clock
float speed_range(float value) {
	return value > F_MAX ? F_MAX : value < F_MIN ? F_MIN : value;
}

// clock/mm
double T_range(double value) {
	return value > T_MAX_TICK ? T_MAX_TICK : value < T_MIN_TICK ? T_MIN_TICK : value;
}
