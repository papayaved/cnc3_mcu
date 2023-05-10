#ifndef INC_ARC_H_
#define INC_ARC_H_

#include "my_types.h"

typedef struct {
	struct {
		uint8_t valid:1;
		uint8_t ccw:1;
		uint8_t R:1;
	} flag;
	fpoint_t A, B, C; // mm
	double R;
} garc_t;

typedef struct {
	uint8_t Cx:1;
	uint8_t Cy:1;
	uint8_t R:1;
	uint8_t alpha:1;
	uint8_t beta:1;
	uint8_t ccw:1;
	uint8_t valid:1;
	uint8_t empty:1;
} arc_flags_t;

typedef struct {
	arc_flags_t flag;
	fpoint_t A, B, C; // mm
	double R, alpha, beta, delta; // mm, rad, rad, rad
	double step_rad;
} arc_t;

typedef enum { ARC_Q1, ARC_Q2, ARC_Q3, ARC_Q4 } ARC_QUARTER_T;

void arc_initCenter(arc_t* const arc, const fpoint_t* const A, const fpoint_t* const B, const fpoint_t* const C, BOOL ccw, double step, const scale_t* const scale);
void arc_initRadius(arc_t* const arc, const fpoint_t* const A, const fpoint_t* const B, double R, BOOL ccw, double step, const scale_t* const scale);

double arc_length(const arc_t* const arc);
void arc_setStepRatio(arc_t* const arc, double base_length);

void polar(const fpoint_t* const C, const fpoint_t* const A, double* const R, double* const Angle, const scale_t* const scale);
double angle(const fpoint_t* const pC, const fpoint_t* const pA, const scale_t* const scale);

double range360(double value);
double deltaRadian(double a, double b, BOOL ccw);

fpoint_t arc_getPoint(const arc_t* const arc, size_t step_id, BOOL* const is_last, BOOL* const valid);
size_t arc_getPos(const arc_t* const arc, const fpoint_t* const pt, const scale_t* const scale, BOOL* const is_last);

double arc_y(const arc_t* const arc, const double x);
double arc_x(const arc_t* const larc, const double y);
fpoint_t arc_error(const arc_t* const arc, const fpoint_t* const pt);

dir_t arc_getDir(const arc_t* const arc, const fpoint_t* const pt, const scale_t* const scale);

main_dir_t arc_getMainDir(const arc_t* const arc, const fpoint_t* const pt);

void arc_swap(arc_t* const arc);

void garc_clear(garc_t* const garc);
void arc_clear(arc_t* const arc);
void arc_clearFlags(arc_t* const arc);

BOOL arc_empty(const arc_t* const arc);
BOOL arc_isCycle(const arc_t* const arc);
BOOL arc_isLast(const arc_t* const arc, const fpoint_t* const pt);

void garc_print(const garc_t* const garc);
void arc_print(const arc_t* const arc);

double arc_remain(const arc_t* const arc, size_t step_id);

#endif /* INC_ARC_H_ */
