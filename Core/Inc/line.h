#ifndef INC_LINE_H_
#define INC_LINE_H_

#include "my_types.h"

typedef struct {
	BOOL valid;
	fpoint_t A, B;
} gline_t;

typedef struct {
//	AXIS_T lead_axis;
	BOOL valid;
	fpoint_t A, B;
	double dx, dy;
	double c;
	double k, b, k_rev, b_rev;
	double length, step;
} line_t;

void gline_clear(gline_t* const gline);

void line_clear(line_t* const line);
void line_init(line_t* const line, const fpoint_t* const A, const fpoint_t* const B, BOOL reverse, double step);
void line_setStepRatio(line_t* const line, double base_length);
void line_swap(line_t* const line);
size_t line_setPoint(const line_t* const line, point_t* pt);
BOOL line_empty(const line_t* const line);
BOOL line_isPoint(const line_t* const line, const scale_t* const scale);

fpoint_t line_getPoint(const line_t* const line, size_t step_id, BOOL* const is_last, BOOL* const valid);
size_t line_getPos(const line_t* const line, const fpoint_t* const pt, BOOL* const is_last);

double line_y(const line_t* const line, const double x);
double line_x(const line_t* const line, const double y);

//fpoint_t line_err(const line_t* const line, const point_t* const cur_pos);
//
//DIRECTION_T line_getDirX(const line_t* const line, const point_t* const pt);
//DIRECTION_T line_getDirY(const line_t* const line, const point_t* const pt);
//
//DIRECTION_T line_getMainDirX(const line_t* const line);
//DIRECTION_T line_getMainDirY(const line_t* const line);

void gline_print(const gline_t* const gline);

void line_test();

double line_remain(const line_t* const line, size_t step_id);

#endif /* INC_LINE_H_ */
