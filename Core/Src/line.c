#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <ctype.h>
#include <math.h>

#include "line.h"
#include "aux_func.h"
#include "cnc_func.h"
#include "defines.h"

void gline_clear(gline_t* const gline) {
	memset(gline, 0, sizeof(gline_t));
}

void line_clear(line_t* const line) {
	memset(line, 0, sizeof(line_t));
}

// k = dy / dx
// b = c / dx
void line_init(line_t* const pLine, const fpoint_t* const pA, const fpoint_t* const pB, BOOL reverse, double step) {
	if (reverse) {
		pLine->A = *pB;
		pLine->B = *pA;
	}
	else {
		pLine->A = *pA;
		pLine->B = *pB;
	}

	pLine->dx = pLine->B.x - pLine->A.x;
	pLine->dy = pLine->B.y - pLine->A.y;

//	if (!reverse) // for 45 degrees reverse movement alone same trajectory
//		line->lead_axis = abs(line->dx) >= abs(line->dy) ? AXIS_X : AXIS_Y;
//	else
//		line->lead_axis = abs(line->dx) > abs(line->dy) ? AXIS_X : AXIS_Y;

	pLine->c = pLine->B.x * pLine->A.y - pLine->A.x * pLine->B.y;

	pLine->k = pLine->dy / pLine->dx;
	pLine->b = pLine->c / pLine->dx;

	pLine->k_rev = 1.0 / pLine->k;
	pLine->b_rev = -pLine->c / pLine->dy;

	pLine->length = sqrt(pLine->dx * pLine->dx + pLine->dy * pLine->dy);

	pLine->step = step < STEP_MIN ? STEP_MIN : step;

	pLine->valid = TRUE;
}

void line_setStepRatio(line_t* const line, double base_length) {
	double k = line->length / base_length;
	line->step *= k;
}

BOOL line_empty(const line_t* const line) { return !line->valid; }

BOOL line_isPoint(const line_t* const line, const scale_t* const scale) {
	return fpoint_cmp(&line->A, &line->B, scale);
}

double line_y(const line_t* const line, const double x) {
	return (line->c + line->dy * x) / line->dx;
}
double line_x(const line_t* const line, const double y) {
	return (-line->c + line->dx * y) / line->dy;
}

void line_swap(line_t* const line) {
	double step = line->step;
	fpoint_t A = line->A;
	fpoint_t B = line->B;
	line_init(line, &A, &B, TRUE, 1);
	line->step = step;
}

//fpoint_t line_next_point(const line_t* const line, const fpoint_t* const cur_pt, double step) {
//	fpoint_t pt;
//	double k = step / line->L;
//
//	pt.x = cur_pt->x + k * line->dx;
//	pt.y = cur_pt->y + k * line->dy;
//
//	return pt;
//}

// Function calculates a position for given step_id
fpoint_t line_getPoint(const line_t* const line, size_t step_id, BOOL* const is_last, BOOL* const valid) {
	if (!line->valid) { // error
		fpoint_t res = {0, 0};
		*is_last = TRUE;
		*valid = FALSE;
		return res;
	}

	*valid = TRUE;

	if (line_empty(line)) {
		fpoint_t res = {line->B.x, line->B.y};
		*is_last = TRUE;
		return res;
	}

	double k = step_id * line->step / line->length;

	if (k >= 1.0) {
		fpoint_t res = {line->B.x, line->B.y};
		*is_last = TRUE;
		return res;
	}

	fpoint_t res = {line->A.x + k * line->dx, line->A.y + k * line->dy};

	if ( fabs(res.x - line->A.x) >= fabs(line->dx) ) {
		res.x = line->B.x;
		*is_last = TRUE;
	} else {
		*is_last = FALSE;
	}

	if ( fabs(res.y - line->A.y) >= fabs(line->dy) ) {
		res.y = line->B.y;
		*is_last &= TRUE;
	} else {
		*is_last = FALSE;
	}

	return res;
}

//static double iilen2(const point_t* const A, const point_t* const B) {
//	uint64_t dx = abs(B->x - A->x); // u32
//	uint64_t dy = abs(B->y - A->y);
//	return (double)(dx * dx) + (double)(dy * dy); // u64
//}

static double len2(const fpoint_t* const A, const fpoint_t* const B) {
	double dx = B->x - A->x;
	double dy = B->y - A->y;
	return dx * dx + dy * dy;
}

size_t line_getPos(const line_t* const line, const fpoint_t* const pt, BOOL* const is_last) {
	double AC2 = len2(&line->A, pt);
	double BC2 = len2(&line->B, pt);
	double AB2 = line->length * line->length;
	double denom = 2 * line->length * line->step;

	double index = (AB2 + AC2 - BC2) / denom;

	if (index <= 0) {
		index = 0;
		*is_last = FALSE;
	} else if (index * line->step > line->length) {
		index = floor(line->length / line->step);
		*is_last = TRUE;
	} else {
		index = round(index); // ?? floor
		*is_last = FALSE;
	}

	return (size_t)index;
}

// ??
fpoint_t line_err(const line_t* const line, const point_t* const cur_pos) {
	fpoint_t err;

	if (line->dx == 0) { // vertical
		if (line->dy == 0) {
			err.x = 0;
			err.y = 0;
		}
		else if (cur_pos->x != line->A.x) {
			err.x = line->B.x - cur_pos->x;
			err.y = 0;
		}
		else {
			err.x = 0;
			err.y = line->B.y - cur_pos->y;
		}
	}
	else if (line->dy == 0) { // horizontal
		if (cur_pos->y != line->A.y) {
			err.x = 0;
			err.y = line->B.y - cur_pos->y;
		}
		else {
			err.x = line->B.x - cur_pos->x;
			err.y = 0;
		}
	}
	else {
		double y = line_y(line, cur_pos->x);
		double x = line_x(line, cur_pos->y);
		err.y = y - cur_pos->y;
		err.x = x - cur_pos->x;
	}

	return err;
}

DIRECTION_T line_getDirX(const line_t* const line, const point_t* const pt) {
	int dx = line->B.x - pt->x;
	return dx < 0 ? BACKWARD : dx > 0 ? FORWARD : STAY;
}
DIRECTION_T line_getDirY(const line_t* const line, const point_t* const pt) {
	int dy = line->B.y - pt->y;
	return dy < 0 ? BACKWARD : dy > 0 ? FORWARD : STAY;
}

DIRECTION_T line_getMainDirX(const line_t* const line) {
	return line->dx < 0 ? BACKWARD : line->dx > 0 ? FORWARD : STAY;
}
DIRECTION_T line_getMainDirY(const line_t* const line) {
	return line->dy < 0 ? BACKWARD : line->dy > 0 ? FORWARD : STAY;
}

//int line_length_x(const line_t* const line) {
//	return line->B.x - line->A.x;
//}
//
//int line_length_y(const line_t* const line) {
//	return line->B.y - line->A.y;
//}

void gline_print(const gline_t* const gline) {
	if (gline) {
		if (gline->valid) {
			printf("GLINE: ");
			fpoint_print(&gline->A);
			printf(" - ");
			fpoint_print(&gline->B);
			printf("\n");
		}
		else
			printf("GLINE: EMPTY\n");
	}
	else
		printf("GLINE: NULL\n");
}

void line_print(const line_t* const line) {
	if (line) {
		printf("LINE: ");
		fpoint_print(&line->A);
		printf(" - ");
		fpoint_print(&line->B);

		printf(" dx:");
		mm_print(line->dx);

		printf(" dy:");
		mm_print(line->dy);

		printf(" c:");
		double_print(line->c);

		printf("\n");
	}
	else
		printf("LINE: NULL\n");
}

void line_test() {
	static line_t line;
	BOOL is_last;

	fpoint_t A = {1, 1};
	fpoint_t B = {5, 5};
	line_init(&line, &A, &B, FALSE, 1);
	line_print(&line);

	fpoint_t pt = {-4, 3};

	for (int k = 0; k < 20; k++) {
		fpoint_print(&pt);
		size_t i = line_getPos(&line, &pt, &is_last);
		printf("Line idx: %d last: %x\n", i, is_last);

		pt.x++;
	}
}

/*	The function calculates the remaining length
 * 	Return:
 * 		Remaining length, mm
 * 	Parameter:
 * 		*line - current line;
 *		step_id - new step id.
 */
double line_remain(const line_t* const line, size_t step_id) {
	double rem = line->length - step_id * line->step;
	return rem > 0 ? rem : 0;
}
