#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <ctype.h>
#include <math.h>

#include "arc.h"
#include "aux_func.h"
#include "defines.h"

static const arc_flags_t ARC_EMPTY_FLAGS = {0, 0, 0, 0, 0, 0, 0, 1};

// all in mm
void arc_initCenter(arc_t* const pArc, const fpoint_t* const pA, const fpoint_t* const pB, const fpoint_t* const pC, BOOL ccw, double step, const scale_t* const pScale) {
	pArc->flag = ARC_EMPTY_FLAGS;
	pArc->flag.valid = 1;
	pArc->flag.ccw = ccw;

	pArc->A = *pA;
	pArc->B = *pB;
	pArc->C = *pC;

	if (step < STEP_MIN)
		step = STEP_MIN;

	if (fpoint_cmp(&pArc->A, &pArc->B, pScale)) { // A == B - circle
    	if (fpoint_cmp(&pArc->A, &pArc->C, pScale)) { // A == B == C - point
            pArc->R = 0;
            pArc->alpha = 0;
            pArc->beta = 0;
            pArc->flag.empty = 1;
        	pArc->delta = 0;
        	pArc->step_rad = 0;
            return;
    	}
    	else {
			polar(&pArc->C, &pArc->A, &pArc->R, &pArc->alpha, pScale);
			pArc->beta = pArc->alpha;

			if (pArc->R * pScale->x >= 0.5 && pArc->R * pScale->y >= 0.5) {
				pArc->delta = 2 * M_PI;
				pArc->step_rad = step / pArc->R;
				pArc->flag.empty = 0;
			}
			else {
	        	pArc->delta = 0;
	        	pArc->step_rad = 0;
	        	pArc->flag.empty = 1;
			}
			return;
    	}
    }
    else if (fpoint_cmp(&pArc->A, &pArc->C, pScale) || fpoint_cmp(&pArc->B, &pArc->C, pScale)) { // A == C || B == C - error -> semicircle (todo: line R == infinite)
        pArc->C.x = (pArc->A.x + pArc->B.x) / 2.0;
        pArc->C.y = (pArc->A.y + pArc->B.y) / 2.0;

    	polar(&pArc->C, &pArc->A, &pArc->R, &pArc->alpha, pScale);
        pArc->beta = range360(pArc->alpha + M_PI);

		if (pArc->R * pScale->x >= 0.5 && pArc->R * pScale->y >= 0.5) {
			pArc->delta = 2 * M_PI;
			pArc->step_rad = step / pArc->R;
			pArc->flag.empty = 0;
		}
		else {
        	pArc->delta = 0;
        	pArc->step_rad = 0;
        	pArc->flag.empty = 1;
		}

        return;
    }
    else { // Main
        double R, R2;

        polar(&pArc->C, &pArc->A, &R, &pArc->alpha, pScale);
        polar(&pArc->C, &pArc->B, &R2, &pArc->beta, pScale);

        R = (R + R2) / 2;

        double delta = deltaRadian(pArc->alpha, pArc->beta, ccw);

        if (delta > M_PI)
        	R = -R;

        arc_initRadius(pArc, pA, pB, R, ccw, step, pScale);
    }
}

void arc_initRadius(arc_t* const pArc, const fpoint_t* const pA, const fpoint_t* const pB, double R, BOOL ccw, double step, const scale_t* const pScale) {
	pArc->flag = ARC_EMPTY_FLAGS;
	pArc->flag.valid = 1;
	pArc->flag.ccw = ccw;

	pArc->A = *pA;
	pArc->B = *pB;

    BOOL biggest = R < 0;
    R = fabs(R);

	if (step < STEP_MIN)
		step = STEP_MIN;

    double AB, a;
    polar(pA, pB, &AB, &a, pScale);

    fpoint_t D;
    D.x = (pA->x + pB->x) / 2.0;
    D.y = (pA->y + pB->y) / 2.0;

    double AB_2 = AB / 2.0;

    if (R < AB_2)
    	R = AB_2;

    double L = sqrt(R * R - AB_2 * AB_2);
    double dx, dy;

//    if (ccw) a += M_PI;

	dx = L * sin(a);
	dy = - L * cos(a);

    if (ccw ^ biggest) {
		dx = -dx;
		dy = -dy;
    }

    pArc->C.x = D.x + dx;
    pArc->C.y = D.y + dy;

    pArc->alpha = angle(&pArc->C, &pArc->A, pScale);
    pArc->beta = angle(&pArc->C, &pArc->B, pScale);
    pArc->R = R;

	if (pArc->R * pScale->x >= 0.5 && pArc->R * pScale->y >= 0.5) {
		pArc->delta = deltaRadian(pArc->alpha, pArc->beta, pArc->flag.ccw);
		pArc->step_rad = step / pArc->R;
		pArc->flag.empty = 0;
	}
	else {
    	pArc->delta = 0;
    	pArc->step_rad = 0;
    	pArc->flag.empty = 1;
	}
}

double arc_length(const arc_t* const arc) {
	return arc->flag.valid ? arc->delta * arc->R : 0;
}

void arc_setStepRatio(arc_t* const arc, double base_length) {
	double k = arc_length(arc) / base_length;
	arc->step_rad *= k;
}

void polar(const fpoint_t* const pC, const fpoint_t* const pA, double* const pR, double* const pAngle, const scale_t* const scale) {
    if ( !fpoint_cmp(pA, pC, scale) ) { // A != C
        double dx = pA->x - pC->x;
        double dy = pA->y - pC->y;

        *pR = sqrt(dx * dx + dy * dy);
        *pAngle = asin(dy / *pR); // [-90, +90]

        if (*pAngle < 0)
            if (dx < 0)
                *pAngle = M_PI - *pAngle;
            else
                *pAngle = 2 * M_PI + *pAngle;
        else if (dx < 0)
            *pAngle = M_PI - *pAngle;

        if (*pAngle <= 0 || *pAngle >= 2 * M_PI)
        	*pAngle = 0;
    }
    else {
        *pR = 0;
        *pAngle = 0;
    }
}

double angle(const fpoint_t* const pC, const fpoint_t* const pA, const scale_t* const scale) {
	double a;

    if ( !fpoint_cmp(pA, pC, scale) ) { // A != C
        double dx = pA->x - pC->x;
        double dy = pA->y - pC->y;

        a = atan2(dy, dx); // [-M_PI, M_PI]

        a = range360(a);

//        if (a < 0)
//            if (dx < 0)
//                a = M_PI - a;
//            else
//                a = 2 * M_PI + a;
//        else if (dx < 0)
//            a = M_PI - a;
//
//        if (a <= 0 || a >= 2 * M_PI)
//        	a = 0;
    }
    else
    	a = 0;

    return a;
}

double range360(double value) {
    const double M_360 = 2 * M_PI;

    if (value >= M_360) {
        while (value > M_360)
            value -= M_360;

        if (value < 0) value = 0;
    }
    else if (value < 0) {
        while (value < 0)
            value += M_360;

        if (value >= M_360) value = 0;
    }
    return value;
}

double deltaRadian(double a, double b, BOOL ccw) {
	double d;
//	a = range360(a);
//	b = range360(b);

	if (ccw)
		d = b - a;
	else
		d = a - b;

	d = range360(d);

	return d;
}

double arc_radiusError(const arc_t* const arc, const fpoint_t* const pt) {
	double dX = (double)pt->x - arc->C.x;
	double dY = (double)pt->y - arc->C.y;
	double R = sqrt(dX * dX + dY * dY);
	return R - arc->R;
}

fpoint_t arc_error(const arc_t* const arc, const fpoint_t* const pt) {
	fpoint_t err;

	double Lx = (double)pt->x - arc->C.x; // from center
	double Ly = (double)pt->y - arc->C.y;

	double R = sqrt(Lx * Lx + Ly * Ly);
	double dR = R - arc->R;

	double k = dR / R;

	err.x = Lx * k;
	err.y = Ly * k;

	return err;
}

ARC_QUARTER_T arc_getQuater(const arc_t* const arc, const fpoint_t* const pt) {
	double dx = pt->x - arc->C.x;
	double dy = pt->y - arc->C.y;

	if (arc->flag.ccw)
		if (dx > 0 && dy >= 0)
			return ARC_Q1;
		else if (dx <= 0 && dy > 0)
			return ARC_Q2;
		else if (dx < 0 && dy <= 0)
			return ARC_Q3;
		else if (dx >= 0 && dy < 0)
			return ARC_Q4;
		else
			return ARC_Q1;
	else
		if (dx > 0 && dy <= 0)
			return ARC_Q4;
		else if (dx <= 0 && dy < 0)
			return ARC_Q3;
		else if (dx < 0 && dy >= 0)
			return ARC_Q2;
		else if (dx >= 0 && dy > 0)
			return ARC_Q1;
		else
			return ARC_Q4;
}

double arc_getdAngle(const arc_t* const arc, const fpoint_t* const pt, const scale_t* const scale) {
	double gamma = angle(&arc->C, pt, scale);
	return gamma - arc->beta;
}

fpoint_t arc_getTarget(const arc_t* const arc, const fpoint_t* const pt, ARC_QUARTER_T* quarter, const scale_t* const scale) {
	fpoint_t T;
	ARC_QUARTER_T q, qB;

	q = arc_getQuater(arc, pt);
	if (quarter) *quarter = q;

	qB = arc_getQuater(arc, &arc->B);

	if (q == qB) {
		BOOL last = arc_isLast(arc, pt);

		if (last) {
			T.x = arc->B.x;
			T.y = arc->B.y;
			return T;
		}

		double delta = arc_getdAngle(arc, pt, scale);

		if ((arc->flag.ccw && delta <= 0) || (!arc->flag.ccw && delta >= 0)) {
			T.x = arc->B.x;
			T.y = arc->B.y;
			return T;
		}
	}

	if (arc->flag.ccw)
		switch (q) {
		case ARC_Q2:
			T.x = arc->C.x - arc->R;
			T.y = arc->C.y;
			break;
		case ARC_Q3:
			T.x = arc->C.x;
			T.y = arc->C.y - arc->R;
			break;
		case ARC_Q4:
			T.x = arc->C.x + arc->R;
			T.y = arc->C.y;
			break;
		case ARC_Q1: default:
			T.x = arc->C.x;
			T.y = arc->C.y + arc->R;
			break;
		}
	else
		switch (q) {
		case ARC_Q1:
			T.x = arc->C.x + arc->R;
			T.y = arc->C.y;
			break;
		case ARC_Q2:
			T.x = arc->C.x;
			T.y = arc->C.y + arc->R;
			break;
		case ARC_Q3:
			T.x = arc->C.x - arc->R;
			T.y = arc->C.y;
			break;
		case ARC_Q4: default:
			T.x = arc->C.x;
			T.y = arc->C.y - arc->R;
			break;
		}

	return T;
}

fpoint_t arc_getPoint(const arc_t* const arc, size_t step_id, BOOL* const is_last, BOOL* const valid) {
	if (!arc->flag.valid) { // error
		fpoint_t res = {0, 0};
		*is_last = TRUE;
		*valid = FALSE;
		return res;
	}

	double delta_phi = step_id * arc->step_rad;
	*valid = TRUE;

	if (delta_phi >= fabs(arc->delta) || arc->flag.empty) {
		fpoint_t res = {arc->B.x, arc->B.y};
		*is_last = TRUE;
		return res;
	}

	if (!arc->flag.ccw)
		delta_phi = -delta_phi;

	double phi = arc->alpha + delta_phi;

	phi = range360(phi);
	fpoint_t res = {arc->R * cos(phi) + arc->C.x, arc->R * sin(phi) + arc->C.y};
	*is_last = FALSE;

	return res;
}

size_t arc_getPos(const arc_t* const arc, const fpoint_t* const pt, const scale_t* const scale, BOOL* const is_last) {
	double R, phi;
	size_t index, index_max;

	polar(&arc->C, pt, &R, &phi, scale);

	double delta_phi = deltaRadian(arc->alpha, phi, arc->flag.ccw);

	double k = 1.0 / arc->step_rad;

	index_max = (size_t)round(arc->delta * k);

	if (delta_phi > arc->delta)
		index = index_max;
	else
		index = (size_t)round(delta_phi * k); // ?? floor

	*is_last = index >= index_max;

	return index;
}

// for first step use main dir
dir_t arc_getDir(const arc_t* const arc, const fpoint_t* const pt, const scale_t* const scale) {
	ARC_QUARTER_T q;
	dir_t dir;
	fpoint_t B_err;
	fpoint_t T, T_err;

	T = arc_getTarget(arc, pt, &q, scale);

	T_err.x = pt->x - T.x;
	T_err.y = pt->y - T.y;

	B_err.x = pt->x - arc->B.x;
	B_err.y = pt->y - arc->B.y;

	if (arc->flag.ccw) {
		dir.x = B_err.x == 0 && fabs(T_err.x) < 1 ? STAY : q == ARC_Q1 || q == ARC_Q2 ? BACKWARD : FORWARD;
		dir.y = B_err.y == 0 && fabs(T_err.y) < 1 ? STAY : q == ARC_Q1 || q == ARC_Q4 ? FORWARD : BACKWARD;
	}
	else {
		dir.x = B_err.x == 0 && fabs(T_err.x) < 1 ? STAY : q == ARC_Q1 || q == ARC_Q2 ? FORWARD : BACKWARD;
		dir.y = B_err.y == 0 && fabs(T_err.y) < 1 ? STAY : q == ARC_Q1 || q == ARC_Q4 ? BACKWARD : FORWARD;
	}

	return dir;
}

//DIRECTION_T arc_getDirX(const arc_t* const arc, const point_t* const pt) {
//	fpoint_t T = arc_getTarget(arc, pt);
//
//	double err = (double)pt->x - T.x;
//
//	if (fabs(err) <= 0.5)
//		return STAY;
//
//	double dy = pt->y - arc->C.y;
//
//	if (arc->flags.ccw == 0)
//		return dy >= 0 ? FORWARD : BACKWARD;
//	else
//		return dy >= 0 ? BACKWARD : FORWARD;
//}
//
//DIRECTION_T arc_getDirY(const arc_t* const arc, const point_t* const pt) {
//	fpoint_t T = arc_getTarget(arc, pt);
//
//	double err = pt->y - T.y;
//
//	if (fabs(err) <= 0.5)
//		return STAY;
//
//	double dx = pt->x - arc->C.x;
//
//	if (arc->flags.ccw == 0)
//		return dx >= 0 ? BACKWARD : FORWARD;
//	else
//		return dx >= 0 ? FORWARD : BACKWARD;
//}

main_dir_t arc_getMainDir(const arc_t* const arc, const fpoint_t* const pt) {
	main_dir_t res = {STAY, AXIS_X};
	double dx = pt->x - arc->C.x;
	double dy = pt->y - arc->C.y;

	if (arc->flag.ccw) {
		if (dx > 0 && dy >= -dx && dy < dx) {
			res.axis = AXIS_Y;
			res.dir = FORWARD;
		}
		else if (dy > 0 && dx > -dy && dx <= dy) {
			res.axis = AXIS_X;
			res.dir = BACKWARD;
		}
		else if (dx < 0 && dy > -fabs(dx) && dy <= fabs(dx)) {
			res.axis = AXIS_Y;
			res.dir = BACKWARD;
		}
		else if (dy < 0 && dx >= -fabs(dy) && dx < fabs(dy)) {
			res.axis = AXIS_X;
			res.dir = FORWARD;
		}
	}
	else { // CW
		if (dx > 0 && dy > -dx && dy <= dx) {
			res.axis = AXIS_Y;
			res.dir = BACKWARD;
		}
		else if (dy < 0 && dx > -fabs(dy) && dx <= fabs(dy)) {
			res.axis = AXIS_X;
			res.dir = BACKWARD;
		}
		else if (dx < 0 && dy >= -fabs(dx) && dy < fabs(dx)) {
			res.axis = AXIS_Y;
			res.dir = FORWARD;
		}
		else if (dy > 0 && dx >= -dy && dx < dy) {
			res.axis = AXIS_X;
			res.dir = FORWARD;
		}
	}

	return res;
}

void arc_swap(arc_t* const arc) {
	arc->flag.ccw = !arc->flag.ccw;
	fpoint_swap(&arc->A, &arc->B);
	fswap(&arc->alpha, &arc->beta);
}

void garc_clear(garc_t* const garc) {
	memset(garc, 0, sizeof(garc_t));
}

void arc_clear(arc_t* const arc) {
	memset(arc, 0, sizeof(arc_t));
	arc->flag = ARC_EMPTY_FLAGS;
}

void arc_clearFlags(arc_t* const arc) {
	arc->flag = ARC_EMPTY_FLAGS;
}

BOOL arc_empty(const arc_t* const arc) {
	return !arc->flag.valid || arc->flag.empty;
}

BOOL arc_isCycle(const arc_t* const arc) {
	return arc->A.x == arc->B.x && arc->A.y == arc->B.y;
}

BOOL arc_isLast(const arc_t* const arc, const fpoint_t* const pt) {
	return pt->x == arc->B.x && pt->y == arc->B.y;
}

arc_flags_t flags;
double R, alpha, beta; // mm, rad, rad
fpoint_t A, B, C; // mm

void arc_print(const arc_t* const arc) {
	static decimal_t Cx, Cy, R, a, b;
	Cx = float2fix(arc->C.x);
	Cy = float2fix(arc->C.y);
	R = float2fix(arc->R);
	a = float2fix(rad2degree(arc->alpha));
	b = float2fix(rad2degree(arc->beta));

	printf("ARC: CCW:%d (%d, %d) - (%d, %d) C:(%s%d.%03d, %s%d.%03d) R:%s%d.%03d a:%s%d.%03d b:%s%d.%03d\n",
		(int)arc->flag.ccw,
		(int)arc->A.x, (int)arc->A.y, (int)arc->B.x, (int)arc->B.y,
		Cx.sign ? "-" : "", (int)Cx.value, (int)Cx.rem,
		Cy.sign ? "-" : "", (int)Cy.value, (int)Cy.rem,
		R.sign ? "-" : "", (int)R.value, (int)R.rem,
		a.sign ? "-" : "", (int)a.value, (int)a.rem,
		b.sign ? "-" : "", (int)b.value, (int)b.rem
	);
}

void garc_print(const garc_t* const garc) {
	static decimal_t Cx, Cy, R;

	if (garc)
		if (garc->flag.valid)
			if (garc->flag.R) {
				R = float2fix(garc->R);

				printf("ARC: CCW:%d (%d, %d) - (%d, %d) R:%s%d.%03d\n",
					(int)garc->flag.ccw,
					(int)garc->A.x, (int)garc->A.y, (int)garc->B.x, (int)garc->B.y,
					R.sign ? "-" : "", (int)R.value, (int)R.rem
				);
			}
			else {
				Cx = float2fix(garc->C.x);
				Cy = float2fix(garc->C.y);

				printf("ARC: CCW:%d (%d, %d) - (%d, %d) C:(%s%d.%03d, %s%d.%03d)\n",
					(int)garc->flag.ccw,
					(int)garc->A.x, (int)garc->A.y, (int)garc->B.x, (int)garc->B.y,
					Cx.sign ? "-" : "", (int)Cx.value, (int)Cx.rem,
					Cy.sign ? "-" : "", (int)Cy.value, (int)Cy.rem
				);
			}
		else
			printf("ARC: EMPTY\n");
	else
		printf("ARC: ERROR\n");
}

/*	The function calculates the remaining length
 * 	Return:
 * 		Remaining length, mm
 * 	Parameter:
 * 		*arc - current acr;
 *		step_id - new step id.
 */
double arc_remain(const arc_t* const arc, size_t step_id) {
	double rem_rad = fabs(arc->delta) - step_id * fabs(arc->step_rad);
	return rem_rad > 0 ? rem_rad * arc->R : 0;
}
