#include "uv.h"
#include <math.h>
#include <stdio.h>
#include "aux_func.h"
#include "cnc_func.h"

static double L = 0; // distance between top and bottom rollers, mm
static double H = 0; // height from bottom roller to workpiece bottom (XY), mm
static double T = 0; // workpiece thickness, mm
static BOOL valid = FALSE;

static AXIS_T D_axis = AXIS_Y; // roller plane
static DIR_T D_dir = DIR_MINUS; // wire side
static BOOL D_tilted_ena = FALSE; // wire side
static double R = 0; // roller diameter, mm
static BOOL D_valid = FALSE;

int32_t u_max = (int32_t)INT32_MAX, u_min = (int32_t)INT32_MIN; // todo
int32_t v_max = (int32_t)INT32_MAX, v_min = (int32_t)INT32_MIN;

void uv_setL(double value) {
	L = value > 0 ? value : 0;
	valid = FALSE;
#ifdef PRINT
	decimal_t x = float2fix(L);
	printf("L=%s%d.%03d\n", x.sign ? "-" : "", x.value, x.rem);
#endif
}
double uv_getL() { return L; }

void uv_setH(double value) {
	H = value > 0 ? value : 0;
	valid = FALSE;
#ifdef PRINT
	decimal_t x = float2fix(H);
	printf("H=%s%d.%03d\n", x.sign ? "-" : "", x.value, x.rem);
#endif
}
double uv_getH() { return H; }

void uv_setT(double value) {
	T = value > 0 ? value : 0;
	valid = FALSE;
#ifdef PRINT
	decimal_t x = float2fix(T);
	printf("T=%s%d.%03d\n", x.sign ? "-" : "", x.value, x.rem);
#endif
}
double uv_getT() { return T; }

void uv_clearLHT() {
	L = H = T = 0;
	valid = FALSE;
}

void uv_clearD() {
	D_axis = AXIS_Y;
	D_dir = DIR_MINUS;
	D_tilted_ena = FALSE;
	R = 0;
	D_valid = FALSE;
}

BOOL uv_enableLHT(BOOL ena) {
#ifndef STONE
	if (ena != 0 && L > 0 && H > 0 && T > 0 && H + T < L)
		valid = TRUE;
	else
		uv_clearLHT();

#ifdef PRINT
	printf("LHT:%x\n", valid);
#endif

#else
	valid = FALSE;
#endif

	return valid;
}

void uv_clear() {
	uv_clearLHT();
	uv_clearD();

#ifdef PRINT
	printf("LHT.CLR\n");
#endif
}

void uv_defaultParam() {
	L = UV_L;
	H = UV_H;
	T = UV_T;

	R = UV_D + UV_WIRE_D / 2; // diameter + half of wire diameter
	D_axis = AXIS_Y;
	D_dir = DIR_MINUS;
	D_tilted_ena = FALSE;

#ifndef STONE
	valid = TRUE;
	D_valid = FALSE;
#else
	valid = D_valid = FALSE;
#endif
}

BOOL uv_valid() { return valid; }

// This is a formula of a line. It gets 2 points (x,z) and returns the x coordinate for arbitrary z
static double line_xz(double x0, double x1, double z0, double z1, double z) {
	if (!valid || z1 == z0)
		return x0;

	return (x1 - x0) * (z - z0) / (z1 - z0) + x0;
}

// XY - Z = 0 plane
double uv_X_to_motor(double X, double U) {
	double dx = line_xz(0, U - X, 0, T, -H);
	double Xm = X + dx;
	return Xm;
}
double uv_U_to_motor(double X, double U) {
	double Um = (U - X) * L / T;
	return Um;
}
double uv_Y_to_motor(double Y, double V) {
	double dy = line_xz(0, V - Y, 0, T, -H);
	double Ym = Y + dy;
	return Ym;
}
double uv_V_to_motor(double Y, double V) {
	double Vm = (V - Y) * L / T;
	return Vm;
}

double uv_motor_to_X(double Xm, double Um)	{
	double dx = H * Um / L;
	double X = Xm + dx;
	return X;
}
double uv_motor_to_U(double Xm, double Um)	{
	double du = (H + T) * Um / L;
	double dx = H * Um / L;
	du -= dx;

	double X = Xm + dx;
	double U = X + du;
	return U;
}
double uv_motor_to_Y(double Ym, double Vm)	{
	double dy = H * Vm / L;
	double Y = Ym + dy;
	return Y;
}
double uv_motor_to_V(double Ym, double Vm)	{
	double dv = (H + T) * Vm / L;
	double dy = H * Vm / L;
	dv -= dy;

	double Y = Ym + dy;
	double V = Y + dv;
	return V;
}

fpoint_t uv_XY_to_motors(const fpoint_t* const XY, const fpoint_t* const UV) {
	fpoint_t XYm = {uv_X_to_motor(XY->x, UV->x), uv_Y_to_motor(XY->y, UV->y)};
	return XYm;
}

fpoint_t uv_UV_to_motors(const fpoint_t* const XY, const fpoint_t* const UV) {
	fpoint_t UVm = {uv_U_to_motor(XY->x, UV->x), uv_V_to_motor(XY->y, UV->y)};
	return UVm;
}

fpoint_t uv_motors_to_XY(const fpoint_t* const XYm, const fpoint_t* const UVm) {
	fpoint_t XY;
	XY.x = uv_motor_to_X(XYm->x, UVm->x);
	XY.y = uv_motor_to_Y(XYm->y, UVm->y);
	return XY;
}

fpoint_t uv_motors_to_UV(const fpoint_t* const XYm, const fpoint_t* const UVm) {
	fpoint_t UV;
	UV.x = uv_motor_to_U(XYm->x, UVm->x);
	UV.y = uv_motor_to_V(XYm->y, UVm->y);
	return UV;
}

point_t uv_limit(point_t pt) {
	if (pt.x > u_max)
		pt.x = u_max;
	if (pt.x < u_min)
		pt.x = u_min;

	if (pt.y > v_max)
		pt.y = v_max;
	if (pt.y < v_min)
		pt.y = v_min;

	return pt;
}

void uv_tb() {
	uv_setL( 190 );
	uv_setH( 50 );
	uv_setT( 30 );
	uv_enableLHT(TRUE);

//	fpoint_t XY = {3, 0};
//	fpoint_t UV = {5.368, 0};

	fpoint_t XY = {-10, -5};
	fpoint_t UV = {-15, -6};

//	fpoint_t XY = {10, 0};
//	fpoint_t UV = {8, 0};

	fpoint_t XYm = uv_XY_to_motors(&XY, &UV);
	fpoint_t UVm = uv_UV_to_motors(&XY, &UV);

	fpoint_t XY2 = uv_motors_to_XY(&XYm, &UVm);
	fpoint_t UV2 = uv_motors_to_UV(&XYm, &UVm);

//	decimal_t x = float2fix(XY.x);
//	decimal_t y = float2fix(XY.y);
//	decimal_t u = float2fix(UV.x);
//	decimal_t v = float2fix(UV.y);

	decimal_t x2 = float2fix(XY2.x);
	decimal_t y2 = float2fix(XY2.y);
	decimal_t u2 = float2fix(UV2.x);
	decimal_t v2 = float2fix(UV2.y);

	printf("[%d.%d, %d.%d, %d.%d, %d.%d]\n", x2.value, x2.rem, y2.value, y2.rem, u2.value, u2.rem, v2.value, v2.rem);
}

// Roller compensation

void uv_setD( double dia ) { R = 0.5 * dia; }
double uv_getD() { return 2 * R; }

void uv_setDAxis( BOOL axis ) { D_axis = axis != 0; }
AXIS_T uv_getDAxis() { return D_axis; }

void uv_setDDir(DIR_T dir) { D_dir = dir != 0; }
DIR_T uv_getDDir() { return D_dir; }

void uv_enableDTilted(BOOL ena) { D_tilted_ena = ena != 0; }
BOOL uv_D_tiltedEna() { return valid && D_valid && D_tilted_ena; }

BOOL uv_enableD(BOOL ena) {
#ifndef STONE
	if (ena != 0 && R >= (UV_D_MIN / 2) && R <= (UV_D_MAX / 2) && (D_axis == AXIS_X || D_axis == AXIS_Y))
		D_valid = TRUE;
	else
		uv_clearD();

#ifdef PRINT
	printf("Roller:%x\n", D_valid);
#endif

#else
	D_valid = FALSE;
#endif

	return D_valid;
}

BOOL uv_D_ena() { return valid && D_valid; }

fpoint_t roller_error_Y(const double* const p_dU, const double* const p_dV) {
	static BOOL sign_dU, sign_dV;
	static double dU, dV, L2, a, b;
	static fpoint_t err;

	sign_dU = *p_dU < 0;
	sign_dV = *p_dV < 0;

	dU = fabs(*p_dU);
	dV = fabs(*p_dV);

	L2 = L * L;
	a = sqrt(L2 + dV * dV);

	if (D_tilted_ena) {
		b = sqrt(L2 + dU * dU);

		err.y = R * (a - L) / b;
		err.x = 0;
	}
	else {
		err.y = R * (a / L - 1);
		err.x = R * dV * dU / (a * L);

		if (D_dir == DIR_MINUS) {
			if (sign_dV) {
				err.x = sign_dU ? -err.x : err.x;
			} else {
				err.x = sign_dU ? err.x : -err.x;
			}
		} else {
			if (sign_dV) {
				err.x = sign_dU ? err.x : -err.x;
			} else {
				err.x = sign_dU ? -err.x : err.x;
			}
		}
	}

	if (D_dir == DIR_MINUS)
		err.y = -err.y;

	return err;
}

// Error depend only on dU, dV and roller diameter
fpoint_t roller_error(double dU, double dV) {
	static fpoint_t err;

	if (D_axis == AXIS_Y)
		return roller_error_Y(&dU, &dV);

	err = roller_error_Y(&dV, &dU);
	swap_coord(&err);

	return err;
}
