#ifndef INC_UV_H_
#define INC_UV_H_

#include "my_types.h"

#define UV_L (150) // mm
#define UV_H (50) // mm
#define UV_T (30) // mm

#define UV_D_MIN (1)
#define UV_D_MAX (1000)
#define UV_D (29.5)
#define UV_WIRE_D (0.18)

extern int32_t u_max, u_min, v_max, v_min;

void uv_setL(double value);
double uv_getL();
void uv_setH(double value);
double uv_getH();
void uv_setT(double value);
double uv_getT();

BOOL uv_enableLHT(BOOL ena);
BOOL uv_valid();

void uv_clearLHT();
void uv_defaultParam();

//
void uv_setD(double dia);
double uv_getD();

void uv_setDAxis(BOOL a);
AXIS_T uv_getDAxis();

void uv_setDDir(DIR_T dir);
DIR_T uv_getDDir();

BOOL uv_enableD(BOOL ena);
BOOL uv_DValid();

//
double uv_X_to_motor(double X, double U);
double uv_U_to_motor(double X, double U);
double uv_Y_to_motor(double Y, double V);
double uv_V_to_motor(double Y, double V);

double uv_motor_to_Y(double Ym, double Vm);
double uv_motor_to_V(double Ym, double Vm);
double uv_motor_to_X(double Xm, double Um);
double uv_motor_to_U(double Xm, double Um);

fpoint_t uv_XY_to_motors(const fpoint_t* const XY, const fpoint_t* const UV);
fpoint_t uv_UV_to_motors(const fpoint_t* const XY, const fpoint_t* const UV);
fpoint_t uv_motors_to_XY(const fpoint_t* const XYm, const fpoint_t* const UVm);
fpoint_t uv_motors_to_UV(const fpoint_t* const XYm, const fpoint_t* const UVm);

point_t uv_limit(point_t pt);

void uv_tb();

#endif /* INC_UV_H_ */
