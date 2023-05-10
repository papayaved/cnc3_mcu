#include "cnc_param.h"

#include <math.h>

#include "my_types.h"
#include "fpga.h"

static double step = STEP; // motor step, mm
static scale_t scale_xy = {SCALE,		SCALE,		1.0 / SCALE,		1.0 / SCALE}; // steps / mm
static scale_t scale_uv = {SCALE_UV,	SCALE_UV,	1.0 / SCALE_UV,		1.0 / SCALE_UV}; // steps / mm
static scale_t scale_enc = {SCALE_ENC,	SCALE_ENC,	1.0 / SCALE_ENC,	1.0 / SCALE_ENC}; // steps / mm

static float acc = ACC, dec = DEC; // mm/tick^2/V

/* result - mean velocity, mm/clock
 * v0 - initial velocity, mm/clock
 * acc - acceleration, mm/clock^2
 * l - lenght, mm
 */
float velocity(float v0, float acc, float d) {
	static float v1_sq, v1;

	v1_sq = v0 * v0 + 2 * acc * d;
	v1 = v1_sq > 0 ? sqrt(v1_sq) : 0;
	return 0.5 * (v0 + v1);
}

void cnc_resetParam() {
	step = STEP;

	scale_xy.x = scale_xy.y = SCALE;
	scale_xy.x_inv = scale_xy.y_inv = 1.0 / SCALE;

	scale_uv.x = scale_uv.y = SCALE_UV;
	scale_uv.x_inv = scale_uv.y_inv = 1.0 / SCALE_UV;

	scale_enc.x = scale_enc.y = SCALE_ENC;
	scale_enc.x_inv = scale_enc.y_inv = 1.0 / SCALE_ENC;

	acc = ACC;
	dec = DEC;
}

BOOL cnc_isIdle();

// mm
void cnc_setStep(double value) {
	if (cnc_isIdle()) {
		step = value < STEP_MIN ? STEP_MIN : value > STEP_MAX ? STEP_MAX : value;
		step = round(step * 1e6) * 1e-6;
	}
}
// mm
double cnc_step() { return step; }

/* Acceleration and Deceleration in a feedback mode
 * um / s^2
 */
void cnc_setAcc(float value) { acc = value * COE_UMSEC2_TO_MMTICK2; }
void cnc_setDec(float value) { dec = value * COE_UMSEC2_TO_MMTICK2; }

// um / s^2
float cnc_getAcc() { return acc * (1.0 / COE_UMSEC2_TO_MMTICK2); }
float cnc_getDec() { return dec * (1.0 / COE_UMSEC2_TO_MMTICK2); }

// mm / clock^2
float cnc_acc() { return acc; }
float cnc_dec() { return dec; }

// mm / clock^2 / Vcode
float cnc_pidAcc() { return acc * COE_DCODE_TO_VOLT; }
float cnc_pidDec() { return dec * COE_DCODE_TO_VOLT; }

/* Scale of motors and linear encoders
 * steps / mm
 */
static double limitScale(double value) {
	if (value < SCALE_MIN)
		value = SCALE_MIN;
	else if  (value > SCALE_MAX)
		value = SCALE_MAX;
	else
		value = round(value);

	return value;
}

void cnc_setScaleX(double value) {
	if (cnc_isIdle()) {
		scale_xy.x = limitScale(value);
		scale_xy.x_inv  = 1 / scale_xy.x;
	}
}
void cnc_setScaleY(double value) {
	if (cnc_isIdle()) {
		scale_xy.y = limitScale(value);
		scale_xy.y_inv  = 1 / scale_xy.y;
	}
}
void cnc_setScaleU(double value) {
	if (cnc_isIdle()) {
		scale_uv.x = limitScale(value);
		scale_uv.x_inv = 1 / scale_uv.x;
	}
}
void cnc_setScaleV(double value) {
	if (cnc_isIdle()) {
		scale_uv.y = limitScale(value);
		scale_uv.y_inv = 1 / scale_uv.y;
	}
}
void cnc_setScaleEncX(double value) {
	if (cnc_isIdle()) {
		scale_enc.x = limitScale(value);
		scale_enc.x_inv = 1 / scale_enc.x;
	}
}
void cnc_setScaleEncY(double value) {
	if (cnc_isIdle()) {
		scale_enc.y = limitScale(value);
		scale_enc.y_inv = 1 / scale_enc.y;
	}
}

// steps / mm
const scale_t* cnc_scaleXY() { return &scale_xy; }
double cnc_scaleX() { return scale_xy.x; }
double cnc_scaleY() { return scale_xy.y; }
double cnc_getScale(AXIS_T axis) { return axis == AXIS_X ? scale_xy.x : (AXIS_Y ? scale_xy.y : 0); }

// steps / mm
const scale_t* cnc_scaleUV() { return &scale_uv; }
double cnc_scaleU() { return scale_uv.x; }
double cnc_scaleV() { return scale_uv.y; }

// steps / mm
const scale_t* cnc_scaleEnc() { return &scale_enc; }
double cnc_scaleEncX() { return scale_enc.x; }
double cnc_scaleEncY() { return scale_enc.y; }

// should be > 1
double cnc_ratioStepsEncX() { return scale_xy.x / scale_enc.x; }
double cnc_ratioStepsEncY() { return scale_xy.y / scale_enc.y; }

//
double cnc_steps2mmX(int32_t cnt) { return (double)cnt * scale_xy.x_inv; }
double cnc_steps2mmY(int32_t cnt) { return (double)cnt * scale_xy.y_inv; }

double cnc_steps2mmU(int32_t cnt) { return (double)cnt * scale_uv.x_inv; }
double cnc_steps2mmV(int32_t cnt) { return (double)cnt * scale_uv.y_inv; }

double cnc_enc2mmX(int32_t cnt) { return (double)cnt * scale_enc.x_inv; }
double cnc_enc2mmY(int32_t cnt) { return (double)cnt * scale_enc.y_inv; }

int32_t cnc_mm2StepsX(double value) { return (int32_t)round(value * scale_xy.x); }
int32_t cnc_mm2StepsY(double value) { return (int32_t)round(value * scale_xy.y); }

int32_t cnc_mm2StepsU(double value) { return (int32_t)round(value * scale_uv.x); }
int32_t cnc_mm2StepsV(double value) { return (int32_t)round(value * scale_uv.y); }
