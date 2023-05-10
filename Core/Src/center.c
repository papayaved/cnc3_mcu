#include "center.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "touch.h"
#include "fpga.h"
#include "step_dir.h"
#include "cnc_param.h"
#include "feedback.h"
#include "defines.h"
#include "cnc_func.h"
#include "fpga_gpo.h"
#include "aux_func.h"

typedef enum {ST_IDLE, ST_HV, ST_TOUCH, ST_CENTER_4R, ST_CENTER_3A, ST_BACK, ST_WAIT} center_state_t;

static center_t center;
static center_state_t state, state_old;

static size_t step; // counters
static fpoint_t N[4] = {{0,0}, {0,0}, {0,0}, {0,0}}, Cn_acc = {0,0}; // accumulated variables
static int att_cnt; // counters

static float D = -1;
static double Dn_acc = 0;

int center_getState() { return (int)state; }
uint8_t center_attempt() { return (uint8_t)att_cnt; }
uint8_t center_attempts() { return center.attempts; }
float center_D() { return D; }

static void center_default() {
	center.mode		= CENTER_NO;
	center.touches	= 1;
	center.attempts	= 1;
	center.drum_vel	= 0;
	center.thld		= ADC_MAX;
	center.R		= 100; // mm
	center.rollback	= 0; // mm
	center.F		= mmm_to_mmclock(18);
	center.F_fine	= mmm_to_mmclock(1.8);
	center.fine_share	= 0;
	center.angle[0]	= M_PI * 1.0/ 4.0;
	center.angle[1]	= M_PI * 1.0/ 2.0;
	center.angle[2]	= M_PI * 3.0/ 4.0;
}

static void clear() {
	for (int i = 0; i < 4; i++)
		N[i].x = N[i].y = 0;

	Cn_acc.x = Cn_acc.y = 0;

	D = -1; // invalid
	Dn_acc = 0;
}

void center_reset() {
	touch_reset();
	center_default();
	state = state_old = ST_IDLE;
	center.touches = 0;
	clear();

	gpo_setCenterEnable(FALSE);
	gpo_apply();
}

// return 0 - OK
int center_init(const center_t* const p) {
	center_reset();

	memcpy(&center, p, sizeof(center_t));

	center.rollback = fabs(center.rollback);

	if (	(center.mode == CENTER_X || center.mode == CENTER_Y || center.mode == CENTER_CIRCLE_4R || center.mode == CENTER_CIRCLE_3ADJ) &&
			center.touches && center.touches < CENTER_ATTEMPTS_MAX &&
			center.attempts && center.attempts < CENTER_ATTEMPTS_MAX &&
			center.drum_vel >= 0 && center.drum_vel <= DRUM_VEL_MAX &&
			center.thld <= ADC_MAX &&
			center.fine_share >= 0.0 && center.fine_share <= 1.0 &&
			fabs(center.R) < CENTER_RADIUS_MAX && center.rollback < CENTER_RADIUS_MAX &&
			center.F >= F_MIN && center.F <= F_MAX &&
			center.F_fine >= F_MIN && center.F_fine <= F_MAX &&
			center.angle[0] >= 0 && center.angle[0] <= 2 * M_PI &&
			center.angle[1] >= 0 && center.angle[1] <= 2 * M_PI &&
			center.angle[2] >= 0 && center.angle[2] <= 2 * M_PI
	){
		if (center.R >= 0)
			center.rollback = -center.rollback;

#ifdef PRINT
		printf("CENTER Md=%d Tou=%d Att=%d DR=%d THLD=%d Z=%d R=%d RB=%d V=%d VF=%d\nA0=%d A1=%d A2=%d\n",
				(int)center.mode, (int)center.touches, (int)center.attempts, (int)center.drum_vel,
				(int)center.thld, (int)round(center.fine_share * 100.0),
				(int)(center.R * 1000), (int)(center.rollback * 1000),
				(int)round((mmclock_to_mmm(center.F) * (1000.0/60))), (int)round((mmclock_to_mmm(center.F_fine) * (1000.0/60))),
				(int)round(center.angle[0] * (360.0f / (2.0f * M_PI))),
				(int)round(center.angle[1] * (360.0f / (2.0f * M_PI))),
				(int)round(center.angle[2] * (360.0f / (2.0f * M_PI)))
			);
#endif

		return 0;
	}

	center_default();
	return -1;
}

static int center_error() {
	center_reset();
	return -1;
}

void cnc_reqG92_xyuv_enc(int32_t x, int32_t y, int32_t u, int32_t v, int32_t enc_x, int32_t enc_y);

// return: 0 - OK, 1 - finish, result < 0 - error
int center_next(float* const T) {
	static int nx, ny, res, ret; // temporary variables
	static double ts = 0; // temporary variable
	static fpoint_t Cn = {0,0};

	state_old = state;
	ret = 0;

	switch (state) {
	default:
		ret = center_error();
		break;

	case ST_IDLE:
		if (center.mode == CENTER_X || center.mode == CENTER_Y || center.mode == CENTER_CIRCLE_4R || center.mode == CENTER_CIRCLE_3ADJ) {
			gpo_setCenterEnable(TRUE);
			gpo_apply();

			cnc_reqG92_xyuv_enc(0, 0, 0, 0, 0, 0);
			Cn.x = Cn.y = 0;
			clear();
			step = 0;
			att_cnt = 1;

			fpga_setCenterMode(FPGA_CENTER_MODE_OFF);
			gpo_setDrumVel(center.drum_vel);
			gpo_setDrumEnable(center.drum_vel != 0);
			fpga_setCenterThld(center.thld);
#ifdef PRINT
			fpga_adcSnapshot();
			uint16_t adc = fpga_getADC(0);
			uint16_t thld = fpga_getCenterThld();
			printf("ADC:%x THLD:%x\n", adc, thld);

			int fb_ena = fpga_getFeedbackEnable();
			int soft_permit = fpga_getSoftPermit();
			printf("FB:%x PERM:%x\n", fb_ena, soft_permit);

			uint16_t mode = fpga_getCenterMode();
			int center_ena = mode >> 8;
			mode &= 0xff;
			printf("C:%x MD:%x\n", center_ena, (int)mode);

			uint16_t status = fpga_getStatus();
			printf("F:%x\n", (int)status);
#endif
			state = ST_HV;
		}
		else {
			ret = center_error();
			break;
		}

	case ST_HV: {
		fpoint_t A = {0,0};

		if (fpga_getCenterHvOk()) { // wait HV
			if (center.mode == CENTER_X) {
				A.x = center.R;
				state = ST_TOUCH;
			}
			else if (center.mode == CENTER_Y) {
				A.y = center.R;
				state = ST_TOUCH;
			}
			else if (center.mode == CENTER_CIRCLE_4R) {
				A.x = center.R;
				state = ST_CENTER_4R;
			}
			else {
				A = polar_cart(center.R, center.angle[0]);
				state = ST_CENTER_3A;
			}

			touch_init(A.x, A.y, center.touches, center.F, center.F_fine, center.fine_share);
			res = touch_next(T);

			if (res != 0)
				ret = center_error();
		}
		else
			ret = 0;
	}

		break;

	case ST_TOUCH:
		res = touch_next(T);

		if (res == 0) // OK, working on
			;
		else if (res == 1) { // finish, rollback
			nx = center.mode == CENTER_X ? mm_to_steps(center.rollback, cnc_scaleX()) : 0;
			ny = center.mode == CENTER_Y ? mm_to_steps(center.rollback, cnc_scaleY()) : 0;

			*T = 1.0f / center.F; // clocks / mm
			ts = fabs(center.rollback) * (*T); // clocks

			fpga_setCenterMode(FPGA_CENTER_MODE_OFF);

			if (nx || ny) {
				step_writeXYUV(0, ts, nx, ny, 0, 0);
				state = ST_WAIT;
#ifdef PRINT
				printf("RB x:%d y:%d\n", (int)nx, (int)ny);
#endif
			}
			else {
				state = ST_IDLE;
				center.touches = 0;
				center.attempts = 0;
				ret = 1;

				gpo_setCenterEnable(FALSE);
				gpo_apply();
			}
		}
		else
			ret = center_error();

		break;

	case ST_CENTER_4R:
		res = touch_next(T);

		if (res == 0) // OK, working on
			;
		else if (res == 1) { // finish, goto start position
			nx = touch_res_x();
			ny = touch_res_y();

			if (step < 4) {
				N[step].x += nx;
				N[step].y += ny;
			}

			switch (step) {
			case 0:
				nx = (int32_t)Cn.x - nx;
				ny = 0;

				if (nx || ny) {
					*T = 1.0f / center.F; // clocks / mm
					ts = steps_to_mm2(nx, ny, cnc_scaleXY()) * (*T); // clocks

					fpga_setCenterMode(FPGA_CENTER_MODE_OFF);
					step_writeXYUV(0, ts, nx, ny, 0, 0);
#ifdef PRINT
					printf("Back %d %d\n", nx, ny);
#endif
					state = ST_BACK;
					step++;
				}
				else
					ret = center_error();

				break;

			case 1: // goto center
				Cn.x = round( ( N[0].x + N[1].x ) / (2.0 * (double)att_cnt) );

				nx = (int32_t)Cn.x - nx;
				ny = 0;

				if (nx || ny) {
					*T = 1.0f / center.F; // clocks / mm
					ts = steps_to_mm2(nx, ny, cnc_scaleXY()) * (*T); // clocks

					fpga_setCenterMode(FPGA_CENTER_MODE_OFF);
					step_writeXYUV(0, ts, nx, ny, 0, 0);
#ifdef PRINT
					printf("Back %d %d\n", (int)nx, (int)ny);
#endif
					state = ST_BACK;
					step++;
				}
				else
					ret = center_error();

				break;

			case 2:
				nx = 0; // to center
				ny = (int)Cn.y - ny;

				if (nx || ny) {
					*T = 1.0f / center.F; // clocks / mm
					ts = steps_to_mm2(nx, ny, cnc_scaleXY()) * (*T); // clocks

					fpga_setCenterMode(FPGA_CENTER_MODE_OFF);
					step_writeXYUV(0, ts, nx, ny, 0, 0);
#ifdef PRINT
					printf("Back %d %d\n", nx, ny);
#endif
					state = ST_BACK;
					step++;
				}
				else
					ret = center_error();

				break;

			case 3: {
				static fpoint_t An[2] = {{0,0}, {0,0}}, Bn[2] = {{0,0}, {0,0}};
				static double Dn = 0;

				An[0].x = N[0].x / (double)att_cnt;
				An[0].y = N[0].y / (double)att_cnt;
				An[1].x = N[1].x / (double)att_cnt;
				An[1].y = N[1].y / (double)att_cnt;
				An[0].x = N[2].x / (double)att_cnt;
				An[0].y = N[2].y / (double)att_cnt;
				Bn[1].x = N[3].x / (double)att_cnt;
				Bn[1].y = N[3].y / (double)att_cnt;

				// Calculate center
//				static BOOL A_vert, B_vert;
//				static double b1, c1, b2, c2;
//
//				A_vert = A1.x > (A2.x - 0.001) && A1.x < (A2.x + 0.001);
//
//				if (!A_vert) {
//					b1 = (A2.y - A1.y) / (A2.x - A1.x);
//					c1 = -b1 * A1.x + A1.y;
//				}
//				else
//					b1 = c1 = 0;
//
//				B_vert = B1.x > (B2.x - 0.001) && B1.x < (B2.x + 0.001);
//
//				if (!B_vert) {
//					b2 = (B2.y - B1.y) / (B2.x - B1.x);
//					c2 = -b2 * B1.x + B1.y;
//				}
//				else
//					b2 = c2 = 0;
//
//				if (A_vert && B_vert)
//					return center_error();
//				else if (A_vert) {
//					Cn.x = (A1.x + A2.x) / 2.0;
//					Cn.y = b2 * Cn.x + c2;
//				}
//				else if (B_vert) {
//					Cn.x = (B1.x + B2.x) / 2.0;
//					Cn.y = b1 * Cn.x + c1;
//				}
//				else {
//					Cn.x = (c2 - c1) / (b1 - b2);
//					Cn.y = b1 * Cn.x + c1;
//				}

				Cn.x = (An[0].x + An[1].x) * 0.5;
				Cn.y = (Bn[0].y + Bn[1].y) * 0.5;

				Cn_acc.x += Cn.x;
				Cn_acc.y += Cn.y;

				Dn = (length_fpoint(&Cn, &An[0]) + length_fpoint(&Cn, &An[1]) + length_fpoint(&Cn, &Bn[0]) + length_fpoint(&Cn, &Bn[1])) * 0.5;
				Dn_acc += Dn;

				if (att_cnt < center.attempts) {
					att_cnt++;

					Cn.x = round(Cn.x);
					Cn.y = round(Cn.y);
					nx = (int)Cn.x - nx;
					ny = (int)Cn.y - nx;
					state = ST_BACK;
				}
				else {
					Cn.x = round(Cn_acc.x / (double)att_cnt);
					Cn.y = round(Cn_acc.y / (double)att_cnt);
					nx = (int)Cn.x - nx;
					ny = (int)Cn.y - ny;

					Dn = round(Dn_acc / (double)att_cnt);
					D = fsteps_to_mm(Dn, cnc_scaleX());

					state = ST_WAIT;
				}

				if (nx || ny) {
					step = 0;

					*T = 1.0f / center.F; // clocks / mm
					ts = steps_to_mm2(nx, ny, cnc_scaleXY()) * (*T); // clocks

					fpga_setCenterMode(FPGA_CENTER_MODE_OFF);
					step_writeXYUV(0, ts, nx, ny, 0, 0);
#ifdef PRINT
					printf("Back %d %d\n", (int)nx, (int)ny);
#endif
				}
				else
					ret = center_error();
			}
				break;

			default:
				ret = center_error();
				break;
			}
		}
		else
			ret = center_error();

	break;

	case ST_CENTER_3A:
		res = touch_next(T);

		if (res == 0) // OK, working on
			;
		else if (res == 1) { // finish, goto start position
			nx = touch_res_x();
			ny = touch_res_y();

			if (step < 3) {
				N[step].x += nx;
				N[step].y += ny;
			}

			switch (step) {
			case 0: case 1: // goto previous center
				nx = (int)Cn.x - nx;
				ny = (int)Cn.y - ny;

				if (nx || ny) {
					*T = 1.0f / center.F; // clocks / mm
					ts = steps_to_mm2(nx, ny, cnc_scaleXY()) * (*T); // clocks

					fpga_setCenterMode(FPGA_CENTER_MODE_OFF);
					step_writeXYUV(0, ts, nx, ny, 0, 0);
#ifdef PRINT
					printf("Back %d %d\n", nx, ny);
#endif
					state = ST_BACK;
					step++;
				}
				else
					ret = center_error();

				break;

			case 2: {
				static fpoint_t An[3] = {{0,0}, {0,0}, {0,0}};
				static double Dn = 0;

				An[0].x = N[0].x / (double)att_cnt;
				An[0].y = N[0].y / (double)att_cnt;
				An[1].x = N[1].x / (double)att_cnt;
				An[1].y = N[1].y / (double)att_cnt;
				An[2].x = N[2].x / (double)att_cnt;
				An[2].y = N[2].y / (double)att_cnt;

				Cn = circle_3pt_n(&An[0], &An[1], &An[2], &Dn);

				if (Dn > 0) {
					Dn_acc += Dn;

					Cn_acc.x += Cn.x;
					Cn_acc.y += Cn.y;

					if (att_cnt < center.attempts) {
						att_cnt++;

						Cn.x = round(Cn.x);
						Cn.y = round(Cn.y);

						nx = (int)Cn.x - nx;
						ny = (int)Cn.y - ny;
						state = ST_BACK;
					}
					else {
						Cn.x = round(Cn_acc.x / (double)att_cnt);
						Cn.y = round(Cn_acc.y / (double)att_cnt);
						nx = (int)Cn.x - nx;
						ny = (int)Cn.y - ny;

						Dn = round(Dn_acc / (double)att_cnt);
						D = fsteps_to_mm(Dn, cnc_scaleX());

						state = ST_WAIT;
					}

					if (nx || ny) {
						step = 0;

						*T = 1.0f / center.F; // clocks / mm
						ts = steps_to_mm2(nx, ny, cnc_scaleXY()) * (*T); // clocks

						fpga_setCenterMode(FPGA_CENTER_MODE_OFF);
						step_writeXYUV(0, ts, nx, ny, 0, 0);
#ifdef PRINT
						printf("Back %d %d\n", (int)nx, (int)ny);
#endif
					}
					else
						ret = center_error();
				}
				else
					ret = center_error();
			}
				break;

			default:
				ret = center_error();
				break;
			}
		}
		else
			ret = center_error();

	break;

	case ST_BACK:
		if (step_isStopped()) {
			switch (step) {
			case 0:
				if (center.mode == CENTER_CIRCLE_4R)
					touch_init(center.R, 0, center.touches, center.F, center.F_fine, center.fine_share);
				else {
					fpoint_t C = polar_cart(center.R, center.angle[0]);
					touch_init(C.x, C.y, center.touches, center.F, center.F_fine, center.fine_share);
				}
				break;
			case 1: // First
				if (center.mode == CENTER_CIRCLE_4R)
					touch_init(-center.R, 0, center.touches, center.F, center.F_fine, center.fine_share);
				else {
					fpoint_t A = polar_cart(center.R, center.angle[1]);
					touch_init(A.x, A.y, center.touches, center.F, center.F_fine, center.fine_share);
				}
				break;
			case 2:
				if (center.mode == CENTER_CIRCLE_4R)
					touch_init(0, center.R, center.touches, center.F, center.F_fine, center.fine_share);
				else {
					fpoint_t B = polar_cart(center.R, center.angle[2]);
					touch_init(B.x, B.y, center.touches, center.F, center.F_fine, center.fine_share);
				}
				break;
			case 3:
				if (center.mode == CENTER_CIRCLE_4R)
					touch_init(0, -center.R, center.touches, center.F, center.F_fine, center.fine_share);
				else
					ret = center_error();
				break;
			default:
				ret = center_error();
				break;
			}

			if (touch_next(T) == 0)
				state = center.mode == CENTER_CIRCLE_4R ? ST_CENTER_4R : ST_CENTER_3A;
			else
				ret = center_error();
		}

		break;

	case ST_WAIT:
		if (step_isStopped()) {
			state = ST_IDLE;
			center.touches = 0;
			center.attempts = 0;
			ret = 1;

			gpo_setCenterEnable(FALSE);
			gpo_apply();
		}
		break;
	}

#ifdef PRINT
	if (state != state_old) {
		switch (state) {
		case ST_IDLE:	printf("\tST_IDLE\n"); break;
		case ST_HV:		printf("\tST_HV\n"); break;
		case ST_TOUCH:	printf("\tST_TOUCH\n"); break;
		case ST_CENTER_4R:	printf("\tST_C4R\n"); break;
		case ST_CENTER_3A:	printf("\tST_C3A\n"); break;
		case ST_BACK:	printf("\tST_BACK\n"); break;
		case ST_WAIT:	printf("\tST_WAIT\n"); break;
		default:		printf("\tstate: %d\n", (int)state); break;
		}
	}
#endif

	return ret;
}

extern uint8_t cnc_ena;
BOOL cnc_isInit();
BOOL key_isInit();
void cnc_centerReq(const center_t* const pcenter);

void center_test() {
	static int cnt = 0;
	static center_t center;

	if (!cnt && cnc_ena && cnc_isInit() && key_isInit()) {
		cnt = 1;

		gpo_setCenterEnable(TRUE);
		gpo_apply();

		center.mode		= CENTER_X;
//		center.mode		= CENTER_CIRCLE;
		center.touches	= 2;
		center.attempts	= 1;
		center.drum_vel	= 0;
		center.thld		= 60; // ~10 V
		center.fine_share	= 0.5;
		center.R			= 2; // mm
		center.rollback	= 0; // mm
		center.F		= mmm_to_mmclock(18);
		center.F_fine	= mmm_to_mmclock(0.9);
		cnc_centerReq(&center);
	}
}
