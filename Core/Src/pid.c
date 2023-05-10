#include "pid.h"

#include <math.h>
#include <float.h>
#include <stdio.h>

#include "feedback.h"
#include "moving_avg.h"
#include "defines.h"
#include "cnc_param.h"
#include "cnc_func.h"

static BOOL wait_gap = TRUE, acc_req = TRUE, i_ena;
static float F0 = 0, F = 0; // mm / clock

// Reset speed regulator
void pid_clear() {
	wait_gap = acc_req = TRUE;
	i_ena = FALSE;
	F0 = F = 0;
	mavg_clear();

#ifdef PRINT
	printf("PID:clear\n");
#endif
}

// Stop on drum limit switches
void pid_stop() {
	if (i_ena) {
		wait_gap = acc_req = TRUE;
		F0 = F = 0;
#ifdef PRINT
	printf("PID:stop\n");
#endif
	}
	else {
#ifdef PRINT
	printf("PID:stop + ");
#endif
		pid_clear();
	}
}

void pid_reset() {
	pid_clear();
}

int mavg_n(); // it uses only for print

/*	The function for the feedback speed regulator
 *	Return:
 *		speed, ticks / mm
 *		negative value means no steps
 *	Parameter:
 *		adc - ADC code;
 *		Tnom - maximum speed, ticks / mm;
 *		dL - elementary step.
 */
float pid(uint16_t adc, float Tnom, float dL) {
	static float err = 0, dF = 0, Fp = 0, Fi = 0, Fmax = 0, Tp = 0, Ti = 0, T = 0;

	err = adc - fb_highThld();

	if (wait_gap) {
		if (err > 0)
			wait_gap = FALSE;
		else {
#ifdef PRINT
			printf("PID:wait\n");
#endif
			return -1;
		}
	}

	// Limits of error. Don't work at normal situations
	if (err > VNOM * COE_VOLT_TO_DCODE)
		err = VNOM * COE_VOLT_TO_DCODE;
	else if (err < -VNOM * COE_VOLT_TO_DCODE)
		err = -VNOM * COE_VOLT_TO_DCODE;

	Fmax = 1 / Tnom;

	if (err < 0) {
		acc_req = FALSE;
		dF = cnc_pidDec() * err;
	}
	else {
		if (acc_req)
			dF = cnc_pidAcc() * (VNOM * COE_VOLT_TO_DCODE); // err = 100 * (1.638 / 100) / (4.096 / 1<<10) = 410
		else
			dF = cnc_pidAcc() * err;
	}

	Fp = velocity(F0, dF, dL);

	// Almost shortcut or needed acceleration. Don't work
	if (i_ena && (Fp < F_MIN || Fp > F_MAX)) {
		i_ena = FALSE;
		mavg_clear();

		Fp = velocity(F, dF, dL); // Recalculate
	}

	BOOL overspeed = Fp > Fmax;
	if (overspeed)
		Fp = Fmax;

	Fp = speed_range(Fp);
	Tp = 1 / Fp;

	Ti = mavg_get();

	if (i_ena && acc_req) {
		if (Tp < Ti) { // Ti is correct here
			acc_req = FALSE; // End of acceleration
			T = Ti;
			F = 1 / T;
		}
		else {
			F = Fp;
			T = Tp;
		}
	}
	else {
		if (i_ena || (mavg_ready( MS_TO_TICK(5000) ) && Ti < Tp)) { // The speed has reached maximum and become smaller than average
			i_ena = TRUE;
			Fi = 1 / Ti;
			F = (Fi + Fp) / 2;

			overspeed = F > Fmax;
			if (overspeed)
				F = Fmax;

			F = speed_range(F);
			T = 1 / F;
		}
		else {
			F = Fp;
			T = Tp;
		}
	}

	if (overspeed)
		acc_req = FALSE;

	if (!acc_req)
		mavg_add(T); // or Tp

	F0 = 2 * Fp - F0; // F0 = F1

#ifdef PRINT
	float FF = (FPGA_CLOCK * 1000.0) * F;
	float FFp = (FPGA_CLOCK * 1000.0) * Fp;
	float FFi = (FPGA_CLOCK * 1000.0) * 1 / Ti;

	printf("E:%d a:%c i:%c F:%d Fp:%d Fi:%d n:%d\n", (int)err, acc_req ? 'T' : 'F', i_ena ? 'T' : 'F', (int)FF, (int)FFp, (int)FFi, mavg_n());
#endif

	return T;
}
