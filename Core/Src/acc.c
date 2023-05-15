#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <ctype.h>
#include <math.h>

#include "acc.h"
#include "cnc_param.h"
#include "defines.h"

/*	The file provides functions for speed recalculation taking into account the permitted acceleration
 *
 */

BOOL m_acc_ena = FALSE;
float m_acc = 100 * COE_UMSEC2_TO_MMTICK2, m_dec = 100  * COE_UMSEC2_TO_MMTICK2; // 100 um/sec^2

void acc_reset() {
	m_acc_ena = FALSE;
	m_acc = m_dec = 100  * COE_UMSEC2_TO_MMTICK2;
}

void acc_enable(BOOL ena) { m_acc_ena = ena != 0; }
BOOL acc_enabled() { return m_acc_ena; }

static BOOL acc_check(float* const acc) {
	if (*acc >= ACC_MIN) {
		if (*acc > ACC_MAX)
			*acc = ACC_MAX;

		return TRUE;
	}

	return FALSE;
}

void acc_setAcc(float acc) {
	if (acc_check(&acc)) {
		m_acc = acc * COE_UMSEC2_TO_MMTICK2;
#ifdef PRINT
		printf("ACC=%d\n", (int)round(acc)); // print acceleration
#endif
	}
}

void acc_setDec(float dec) {
	if (acc_check(&dec)) {
		m_dec = dec * COE_UMSEC2_TO_MMTICK2;
#ifdef PRINT
		printf("DEC=%d\n", (int)round(dec)); // print acceleration
#endif
	}
}

// um / s^2
float acc_getAcc() { return m_acc * (1.0 / COE_UMSEC2_TO_MMTICK2); }
float acc_getDec() { return m_dec * (1.0 / COE_UMSEC2_TO_MMTICK2); }

/*	The function corrects the current speed according to a given acceleration
 * 	Return:
 * 		Corrected speed, ticks/mm
 * 	Parameters:
 * 		T - last speed, ticks/mm;
 * 		Tnom - required speed, ticks/mm;
 * 		dL - elementary step, mm.
 */
double acc_acc(double T, double Tnom, float dL) {
	if (m_acc_ena && T > Tnom) { // slower
		float F = 1 / T;
		float Fmax = 1 / Tnom;

		F = velocity(F, m_acc, dL);

		if (F < Fmax) // faster
			return 1 / F;
	}

	return Tnom;
}

/*	The function corrects the current speed if deceleration is needed
 * 	Return:
 * 		Brake required, boolean
 * 	Parameters:
 * 		*pT - current speed and new speed, ticks/mm;
 * 		rem - remained distance, mm;
 * 		dL - elementary step, mm.
 */
BOOL acc_brake(float T, float rem) {
//	return m_acc_ena && (T * T * rem * m_dec < 0.5f);
	return m_acc_ena && (T * T * rem * m_dec < 1.0f);
}

/*	The function corrects the current speed if deceleration is needed
 * 	Return:
 * 		Brake required, boolean
 * 	Parameters:
 * 		*pT - current speed and new speed, ticks/mm;
 * 		rem - remained distance, mm;
 * 		dL - elementary step, mm.
 */
double acc_dec(double T, float rem, float dL) {
	if (rem > dL) {
		float F = 1 / T;
		float k = 1 - dL / rem;

		float F1 = F * sqrt(k);
//		F = (F + F1) * 0.5;
		return 1 / F1;
	}

	return T;
}
