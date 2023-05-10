#include <stdio.h>

#include "stm32h7xx_hal.h"

#include "feedback.h"
#include "defines.h"
#include "fpga.h"
#include "cnc_func.h"
#include "math.h"
#include "cnc_task.h"
#include "moving_avg.h"

static BOOL fb_ena = FALSE;
static uint16_t low_thld = 0, high_thld = ADC_MAX;

//static uint16_t adc_nom = 100.0 / COE_DIFF(DIFF_GAIN, ADC_VREF, ADC_WIDTH);
static double fb_nom = FB_NOM; // V
static uint16_t adc_nom = 0;

static double Trb = T_ROLLBACK_DEFAULT_TICKS;

//static BOOL T_avg_valid = FALSE;
//static float Fi = 0;

static double T = T_MAX_TICK;
static int T_valid = 0;

//static uint32_t tic, toc;

//static const double coe[ADC_NUM] = {
//	COE_DIFF(K_DIFF, ADC_VREF, ADC_WIDTH),
//	COE_DIFF(1.0, ADC_VREF, ADC_WIDTH), // unused
//	COE_DIFF(1.0 / 111.08, 4.096, 10), // back
//	COE_DIFF(1.0, 4.096, 10), // unused
//	COE_DIV(1e6, 20e3, 1, 4.096, 10), // workpeice
//	COE_DIV(1e6, 20e3, 1, 4.096, 10), // wire
//	COE_DIV(1e6, 20e3, 1, 4.096, 10), // hv
//	COE_DIFF(10, 4.096, 10) // shunt
//};

void fb_reset() {
	fb_ena = FALSE;
	fpga_setFeedbackEnable(FALSE);

	fb_setThld(0, ADC_MAX);

	adc_nom = volt_to_code(fb_nom);
	Trb = T_ROLLBACK_DEFAULT_TICKS;

//	T_avg_valid = FALSE;
	T = T_MAX_TICK;
	T_valid = FALSE;
}

// Feedback
//void fb_storeEnable() {
//	fb_ena_reg = fb_ena;
//}

void fb_enableRestore() {
	fpga_setFeedbackEnable(fb_ena);
}

void fb_enableForce(BOOL value) {
	fpga_setFeedbackEnable(value);
}
void fb_enable(BOOL value) {
	fb_ena = value != 0;
	fpga_setFeedbackEnable(fb_ena);
}

BOOL fb_isEnabledForce() {
	return fpga_getFeedbackEnable();
}
BOOL fb_isEnabled() {
	return fb_ena;
}

void fb_setThld(uint16_t low, uint16_t high) {
	if (high > ADC_MAX)
		high = ADC_MAX;

	if (low > high)
		low = high;

	low_thld = low;
	high_thld = high;

	fpga_setLowThld(low_thld);
	fpga_setHighThld(high_thld);
}

void fb_setThld_Volt(double low, double high) {
	fb_setThld( volt_to_code(low), volt_to_code(high) );
}

void fb_setLowThld(uint16_t low) {
	fb_setThld(low, high_thld);
}

//void fb_setPidThld(uint16_t high, uint16_t middle) {
//	middle_thld = middle > ADC_MAX ? ADC_MAX : middle;
//	high_thld = high > ADC_MAX ? ADC_MAX : high;
//
//	if (middle_thld > high_thld)
//		middle_thld = high_thld;
//
//	if (low_thld > middle_thld)
//		low_thld = middle_thld;
//
//	fb_setThld(high_thld, middle_thld, low_thld);
//
//	printf("THDL L:%04x(%04x) M:%04x H:%04x(%04x)\n", (int)low_thld, (int)fb_getLowThld(), (int)middle_thld, (int)high_thld, (int)fb_getHighThld());
//}

uint16_t fb_lowThld() {
	return low_thld = fpga_getLowThld();
}
uint16_t fb_highThld() {
	return high_thld = fpga_getHighThld();
}

void fb_setRollbackSpeed(float F) {
	Trb = speed_to_period(F);
}

float fb_getRollbackSpeed() {
	return period_to_speed(Trb);
}

double fb_Trb() {
	return Trb;
}

#define TI_LEN (1<<8)

//void fb_speed_task() {
//	static float F_cur = 0, F_acc = 0, F_acc_reg = 0;
//	static unsigned avg_cnt;
//
//	toc = HAL_GetTick();
//
//	if (toc - tic >= FB_PERIOD) {
//		tic += FB_PERIOD;
//
//		if (fb_ena && cnc_isNormal() && cnc_run() && !cnc_stop() && !cnc_pause()) {
//			if (T_avg_valid) {
//				if (fpga_getFlags() & FLAG_NO_PERMIT_MSK)
//					F_cur = 0;
//				else
//					F_cur = 1.0f / cnc_getCurrentT();
//
//				F_acc += F_cur;
//
//				if (avg_cnt < TI_LEN)
//					avg_cnt++;
//				else {
//					avg_cnt = 0;
//					F_acc -= F_acc_reg;
//					F_acc_reg = F_acc;
//					Fi = F_acc_reg / TI_LEN;
//				}
//			}
//			else {
//				T_avg_valid = TRUE;
//				avg_cnt = 0;
//				F_acc_reg = F_acc = TI_LEN / T_MAX_TICK;
//			}
//		}
//	}
//}

//float fb_getAvarageF() { return Fi; }

// use function if FLAG_NO_PERMIT_MSK is set
//void fb_clearAvarageSpeed() {
//	mavg_clear();
//	T_valid = FALSE;
//}

//double fb_T(double Tnom, uint16_t adc) {
//	static int cnt = 0;
//	static float F = 0;
//
//	if (cnt++ < 1) {
//		printf("THDL %x-%x\n", (int)low_thld, (int)high_thld);
//		printf("ADC %x(%x)\n", (int)adc, (int)adc_nom);
//	}
//
//	// Integral part
//	mavg_add(F);
//	Fi = mavg_get();
//
//	float Fp;
//
//	if (adc >= adc_nom)
//		Fp = 1.0f / Tnom;
//	else if (adc_nom <= high_thld) // error
//		Fp = 1.0f / Tnom; // ?
//	else if (adc > ADC_MAX) // error
//		return Tnom;
//	else if (adc <= high_thld) {
////		return Trb > Tnom ? Trb : Tnom;
//		F = 0;
//		return T_MAX_TICK; // 1 us/sec
//	}
//	else { // Proportional part
//		float Tp = range( Tnom * (adc_nom - high_thld) / (adc - high_thld) ); // k = (100 - 60) / (80 - 60) - period only increases
//		Fp = 1.0f / Tp;
//	}
//
//	const float Ki = 0.5, Kp = 0.5;
//	F = Ki * Fi + Kp * Fp;
//
//	T = range(1.0f / F);
//	F = 1.0f / T;
//
//	if (cnt++ < 2) {
//		printf("T %d(%d) %d%%\n", (int)round(T * 1e-6), (int)round(Tnom * 1e-6), (int)round(T * 100 / Tnom));
//	}
//
//	T_valid = 1;
//	return T;
//}

//double fb_lastT(double Tnom) {
//	return T_valid ? T : Tnom;
//}

// ADC
// return: 1 bit - 10mV
//uint16_t fb_getADC10mv(uint8_t i) {
//	if (i < ADC_NUM) {
//		uint16_t adc = fpga_getADC(i);
//
//		if (adc <= ADC_MAX) {
//			double data = round(coe[i] * adc * 100);
//
//			if (data < UINT16_MAX)
//				return (uint16_t)data;
//		}
//	}
//
//	return UINT16_MAX;
//}

// msec
void fb_setRollbackTimeout(uint32_t value) {
	uint32_t timeout = fpga_MsToPauseTicks(value);

	fpga_setTimeoutEnable(FALSE);

	if (timeout && timeout <= TIMEOUT_MAX) {
		fpga_setTimeout(timeout);
		fpga_setTimeoutEnable(TRUE);
	}
	else
		fpga_setTimeout(TIMEOUT_MAX);
}

// msec
uint32_t fb_getRollbackTimeout() {
	BOOL enable = fpga_getTimeoutEnable();

	if (enable)
		return (uint32_t)round( fpga_pauseTicksToMs( fpga_getTimeout() ) );
	else
		return UINT32_MAX;
}

double code_to_volt(uint16_t code) {
	return (code * ADC_LSB + V_DIOD) * (1.0 / K_DIFF);
}

uint16_t volt_to_code(double volt) {
	double code = (volt * K_DIFF - V_DIOD) * (1.0 / ADC_LSB);

    code = round(code);

	if (code < 0)
		return 0;
	else if (code > ADC_MAX)
		return ADC_MAX;

    return (uint16_t)code;
}
// (80V * 1.638/100 - 0.624) * (1<<10) / 4.096 = 171.6
// (70V * 1.638/100 - 0.624) * (1<<10) / 4.096 = 130.65
// 80 - 70 = 40.95
