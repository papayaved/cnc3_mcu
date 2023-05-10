#include <stdio.h>

#include "defines.h"
#include "fpga.h"
#include "fpga_gpo.h"
#include "my_wdt.h"
#include "sem_led.h"
#include "enc_recalc_pos.h"

BOOL cnc_isLocked();
BOOL cnc_drumPermit();

const uint8_t current_codes[CURRENT_CODES] = {0, 1, 2, 3, 6, 7, 0xE, 0xF, 0x1E, 0x1F, 0x3E, 0x3F};
static drum_state_t drum_state_reg = DRUM_DIS;

size_t seek_index(const uint8_t* array, size_t size, uint8_t value) {
	size_t i;
	for (i = 0; i < size; i++)
		if (array[i] == value)
			break;

	return i;
}

size_t current_index;

//#define DATA48 ( (controls_t)(0ULL<<42 | 0ULL<<41 | 0ULL<<40 | 36ULL<<32 | 8<<24 | 7<<16 | 7<<2 | 0<<0) )
fpga_controls_t dato32, dato32_reg;

//#define DATA16 ( (indicator_t)((uint16_t)0xFF00) )
fpga_ind_t dato16;
static BOOL dato32_valid, dato16_valid;

// work automatic in FPGA
static void updateIndicator() {
	dato16.field.ena = 1;
	dato16.field.drum_ena = getDrumEnable(dato32.data) ? 1 : 0;
	dato16.field.pump_ena = dato32.field.pump_ena ? 1 : 0;

	unsigned vel = (1 << dato32.field.drum_vel) - 1;
	dato16.field.drum_vel = vel;

	dato16_valid = FALSE;
}

void gpo_updateIndicator() {
	updateIndicator();
	fpga_setPult(dato16.data);
	dato16_valid = TRUE;
}

void gpo_store() { dato32_reg = dato32; }

void gpo_restore() {
	dato32 = dato32_reg;
	dato32_valid = FALSE;
	updateIndicator();
}

#ifndef STONE
static BOOL idle() { return !dato32.field.voltage_ena; }
#endif

//void gpo_setControls(uint64_t value) {
//	gpo_setDrumState( getDrumState(value) );
//	gpo_setDrumVel( getDrumVel(value) );
//	gpo_setCurrentIndex( getCurrentIndex(value) );
//	gpo_setPulseRatio( getPulseRatio(value) );
//	gpo_setPulseWidth( getPulseWidth(value) );
//	gpo_setVoltageLevel( getVoltageLevel(value) );
//	gpo_setVoltageEnable( getVoltageEnable(value) );
//	gpo_setPumpEnable( getPumpEnable(value) );
//
//	updateIndicator();
//}

static BOOL cnc_enable; // the flag that is set by computer

//void gpo_cncEnable(BOOL value) {
//	cnc_enable = value;
//}

BOOL gpo_cncEnabled() {
#if defined(RESEASE)
	return cnc_enable;
#else
	return TRUE;
#endif
}

void cnc_enableUV(BOOL ena);

void gpo_setControlsEnable(uint16_t ena, uint16_t mask) {
	 if (mask & 1<<0)
		 gpo_setPumpEnableKey((ena & 1) != 0);
	 if (mask & 3<<1)
		 gpo_setDrumEnableKey((ena & 3<<1) != 0);
	 if (mask & 8)
		 gpo_setWireControlEnable((ena & 8) != 0);
	 if (mask & 0x10)
		 gpo_setVoltageEnableKey((ena & 0x10) != 0);
	 if (mask & 0x20)
		 fpga_setMotorOE((ena & 0x20) != 0);
	 if (mask & 0x40)
		 sem_enable((ena & 0x40) != 0);
	 if (mask & 0x80) {
		 cnc_enable = (ena & 0x80) != 0;
		 gpo_setWireControlEnable(cnc_enable);
		 gpo_updateIndicator();
	 }
	 if (mask & 0x100)
		 gpo_setCenterEnable((ena & 0x100) != 0);
}

void gpo_setDrumState(drum_state_t state) {
	if (state == DRUM_ANY) {
		gpo_setDrumEnable(TRUE);
		return;
	}
	else if (state > DRUM_ANY)
		state = DRUM_DIS;

	BOOL ena = getDrumEnable(dato32.data);

	if (ena && state == DRUM_DIS)
		gpo_setDrumStateReg(dato32.field.drum_state);

	dato32.field.drum_state = state;
	dato32_valid = FALSE;
	updateIndicator();
}

void gpo_setDrumStateReg(drum_state_t state) {
	if (state > DRUM_REV)
		drum_state_reg = DRUM_DIS;
	else
		drum_state_reg = state;
}

void gpo_setDrumStateIf(drum_state_t state) {
	if (getDrumEnable(dato32.data))
		dato32.field.drum_state != state ? gpo_setDrumState(state) : gpo_setDrumState(DRUM_DIS); // ??
	else
		gpo_setDrumState(DRUM_DIS);
}

// return 1 - stop request
int gpo_setDrumEnable(BOOL ena) {
	drum_state_t state = getDrumState(dato32.data);

	if (ena) {
		if (state == DRUM_FWD || state == DRUM_REV)
			return 0;

		uint16_t keys = fpga_getInput();
#ifdef PRINT
		printf("KEYS:%x DRUM:%x\n", (int)keys, (int)state);
#endif

		if ( (keys & (SIGI_REV_MSK | SIGI_FWD_MSK)) == (SIGI_REV_MSK | SIGI_FWD_MSK) || dato32.field.drum_state != DRUM_DIS ) // limit switch error, state error
			gpo_setDrumState(DRUM_DIS);
		else if (keys & SIGI_FWD_MSK)
			gpo_setDrumState(DRUM_REV);
		else if (keys & SIGI_REV_MSK)
			gpo_setDrumState(DRUM_FWD);
		else { // no limit switch
			if (drum_state_reg == DRUM_FWD || drum_state_reg == DRUM_REV)
				gpo_setDrumState(drum_state_reg); // restore
			else
				gpo_setDrumState(DRUM_FWD);
		}

		drum_state_reg = DRUM_DIS;
	}
	else { // disable and save state
		if (state == DRUM_DIS)
			return 0;

		if (dato32.field.drum_state == DRUM_FWD || dato32.field.drum_state == DRUM_REV) {
			drum_state_reg = dato32.field.drum_state;
		}
		gpo_setDrumState(DRUM_DIS);

		return 1;
	}

	return 0;
}

void gpo_setDrumEnableKey(BOOL ena) {
	if (gpo_setDrumEnable(ena))
		fpga_setSoftAlarm();
}


void gpo_switchDrumKey() {
	if (dato32.field.drum_state == DRUM_FWD || dato32.field.drum_state == DRUM_REV)
		gpo_setDrumEnableKey(FALSE);
	else if (cnc_drumPermit())
		gpo_setDrumEnableKey(TRUE);
}

void gpo_switchDrumDir() {
	drum_state_t state = dato32.field.drum_state;

	if (state == DRUM_FWD)
		gpo_setDrumState(DRUM_REV);
	else if (state == DRUM_REV)
		gpo_setDrumState(DRUM_FWD);
	else
		gpo_setDrumState(DRUM_DIS);
}

void gpo_setDrumForward() { gpo_setDrumState(DRUM_FWD); }
void gpo_setDrumReverse() { gpo_setDrumState(DRUM_REV); }

drum_state_t gpo_getDrumState() { return dato32.field.drum_state; }
BOOL gpo_getDrumEnable() { return dato32.field.drum_state == DRUM_FWD || dato32.field.drum_state == DRUM_REV; }

void gpo_setDrumVel(uint8_t code) {
	if (code < 1)
		code = 1;
	else if (code > DRUM_VEL_MAX)
		code = DRUM_VEL_MAX;

	dato32_reg.field.drum_vel = dato32.field.drum_vel = code; // velocity can be changed on limit switches
	dato32_valid = FALSE;
	updateIndicator();
}

uint8_t gpo_getDrumVel() {
	return dato32.field.drum_vel;
}

void gpo_drumVelInc() {
	uint8_t vel = dato32.field.drum_vel;

	if (vel < DRUM_VEL_MAX)
		gpo_setDrumVel(++vel);
	else
		gpo_setDrumVel(DRUM_VEL_MAX);
}

void gpo_drumVelDec() {
	uint8_t vel = dato32.field.drum_vel;

	if (vel > 1)
		gpo_setDrumVel(--vel);
	else
		gpo_setDrumVel(1);
}

void gpo_setCurrentIndex(size_t index) {
	current_index = index < CURRENT_CODES ?  index : CURRENT_CODES - 1;
	dato32_reg.field.current_code = dato32.field.current_code = current_codes[current_index];
	dato32_valid = FALSE;
}

void gpo_setPulseWidth(uint8_t value) {
	if (value < 2)
		value = 2;
	else if (value > 99)
		value = 99;

	dato32_reg.field.pulse_width = dato32.field.pulse_width = value;
	dato32_valid = FALSE;
}

void gpo_setPulseRatio(uint8_t value) {
	if (value < 2)
		value = 2;
	else if (value > 32)
		value = 32;

	dato32_reg.field.pulse_ratio = dato32.field.pulse_ratio = value;
	dato32_valid = FALSE;
}

void gpo_setVoltageLevel(uint8_t code) {
	if (code > 1)
		code = 1;

	dato32_reg.field.voltage_level = dato32.field.voltage_level = code;
	dato32_valid = FALSE;
}

void gpo_disableVDP() {
	dato32_reg.field.voltage_ena = dato32.field.voltage_ena = 0;
	dato32_reg.field.drum_state = dato32.field.drum_state = DRUM_DIS;
	dato32_reg.field.pump_ena = dato32.field.pump_ena = 0;
	dato32_valid = FALSE;
}

// return 1 - stop request
int gpo_setVoltageEnable(BOOL value) {
	value = value ? TRUE : FALSE;
	BOOL ena = dato32.field.voltage_ena ? TRUE : FALSE;

	if (value == ena)
		return 0;

	if (value) {
		if (getDrumEnable(dato32.data) && dato32.field.pump_ena) {
			dato32_reg.field.voltage_ena = dato32.field.voltage_ena = 1;
			dato32_valid = FALSE;
			updateIndicator();
		}
	}
	else if (ena) { // Disable
		dato32_reg.field.voltage_ena = dato32.field.voltage_ena = 0;
		gpo_setDrumEnable(FALSE);
		dato32_reg.field.pump_ena = dato32.field.pump_ena = 0;
		dato32_valid = FALSE;
		updateIndicator();

		return 1;
	}

	return 0;
}

void gpo_setVoltageEnableKey(BOOL value) {
	if (gpo_setVoltageEnable(value))
		fpga_setSoftAlarm();
}

// return 1 - stop request
int gpo_setPumpEnable(BOOL value) {
	value = value ? TRUE : FALSE;
	BOOL ena = dato32.field.pump_ena ? TRUE : FALSE;

	if (ena == value)
		return 0;

	dato32_reg.field.pump_ena = dato32.field.pump_ena = value;
	dato32_valid = FALSE;
	updateIndicator();

#ifdef STONE
	return ena && !value;
#else
	return ena && !value && !idle();
#endif
}

void gpo_setPumpEnableKey(BOOL value) {
	if (gpo_setPumpEnable(value))
		fpga_setSoftAlarm();
}

void gpo_switchPumpKey() {
	if (!cnc_isLocked())
		dato32.field.pump_ena ? gpo_setPumpEnableKey(FALSE) : gpo_setPumpEnableKey(TRUE);
	else if (dato32.field.pump_ena)
		gpo_setPumpEnableKey(FALSE);
}

void gpo_setCenterEnable(BOOL value) {
	value = value ? TRUE : FALSE;
	dato32_reg.field.center_ena = dato32.field.center_ena = value;
	dato32_valid = FALSE;
}

void gpo_readControls() {
	dato32 = (fpga_controls_t)fpga_getControls();
}

void gpo_apply() {
	/*
	{
		uint16_t keys = fpga_getKeys();
		uint16_t clicked = fpga_getKeysClicked();
		uint16_t limsw = fpga_getLimSw();
		uint16_t limsw_reg = fpga_getLimSwReg();

		uint16_t ind = fpga_getIndicator();
		uint64_t ctrl = fpga_getControls();

		uint16_t flags = fpga_getFlags();

		printf("KEY:%05x(CLK:%05x) LS:%02x(REG:%02x) F:%x CTRL:%04x%08x IND:%04x\n",\
				(unsigned)keys, (unsigned)clicked, (unsigned)limsw, (unsigned)limsw_reg, (unsigned)flags, (unsigned)(ctrl>>32), (unsigned)ctrl, (unsigned)ind);
	}

	printf("Apply CTRL:%04x%08x IND:%04x\n",\
			(unsigned)(data48.data>>32), (unsigned)data48.data, (unsigned)data16.data);
			*/

	fpga_setControls(dato32.data);
	fpga_setPult(dato16.data);
	dato32_valid = dato16_valid = TRUE;

	/*
	{
		uint16_t keys = fpga_getKeys();
		uint16_t clicked = fpga_getKeysClicked();
		uint16_t limsw = fpga_getLimSw();
		uint16_t limsw_reg = fpga_getLimSwReg();

		uint16_t ind = fpga_getIndicator();
		uint64_t ctrl = fpga_getControls();

		uint16_t flags = fpga_getFlags();

		printf("KEY:%05x(CLK:%05x) LS:%02x(REG:%02x) F:%x CTRL:%04x%08x IND:%04x\n",\
				(unsigned)keys, (unsigned)clicked, (unsigned)limsw, (unsigned)limsw_reg, (unsigned)flags, (unsigned)(ctrl>>32), (unsigned)ctrl, (unsigned)ind);
	}
	*/
}

BOOL gpo_getValid() { return dato32_valid && dato16_valid; }

drum_state_t getDrumState(uint32_t value) {
	const fpga_controls_t v = (fpga_controls_t)value;
	drum_state_t state = v.field.drum_state;

	if (state > DRUM_ANY)
		state = DRUM_DIS;

	return state;
}

BOOL getDrumEnable(uint32_t value) {
	drum_state_t state = getDrumState(value);
	return state == DRUM_FWD || state == DRUM_REV;
}

uint8_t getDrumVel(uint32_t value) {
	const fpga_controls_t v = (fpga_controls_t)value;
	return v.field.drum_vel;
}

size_t getCurrentIndex(uint32_t value) {
	const fpga_controls_t v = (fpga_controls_t)value;
	uint8_t code = v.field.current_code;
	size_t index = seek_index(current_codes, CURRENT_CODES, code);

	if (index >= CURRENT_CODES)
		index = CURRENT_CODES - 1;

	return index;
}

uint8_t getPulseRatio(uint32_t value) {
	const fpga_controls_t v = (fpga_controls_t)value;
	uint8_t ratio = v.field.pulse_ratio;

	if (ratio < 2)
		ratio = 2;
	else if (ratio > 32)
		ratio = 32;

	return ratio;
}

uint8_t getPulseWidth(uint32_t value) {
	const fpga_controls_t v = (fpga_controls_t)value;
	uint8_t width = v.field.pulse_width;

	if (width < 2)
		width = 2;
	else if (width > 99)
		width = 99;

	return width;
}

uint8_t getVoltageLevel(uint32_t value) {
	const fpga_controls_t v = (fpga_controls_t)value;
	return v.field.voltage_level;
}
BOOL getVoltageEnable(uint32_t value) {
	const fpga_controls_t v = (fpga_controls_t)value;
	return v.field.voltage_ena != 0;
}
BOOL getPumpEnable(uint32_t value) {
	const fpga_controls_t v = (fpga_controls_t)value;
	return v.field.pump_ena != 0;
}

BOOL gpo_getCncEnable() {
	BOOL ena = dato32_valid && getDrumEnable(dato32.data) && dato32.field.pump_ena && dato32.field.current_code != 0 && dato32.field.voltage_ena;
#ifdef STONE
	return ena;
#else
	return ena || idle();
#endif
}

BOOL gpo_isRun() {
	return dato32.field.voltage_ena || dato32.field.pump_ena || dato32.field.drum_state == DRUM_FWD || dato32.field.drum_state == DRUM_REV;
}

// LIM SW MASK
BOOL getWireControlEnable(uint16_t mask) {
	const fpga_lim_switch_t m = (fpga_lim_switch_t)mask;
	return m.field.wire_ctrl;
}

static void updateLimSwMask(uint16_t mask) {
	const fpga_lim_switch_t m = (fpga_lim_switch_t)mask;
	dato16.field.wire_ctrl = m.field.wire_ctrl ? 1 : 0;
	dato16_valid = FALSE;
}

void gpo_switchWireControl() {
	fpga_lim_switch_t mask = (fpga_lim_switch_t)fpga_getLimSwMask();
	mask.field.wire_ctrl ^= 1;
	fpga_setLimSwMask(mask.data);
	updateLimSwMask(mask.data);
}

void gpo_setWireControlEnable(BOOL ena) {
	fpga_lim_switch_t mask = (fpga_lim_switch_t)fpga_getLimSwMask();
	mask.field.wire_ctrl = ena ? 1 : 0;
	fpga_setLimSwMask(mask.data);
	updateLimSwMask(mask.data);
}

//void gpo_setLimSwMask(uint16_t set, uint16_t clear) {
//	uint16_t mask = fpga_getLimSwMask();
//	mask |= set;
//	mask &= ~clear;
//	fpga_setLimSwMask(mask);
//	updateLimSwMask(mask);
//}

//
//void gpo_setSoftAlarm() {
//	fpga_setSoftAlarm();
//	dato32_reg = dato32;
//	// todo: clear alarm mask
//}
//void gpo_clearSoftAlarm() {
//	fpga_clearSoftAlarm();
//	dato32 = dato32_reg; // ??? i have func
//}

//
void fpga_init() {
	fpga_reset();

	dato16.field.ena = 1;

	gpo_setDrumState(DRUM_DIS);
	drum_state_reg = DRUM_DIS;

	gpo_setDrumVel(2);
	gpo_setCurrentIndex(5);
	gpo_setPulseRatio(7);
	gpo_setPulseWidth(35);
	gpo_setVoltageLevel(0);
	gpo_setVoltageEnable(FALSE);
	gpo_setPumpEnable(FALSE);

	gpo_setWireControlEnable(TRUE);
	gpo_apply();

	// Clear
//	fpga_write_u16(KEYS_UP>>1, 0xFFFF);
	fpga_write_u16(LIMSW_FLAG>>1, 0xFFFF);

	// ADC
	fpga_setFeedbackEnable(FALSE); // ADC disabled

//	uint32_t timeout = fpga_MsToPauseTicks(30e3);
//	printf("TO %d\n", (int)timeout); // 0.1 ms
//	fpga_setTimeout( timeout ); // ms todo:uncomment

	fpga_clearTimeout();
	fpga_setSoftPermit(TRUE); // global run permit

	cnc_enable = FALSE;
	sem_reset();
	soft_wdt_reset();
}

//
#include "stm32h7xx_hal.h"
void cnc_direct_test_task();

void ind_test_task() {
	static const uint32_t PERIOD = 500;

	static uint8_t cnt;
	static uint32_t tic, toc;

	static BOOL init = FALSE;
	static uint16_t ind = 1, sig_out = 1;

	if (!init) {
		fpga_testEnable(TRUE);
		BOOL ena = fpga_testEnabled();
		printf("TEST ENA: %x\n", (int)ena);
		init = TRUE;
	}

	toc = HAL_GetTick();

	if (toc - tic >= PERIOD) {
		cnt++;
		tic += PERIOD;

		// TEST
		fpga_setPult(ind);
		uint16_t ind_reg = fpga_getPult();
		printf("IND: %x\n", (int)ind_reg);

		ind <<= 1;

		if (ind == 0)
			ind = 1;

		//
		fpga_setOutput(sig_out);
		uint16_t sigo_reg = fpga_getOutput();
		printf("SIGO: %x\n", (int)sigo_reg);

		sig_out <<= 1;

		if (sig_out == 0)
			sig_out = 1;

		cnc_direct_test_task();
	}
}

void pult_test_task() {
	static const uint32_t PERIOD = 2000;
	static uint8_t cnt;
	static uint32_t tic, toc;

	static int i = 0;

	toc = HAL_GetTick();

	if (toc - tic >= PERIOD) {
		cnt++;
		tic += PERIOD;

		// TEST
		switch (i) {
		case 0: gpo_setDrumVel(1); break;
		case 1: gpo_setDrumVel(2); break;
		case 2: gpo_setDrumVel(3); break;
		case 3: gpo_setDrumVel(4); break;
		case 4: gpo_setDrumVel(5); break;
		case 5: gpo_setDrumVel(6); break;
		case 6: gpo_setDrumVel(7); break;
		case 7: gpo_setPumpEnable(TRUE); break;
		case 8: gpo_setDrumState(DRUM_FWD); break;
		case 9: gpo_setDrumState(DRUM_DIS); break;
		case 10: gpo_setDrumState(DRUM_REV); break;
		case 11: gpo_setWireControlEnable(TRUE); break;
		case 12: gpo_setVoltageEnable(TRUE); break;
		case 13: gpo_setVoltageLevel(1); break;
		case 14: gpo_setPulseWidth(31); gpo_setPulseRatio(7); break;
		case 15: gpo_setCurrentIndex(11); break;

		case 16: gpo_setCurrentIndex(0); break;
		case 17: gpo_setPulseWidth(36); gpo_setPulseRatio(8); break;
		case 18: gpo_setVoltageLevel(0); break;
		case 19: gpo_setVoltageEnable(FALSE); break;
		case 20: gpo_setWireControlEnable(FALSE); break;
		case 21: gpo_setDrumState(DRUM_DIS); break;
		case 22: gpo_setPumpEnable(FALSE); break;
		case 23: gpo_setDrumVel(1); break;
		default: break;
		}

		if (i >= 23)
			i = 0;
		else
			i++;

		gpo_apply();

		uint16_t sigo_reg = fpga_getOutput();
		uint16_t gen_reg = fpga_getGenerator();

		printf("WR:%08x GEN:%04x SIGO: %04x\n", (int)dato32.data, (int)sigo_reg, (int)gen_reg);
	}
}
