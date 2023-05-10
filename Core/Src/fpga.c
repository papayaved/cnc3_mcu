#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <math.h>

#include "fmc.h"
#include "stm32h7xx_hal_sram.h"

#include "fpga.h"
#include "defines.h"
#include "line.h"
#include "cnc_task.h"
#include "imit_fifo.h"

#include "step_dir.h"

static volatile uint16_t* const BANK1_U16 = (uint16_t*)BANK1;
static volatile int16_t* const BANK1_S16 = (int16_t*)BANK1;
static volatile uint32_t* const BANK1_U32 = (uint32_t*)BANK1;
static volatile int32_t* const BANK1_S32 = (int32_t*)BANK1;
static volatile uint64_t* const BANK1_U64 = (uint64_t*)BANK1;
static volatile int64_t* const BANK1_S64 = (int64_t*)BANK1;

uint16_t fpga_read_u16(uint16_t index) { return BANK1_U16[index]; }
int16_t fpga_read_s16(uint16_t index) { return BANK1_S16[index]; }
uint32_t fpga_read_u32(uint16_t index) { return BANK1_U32[index]; }
int32_t fpga_read_s32(uint16_t index) { return BANK1_S32[index]; }
uint64_t fpga_read_u64(uint16_t index) { return BANK1_U64[index]; }
int64_t fpga_read_s64(uint16_t index) { return BANK1_S64[index]; }

uint64_t fpga_read_u48(uint16_t index) {
	uint64_t data;
	uint16_t* const p16 = (uint16_t*)&data;
	uint32_t* const p32 = (uint32_t*)&data;

	p32[0] = BANK1_U32[index<<1];
	p16[2] = BANK1_U16[(index<<2) + 2];
	p16[3] = 0;

	return data;
}

void fpga_write_u16(uint16_t index, uint16_t data) { BANK1_U16[index] = data; }
void fpga_write_s16(uint16_t index, int16_t data) { BANK1_S16[index] = data; }
void fpga_write_u32(uint16_t index, uint32_t data) { BANK1_U32[index] = data; }
void fpga_write_s32(uint16_t index, int32_t data) { BANK1_S32[index] = data; }
void fpga_write_u64(uint16_t index, uint64_t data) { BANK1_U64[index] = data; }
void fpga_write_s64(uint16_t index, int64_t data) { BANK1_S64[index] = data; }

void fpga_write_u48(uint16_t index, uint64_t data) {
	uint16_t* const p16 = (uint16_t*)&data;
	uint32_t* const p32 = (uint32_t*)&data;

	BANK1_U32[index<<1] = p32[0];
	BANK1_U16[(index<<2) + 2] = p16[2];
}

// MOTORS
void fpga_step(uint8_t i, int N, unsigned T) {
	if (i < MTR_NUM) {
		fpga_write_s32((NT32>>2) + (i << 1), N);
		fpga_write_u32((NT32>>2) + 1 + (i << 1), T);
	}
}

int32_t fpga_getN(uint8_t i) {
	uint32_t data;

	if (i < MTR_NUM) {
		data = fpga_read_u32((NT32>>2) + (i << 1));
		return *(int32_t*)&data;
	}
	return 0;
}

uint32_t fpga_getT(uint8_t i) {
	if (i < MTR_NUM) {
		return fpga_read_u32((NT32>>2) + 1 + (i << 1));
	}
	return 0;
}

void fpga_setMotorDir(uint16_t dir) { fpga_write_u16(MOTOR_DIR>>1, dir); }
uint16_t fpga_getMotorDir() { return fpga_read_u16(MOTOR_DIR>>1); }

void fpga_setMotorDirSwap(uint8_t dir, uint8_t swap) { fpga_setMotorDir((uint16_t)swap<<8 | (uint16_t)dir); }
void fpga_getMotorDirSwap(uint8_t* const dir, uint8_t* const swap) {
	uint16_t data = fpga_getMotorDir();
	if (dir) *dir = (uint8_t)data;
	if (swap) *swap = (uint8_t)(data>>8);
}

void fpga_setMotorOE(BOOL oe) { fpga_write_u16(MOTOR_OE>>1, oe ? 1 : 0); }
BOOL fpga_getMotorOE() { return fpga_read_u16(MOTOR_OE>>1) == 1; }

uint16_t fpga_getRun() { return fpga_read_u16(MTR_WRREQ>>1); }
BOOL fpga_getStop() { return fpga_getRun() == 0; }

uint8_t fpga_getWrreq() {
	uint16_t run = fpga_getRun();
//#ifdef PRINT
//	uint16_t thld = fpga_getThld();
//	BOOL adc_ena = fpga_getAdcEnable();
//	BOOL run_ena = fpga_getRunEnable();
//	BOOL permit = fpga_getPermit();
//	printf("run=%x thld=%x adc_ena=%x run_ena=%x permit=%x\n", run, thld, adc_ena, run_ena, permit);
//#endif
	return run;
}
BOOL fpga_getWrRdy() { return fpga_getWrreq() == 0; }

BOOL fpga_isTimeout() { return (fpga_read_u16(MTR_STATUS>>1) & MTR_TIMEOUT_BIT) != 0; }

void fpga_clearTimeout() { fpga_write_u16(MTR_CONTROL>>1, MTR_TIMEOUT_BIT); }
void fpga_motorAbort() { fpga_write_u16(MTR_CONTROL>>1, MTR_ABORT_BIT); }
void fpga_globalSnapshot() { fpga_write_u16(MTR_CONTROL>>1, MTR_GLOBAL_SNAPSHOT_BIT); }

void fpga_setTaskID(int32_t value) { fpga_write_s32(TASK_ID>>2, value); }
int32_t fpga_getTaskID() { return fpga_read_s32(TASK_ID>>2); }

void fpga_setTimeout(uint32_t value) { fpga_write_u32(TIMEOUT32>>2, value); }
uint32_t fpga_getTimeout() { return fpga_read_u32(TIMEOUT32>>2); }

void fpga_setTimeoutEnable(BOOL value) { fpga_write_u16(TIMEOUT_ENA>>1, value != 0); }
BOOL fpga_getTimeoutEnable() { return fpga_read_u16(TIMEOUT_ENA>>1) & 1; }

void fpga_setSdOutEnable(BOOL value) { fpga_write_u16(SD_OE>>1, value != 0); }
BOOL fpga_getSdOutEnable() { return fpga_read_u16(SD_OE>>1) & 1; }

void fpga_setSdEnable(BOOL value) { fpga_write_u16(SD_ENA>>1, value != 0); }
BOOL fpga_getSdEnable() { return fpga_read_u16(SD_ENA>>1) & 1; }

void fpga_setPos(uint8_t index, int32_t value) { fpga_write_s32((POS32>>2) + (index<<1), value); }
//void fpga_setDist(uint8_t index, uint32_t value) { fpga_write_u32((POS32>>2) + (index<<1) + 1, value); }

int32_t fpga_getPos(uint8_t index) { return fpga_read_s32((POS32>>2) + (index<<1)); }
int32_t fpga_getPosX() { return fpga_read_s32(POS32 >> 2); }
int32_t fpga_getPosY() { return fpga_read_s32((POS32 + 8) >> 2); }
int32_t fpga_getPosU() { return fpga_read_s32((POS32 + 0x10) >> 2); }
int32_t fpga_getPosV() { return fpga_read_s32((POS32 + 0x18) >> 2); }

// Position at encoder
int32_t fpga_getPosXEnc() { return fpga_read_s32( (POS32 + 4) >> 2 ); }
int32_t fpga_getPosYEnc() { return fpga_read_s32( (POS32 + 0xC) >> 2 ); }

void fpga_getPosXY(int32_t* const pos_x, int32_t* const pos_x_enc, int32_t* const pos_y, int32_t* const pos_y_enc) {
	*pos_x		= fpga_read_s32(POS32 >> 2);
	*pos_x_enc	= fpga_read_s32((POS32 >> 2) + 1);
	*pos_y		= fpga_read_s32((POS32 >> 2) + 2);
	*pos_y_enc	= fpga_read_s32((POS32 >> 2) + 3);
}

// Control module
uint16_t fpga_getStatus() { return fpga_read_u16(STATUS>>1); }
//void fpga_clearFlags(uint16_t value) { fpga_write_u16(IRQ_FLAGS>>1, value); }
void fpga_setIrqMask(uint16_t value) { fpga_write_u16(IRQ_MASK>>1, value); }
uint16_t fpga_getIrqMask() { return fpga_read_u16(IRQ_MASK>>1); }

void fpga_reset() { fpga_write_u16(FPGA_RESET>>1, 1); }

void fpga_setLimSwMask(uint16_t value) { fpga_write_u16(LIMSW_MASK>>1, value); }
uint16_t fpga_getLimSwMask() { return fpga_read_u16(LIMSW_MASK>>1); }

void fpga_setSoftAlarm() {
	fpga_write_u16(LIMSW>>1, LIM_SOFT_MSK);
#ifdef PRINT
	printf("Soft alarm\n");
#endif
}
void fpga_clearSoftAlarm() {
	fpga_write_u16(LIMSW_FLAG>>1, LIM_SOFT_MSK);
#ifdef PRINT
	printf("Clear soft alarm\n");
#endif
}


uint16_t fpga_getLimSw() { return fpga_read_u16(LIMSW>>1); }
uint16_t fpga_getLimSwFlags() { return fpga_read_u16(LIMSW_FLAG>>1); }
void fpga_clearLimSwFlags(uint16_t value) { fpga_write_u16(LIMSW_FLAG>>1, value); }

cnc_version_t fpga_getVersion() {
	return (cnc_version_t)fpga_read_u32(VER32>>2);
}

// ADCs
void fpga_adcSnapshot() { fpga_write_u16(ADC_SNAPSHOT>>1, 1); }

// return ADC data or ADC_ERROR_VALUE (0xEEEE)
uint16_t fpga_getADC(uint8_t i) {
	uint16_t data;

	if (i < ADC_NUM) {
		data = fpga_read_u16((ADC>>1) + i);
		return data <= ADC_MAX ? data : ADC_ERROR;
	}
	return ADC_ERROR;
}

void fpga_setLowThld(uint16_t low) { fpga_write_u16(ADC_LOW_THLD>>1, low); }
void fpga_setHighThld(uint16_t high) { fpga_write_u16(ADC_HIGH_THLD>>1, high); }
uint16_t fpga_getLowThld() { return fpga_read_u16(ADC_LOW_THLD>>1); }
uint16_t fpga_getHighThld() { return fpga_read_u16(ADC_HIGH_THLD>>1); }

void fpga_setFeedbackEnable(BOOL ena) { fpga_write_u16(FB_ENA>>1, ena ? 1 : 0); }
BOOL fpga_getFeedbackEnable() { return fpga_read_u16(FB_ENA>>1) == 1; }
void fpga_setSoftPermit(BOOL ena) { fpga_write_u16(SOFT_PERMIT>>1, ena ? 1 : 0); }
BOOL fpga_getSoftPermit() { return fpga_read_u16(SOFT_PERMIT>>1) == 1; }

uint16_t fpga_getADCStatus() { return fpga_read_u16(ADC_STATUS>>1); }

//BOOL fpga_getPermit() { return (fpga_getADCState() & 3) == 0; }
BOOL fpga_getADCPermit() { return (fpga_getADCStatus() & 3) == 0; }
BOOL fpga_getCentered() {
	uint16_t status = fpga_getADCStatus();
	BOOL res = (status & ADC_CENTER_FLAG) != 0;
	fpga_clearCentered();
	return res;
}
BOOL fpga_getCenterHvOk() { return (fpga_getADCStatus() & 4) == 0; }

void fpga_clearCentered() { return fpga_write_u16(ADC_STATUS>>1, ADC_CENTER_FLAG); }

// 1 - forward threshold, 2 - reverse threshold
void fpga_setCenterMode(FPGA_CENTER_MODE_T mode) { fpga_write_u16(CENTER_MODE>>1, (uint16_t)mode); }
uint16_t fpga_getCenterMode() { return fpga_read_u16(CENTER_MODE>>1) & 3; }

void fpga_setCenterThld(uint16_t low) { fpga_write_u16(CENTER_THLD>>1, low); }
uint16_t fpga_getCenterThld() { return fpga_read_u16(CENTER_THLD>>1); }

// High Voltage
//void setHV(uint32_t per, uint32_t t0, uint32_t t1, uint8_t code) {
//	fpga_write32(HV_PER32>>2, per);
//	fpga_write32(HV_T32_0>>2, t0);
//	fpga_write32(HV_T32_1>>2, t1);
//	fpga_write16(HV_CODE>>1, code);
//	fpga_write16(HV_UPDATE>>1, 1);
//}
//
//uint32_t getHVPeriod() { return fpga_read32(HV_PER32>>2); }
//uint32_t getHVT0() { return fpga_read32(HV_T32_0>>2); }
//uint32_t getHVT1() { return fpga_read32(HV_T32_1>>2); }
//uint8_t getHVCode() { return (uint8_t)fpga_read16(HV_CODE>>1); }

//void setHVEna(BOOL enable) { fpga_write16(HV_ENA>>1, enable ? 1 : 0); }
//BOOL getHVEna() { return fpga_read16(HV_ENA>>1) == 1; }

//////////////////////////////////

//void alt_step_valid(int str_num) {
//	ctx = str_num;
//}

#ifdef RB_TEST
static BOOL test_req = TRUE;
#endif

void roolback_ack() {
	printf("RB Start\n");
#ifdef RB_TEST
	test_req = FALSE;
#endif
}

// value - ms
// max 59 ms
uint32_t fpga_MsToTicks(double value) {
	value = round( value * (FPGA_CLOCK / 1000) );

	if (value < 0)
		return 0;

	if (value > (uint32_t)UINT32_MAX)
		return (uint32_t)UINT32_MAX;

	return (uint32_t)value;
}

double fpga_ticksToMs(uint32_t ticks) {
	return (double)ticks * (1000.0 / FPGA_CLOCK);
}

// ms to pause clocks
uint32_t fpga_MsToPauseTicks(double value) {
	value = round( value * (DEFAULT_PAUSE_CLOCK / 1000) ); // * 10

	if (value < 0)
		return 0;

	if (value > (uint32_t)UINT32_MAX)
		return (uint32_t)UINT32_MAX;

	return (uint32_t)value;
}

double fpga_pauseTicksToMs(uint32_t ticks) {
	return (double)ticks * (1000.0 / DEFAULT_PAUSE_CLOCK); // * 0.1
}

BOOL fpga_getSpiBusy() {
	return (fpga_read_u16(SIGNAL_STATUS>>1) & 0xF) == 0;
}
void fpga_setPult(uint16_t value) { fpga_write_u16(PULT_OUT>>1, value); }
uint16_t fpga_getPult() { return fpga_read_u16(PULT_OUT>>1); }
uint16_t fpga_getPultOld() { return fpga_read_u16(PULT_OLD>>1); }

// INPUTS
void fpga_setInputLevel(uint16_t value) { return fpga_write_u16(SIGI_LEVEL>>1, value); }
uint16_t fpga_getInputLevel() { return fpga_read_u16(SIGI_LEVEL>>1); }

uint16_t fpga_getInput() { return fpga_read_u16(SIGI>>1); }

uint16_t fpga_getInputFlag() { return fpga_read_u16(SIGI_FLAG>>1); }
void fpga_clearInputFlag(uint16_t value) { return fpga_write_u16(SIGI_FLAG>>1, value); }

uint16_t fpga_getInputTimeout() { return fpga_read_u16(SIGI_TIMEOUT>>1); }

// OUTPUTS
void fpga_setOutput(uint16_t value) { fpga_write_u16(SIG_OUT>>1, value); }
uint16_t fpga_getOutput() { return fpga_read_u16(SIG_OUT>>1); }

void fpga_setGenerator(uint16_t value) { fpga_write_u16(GEN_OUT>>1, value); }
uint16_t fpga_getGenerator() { return fpga_read_u16(GEN_OUT>>1); }
uint16_t fpga_getGeneratorOld() { return fpga_read_u16(GEN_OLD>>1); }

void fpga_setControls(uint32_t value) { fpga_write_u32(CONTROL32_OUT>>2, value); }
uint32_t fpga_getControls() { return fpga_read_u32(CONTROL32_OUT>>2); }
uint32_t fpga_getControlsOld() { return fpga_read_u32(CONTROL32_OLD>>2); }

//
void fpga_testEnable(BOOL value) { fpga_write_u16(TEST_ENA>>1, value ? 1 : 0); }
BOOL fpga_testEnabled() { return fpga_read_u16(TEST_ENA>>1) & 1; }

uint16_t fpga_readSemReg() { return fpga_read_u16(SEM>>1); }
void fpga_writeSemReg(uint16_t value) { fpga_write_u16(SEM>>1, value); }

BOOL fpga_readOeReg() { return fpga_read_u16(OE>>1) & 1; }
void fpga_writeOeReg(BOOL value) { fpga_write_u16(OE>>1, value != 0); }

// TEST
#include "stm32h7xx_hal.h"

void printInput() {
	int lvl = fpga_getInputLevel();
	int sigi = fpga_getInput();
	int flag = fpga_getInputFlag();
	fpga_clearInputFlag((uint16_t)flag);
	int timeout = fpga_getInputTimeout();

	printf("LVL:%x SIGI:%x F:%x TO:%x\n", lvl, sigi, flag, timeout);
}

void printKey();

void input_test_task() {
	static const uint32_t PERIOD = 1000;
	static uint8_t cnt;
	static uint32_t tic, toc;
	static BOOL init = FALSE;

	if (!init) {
		init = TRUE;
		fpga_setInputLevel(0);
	}

	toc = HAL_GetTick();

	if (toc - tic >= PERIOD) {
		cnt++;
		tic += PERIOD;

		// TEST
//		printInput();
		printKey();
	}
}
