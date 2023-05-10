#ifndef INC_FPGA_H_
#define INC_FPGA_H_

#include "my_types.h"
#include "defines.h"

#define FPGA_PERIOD (1.0 / FPGA_CLOCK) // s

#define BANK1 (0xC0000000)

#define MTR_BAR	(0)
#define ENC_BAR	(0x100)
#define CTRL_BAR (0x180)
#define ADC_BAR (0x1C0)
#define HV_BAR (0x200)

#define MTR_NUM (4)

#define NT32 (MTR_BAR)
#define MOTOR_DIR (MTR_BAR + 0x40)

#define SWAP_XY_BIT (8)
#define SWAP_UV_BIT (9)

#define MOTOR_OE (MTR_BAR + 0x42)
#define TASK_ID (MTR_BAR + 0x44)
#define MTR_WRREQ (MTR_BAR + 0x48)

#define MTR_STATUS (MTR_BAR + 0x4A)
#define MTR_CONTROL (MTR_STATUS)
#define MTR_TIMEOUT_BIT (1<<0)
#define MTR_ABORT_BIT (1<<1)
#define MTR_GLOBAL_SNAPSHOT_BIT (1<<2)

#define TIMEOUT32 (MTR_BAR + 0x4C)
#define TIMEOUT_ENA (MTR_BAR + 0x50)
#define SLOW_SCALE (MTR_BAR + 0x54)
#define SD_OE (MTR_BAR + 0x58)
#define SD_ENA (MTR_BAR + 0x5A)

#define TIMEOUT_WIDTH (20)
#define TIMEOUT_MASK ((1<<TIMEOUT_WIDTH) - 1)
#define TIMEOUT_MAX (TIMEOUT_MASK)

#define POS32 (MTR_BAR + 0x80)

#define ENC_NUM	(2)

#define ENC_FLAGS (ENC_BAR)
#define ENC_CLEAR (ENC_BAR)
#define ENC_STATUS (ENC_BAR + 2)
#define ENC_ENABLE (ENC_BAR + 4)
#define ENC_DIR (ENC_BAR + 6)
#define ENC_SNAPSHOT (ENC_BAR + 8)

#define ENC32_Z32 (ENC_BAR + 0x40)
#define ENC32_0 (ENC32_Z32 + 0)
//#define ENC_Z32_0 (ENC_BAR + 0x14)
#define ENC32_1 (ENC32_Z32 + 8)
//#define ENC_Z32_1 (ENC_BAR + 0x1C)

#define STATUS (CTRL_BAR)
#define IRQ_MASK (CTRL_BAR + 2)

#define FLAG_LIMSW_MSK (1<<0)
#define FLAG_NO_PERMIT_MSK (1<<1)
#define FLAG_TIMEOUT_MSK (1<<2)
#define FLAG_HV_ENA_MSK (1<<3)

#define FPGA_RESET (CTRL_BAR + 4)

#define SIGNAL_STATUS (CTRL_BAR + 6)

#define LIMSW_MASK (CTRL_BAR + 8)
//#define LIMSW_LEVEL (CTRL_BAR + 0xA)
#define LIMSW (CTRL_BAR + 0xC)
#define LIMSW_FLAG (CTRL_BAR + 0xE)

//#define LIM_FWD_MSK (1<<0)
//#define LIM_REV_MSK (1<<1)

#define LIM_FWD_MSK (1<<0)
#define LIM_REV_MSK (1<<1)
#define LIM_ALARM_MSK (1<<2)
#define LIM_WIRE_MSK (1<<3)
#define LIM_POWER_MSK (1<<4)
#define LIM_SOFT_MSK (1<<5)

#define SIGI_BYTES (2)
#define CONTROL_BYTES (4)
#define PULT_BYTES (2)

#define SIGI_LEVEL (CTRL_BAR + 0x10)
#define SIGI (CTRL_BAR + 0x12)
#define SIGI_FLAG (CTRL_BAR + 0x14)
#define SIGI_TIMEOUT (CTRL_BAR + 0x16)

#define SEM (CTRL_BAR + 0x18)
#define SEM_MSK (0x3F)
#define SEM_ENA_BIT (8)
#define SEM_ENA_MSK (1<<SEM_ENA_BIT)

#define OE (CTRL_BAR + 0x1A)
#define CENTER (CTRL_BAR + 0x1C)

#define SIGI_ALARM_MSK (1<<0) // level
#define SIGI_FWD_MSK (1<<1) // level
#define SIGI_REV_MSK (1<<2) // level

#define SIGI_WIRE_BREAK_MSK (1<<8) // level
#define SIGI_POWER_OFF_MSK (1<<9) // level

#define SIGI_MASK_ALL (SIGI_ALARM_MSK | SIGI_FWD_MSK | SIGI_REV_MSK |  SIGI_WIRE_BREAK_MSK | SIGI_POWER_OFF_MSK)

#define SIG_OUT (CTRL_BAR + 0x20)
#define GEN_OUT (CTRL_BAR + 0x22)

#define CONTROL32_OUT (SIG_OUT)
#define CONTROL32_OLD (CTRL_BAR + 0x28)
#define GEN_OLD (CTRL_BAR + 0x2A)

#define PULT_OUT (CTRL_BAR + 0x30)
#define PULT_OLD (CTRL_BAR + 0x32)

#define PULT_WIRE_ENA (1<<0)

#define TEST_ENA (CTRL_BAR + 0x38)
#define LED (CTRL_BAR + 0x3A)
#define VER32 (CTRL_BAR + 0x3C)

#define ADC_WIDTH (10)
#define ADC_MASK ((1 << ADC_WIDTH) - 1)
#define ADC_MAX (ADC_MASK)
#define ADC_NUM (8)

#define ADC_VREF (4.096) // V
#define ADC_ERROR (0xEEEE)

#define ADC_SNAPSHOT (ADC_BAR)
#define ADC (ADC_BAR)
#define ADC_LOW_THLD (ADC_BAR + 0x10)
#define ADC_HIGH_THLD (ADC_BAR + 0x12)
#define FB_ENA (ADC_BAR + 0x14)
#define SOFT_PERMIT (ADC_BAR + 0x16)
#define ADC_STATUS (ADC_BAR + 0x18)
#define CENTER_MODE (ADC_BAR + 0x1A)
#define CENTER_THLD (ADC_BAR + 0x1C)

#define ADC_CLESS_FLAG (1<<2)
#define ADC_CENTER_FLAG (1<<8)

#define HV_PER32 (HV_BAR)
#define HV_T32_0 (HV_BAR + 4)
#define HV_T32_1 (HV_BAR + 8)
#define HV_CODE (HV_BAR + 0xC)
#define HV_UPDATE (HV_BAR + 0xE)

#define HV_ENA (HV_BAR + 0x10)
#define HV_INV (HV_BAR + 0x12)

typedef union {
    struct {
        uint32_t rev:8;
        uint32_t ver:8;
        uint32_t fac_rev:8;
        uint32_t fac_ver:6;
        uint32_t type:2;
    } field;
    uint32_t data;
} cnc_version_t;

//
typedef struct {
	uint32_t alarm:1;
	uint32_t roolback_req:1;
} alt_status_t;

uint16_t fpga_read_u16(uint16_t index);
int16_t fpga_read_s16(uint16_t index);
uint32_t fpga_read_u32(uint16_t index);
int32_t fpga_read_s32(uint16_t index);
uint64_t fpga_read_u64(uint16_t index);
int64_t fpga_read_s64(uint16_t index);
uint64_t fpga_read_u48(uint16_t index);

void fpga_write_u16(uint16_t index, uint16_t data);
void fpga_write_s16(uint16_t index, int16_t data);
void fpga_write_u32(uint16_t index, uint32_t data);
void fpga_write_s32(uint16_t index, int32_t data);
void fpga_write_u64(uint16_t index, uint64_t data);
void fpga_write_s64(uint16_t index, int64_t data);
void fpga_write_u48(uint16_t index, uint64_t data);

// Motors
void fpga_step(uint8_t i, int N, unsigned T);
int32_t fpga_getN(uint8_t i);
uint32_t fpga_getT(uint8_t i);

void fpga_setMotorDir(uint16_t dir);
uint16_t fpga_getMotorDir();
void fpga_setMotorDirSwap(uint8_t dir, uint8_t swap);
void fpga_getMotorDirSwap(uint8_t* const dir, uint8_t* const swap);
void fpga_setMotorOE(BOOL oe);
BOOL fpga_getMotorOE();
uint16_t fpga_getRun();
uint8_t fpga_getWrreq();
BOOL fpga_getWrRdy();
BOOL fpga_getStop();

BOOL fpga_isTimeout();
void fpga_clearTimeout();

void fpga_setTimeout(uint32_t value);
uint32_t fpga_getTimeout();

void fpga_setTimeoutEnable(BOOL value);
BOOL fpga_getTimeoutEnable();

void fpga_setSdOutEnable(BOOL value);
BOOL fpga_getSdOutEnable();

void fpga_setSdEnable(BOOL value);
BOOL fpga_getSdEnable();

void fpga_globalSnapshot();

void fpga_setTaskID(int32_t value);
int32_t fpga_getTaskID();

void fpga_setPos(uint8_t index, int32_t value);
void fpga_setEnc(uint8_t index, int32_t value);
void fpga_setDist(uint8_t index, uint32_t value);

int32_t fpga_getPos(uint8_t index);
int32_t fpga_getPosX();
int32_t fpga_getPosY();
int32_t fpga_getPosU();
int32_t fpga_getPosV();

int32_t fpga_getPosXEnc();
int32_t fpga_getPosYEnc();

// Control module
uint16_t fpga_getStatus();
//void fpga_clearFlags(uint16_t value);
void fpga_setIrqMask(uint16_t value);
uint16_t fpga_getIrqMask();

BOOL fpga_getSpiBusy();

void fpga_setLimSwMask(uint16_t value);
uint16_t fpga_getLimSwMask();

void fpga_setSoftAlarm();
void fpga_clearSoftAlarm();

uint16_t fpga_getLimSw();
uint16_t fpga_getLimSwFlags();
void fpga_clearLimSwFlags(uint16_t value);

void fpga_setPult(uint16_t value);
uint16_t fpga_getPult();
uint16_t fpga_getPultOld();

void fpga_setInputLevel(uint16_t value);
uint16_t fpga_getInputLevel();

uint16_t fpga_getInput();
uint16_t fpga_getInputFlag();
void fpga_clearInputFlag(uint16_t value);

void fpga_setOutput(uint16_t value);
uint16_t fpga_getOutput();

void fpga_setGenerator(uint16_t value);
uint16_t fpga_getGenerator();
uint16_t fpga_getGeneratorOld();

void fpga_setControls(uint32_t value);
uint32_t fpga_getControls();
uint32_t fpga_getControlsOld();

cnc_version_t fpga_getVersion();

// ADCs
void fpga_adcSnapshot();
uint16_t fpga_getADC(uint8_t i);

void fpga_setLowThld(uint16_t low);
void fpga_setHighThld(uint16_t high);
uint16_t fpga_getLowThld();
uint16_t fpga_getHighThld();

void fpga_setFeedbackEnable(BOOL ena);
BOOL fpga_getFeedbackEnable();
void fpga_setSoftPermit(BOOL ena);
BOOL fpga_getSoftPermit();

uint16_t fpga_getADCStatus();
BOOL fpga_getPermit();
BOOL fpga_getADCPermit();
BOOL fpga_getCentered();
BOOL fpga_getCenterHvOk();
void fpga_clearCentered();

typedef enum {FPGA_CENTER_MODE_OFF, FPGA_CENTER_MODE_FORWARD, FPGA_CENTER_MODE_REVERSE} FPGA_CENTER_MODE_T;
void fpga_setCenterMode(FPGA_CENTER_MODE_T state);
uint16_t fpga_getCenterMode();
void fpga_setCenterThld(uint16_t low);
void fpga_setCenterHighThld(uint16_t high);
uint16_t fpga_getCenterThld();
uint16_t fpga_getCenterHighThld();

//
//int alt_read_ctx(point_t* const pt);
//void alt_clear();

uint32_t fpga_MsToTicks(double ms);
uint32_t fpga_MsToPauseTicks(double ms);

double fpga_ticksToMs(uint32_t ticks);
double fpga_pauseTicksToMs(uint32_t ticks);

void fpga_reset();
void fpga_init();

void fpga_motorAbort();

void fpga_testEnable(BOOL value);
BOOL fpga_testEnabled();

uint16_t fpga_readSemReg();
void fpga_writeSemReg(uint16_t value);

BOOL fpga_readOeReg();
void fpga_writeOeReg(BOOL value);

#endif /* INC_FPGA_H_ */
