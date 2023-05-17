#ifndef INC_CNC_TASK_H_
#define INC_CNC_TASK_H_

#include "defines.h"
#include "my_types.h"
#include "fpga.h"
#include "gcode.h"
#include "center.h"

//#define TEST_REVERSE
#ifndef RESEASE
//	#define TEST_NUM (13)
#endif
//#define RB_TEST (0)

typedef struct {
	BOOL valid;
	double value;
} double_valid_t;

// Active flags in g-code frame
typedef struct {
	uint16_t G:1;

	uint16_t X:1;
	uint16_t Y:1;
	uint16_t U:1;
	uint16_t V:1;

	uint16_t I:1;
	uint16_t J:1;
	uint16_t R:1;

	uint16_t F:1;

	uint16_t M:1;
	uint16_t P:1;
	uint16_t Q:1;
} gflags_t;

typedef struct {
	uint32_t width;
	uint32_t ratio;
} pulse_t;

typedef struct {
	uint8_t pump_ena:1;
	uint8_t drum_ena:1;
	uint8_t hv_ena:1;
	uint8_t wire_control_ena:1;
	uint8_t inout_ena:1;
} cnc_param_flags_t;

typedef struct {
	uint8_t current_index;
	uint8_t low_hv_ena;
	uint8_t drum_vel;
	cnc_param_flags_t flags;
	pulse_t pulse;
} cnc_param_t;

void cnc_resetSpeed();

void cnc_clearDirectParam(cnc_param_t* const par);

void cnc_enableUV(BOOL value);
BOOL cnc_uvEnabled();

void cnc_setParam(size_t index, int32_t value);
int32_t cnc_getParam(size_t index);
void cnc_recovery();
void cnc_reqG92();
void cnc_reqG92_xyuv_enc(int32_t x, int32_t y, int32_t u, int32_t v, int32_t enc_x, int32_t enc_y);
void cnc_reqG1();
void cnc_centerReq(const center_t* const pcenter);

// Recovery
void cnc_goto(uint32_t str_num);

void cnc_task();
BOOL cnc_isNormal();

int cnc_getState();
BOOL cnc_isIdle();
BOOL cnc_isIdleStop();
int cnc_getStringNum();

void cnc_runReq();
void cnc_stopReq();
void cnc_cancelReq();

void cnc_revReq();

BOOL cnc_run();
BOOL cnc_stop();
BOOL cnc_pause();
BOOL cnc_error();

float cnc_getT();
float cnc_getCurrentT();
float cnc_speed();
void cnc_setSpeed(float F);

int cnc_setRollbackLength(float value);
float cnc_getRollbackLength();

void cnc_setRollbackAttempts(uint8_t value);
uint8_t cnc_getRollbackAttempt();
uint8_t cnc_getRollbackAttempts();

void cnc_applyMCmd();
BOOL cnc_setMCmd(const gcmd_t* const cmd);
void cnc_turnOff();
void cnc_turnOn();

void cnc_reset();
void cnc_state_reset();				  
BOOL cnc_isInit();

BOOL cnc_isRollback();
BOOL cnc_isReverse();

void cnc_gotoPos(const cnc_context_t* const ctx);

void cnc_printState();

BOOL cnc_fault();

void cnc_onLoseConnection();

void cnc_onFailure();
BOOL cnc_isDirectState();
void cnc_exit();
void cnc_onAlarm();

#endif /* INC_CNC_TASK_H_ */
