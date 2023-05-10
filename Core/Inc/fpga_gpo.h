#ifndef INC_FPGA_GPO_H_
#define INC_FPGA_GPO_H_

#include "my_types.h"

typedef enum {DRUM_DIS = 0, DRUM_FWD = 1, DRUM_REV = 2, DRUM_ANY = 3} drum_state_t;

#define CURRENT_CODES (12)
extern const uint8_t current_codes[CURRENT_CODES];
size_t seek_index(const uint8_t* array, size_t size, uint8_t value);

//typedef struct packed {
//	logic [1:0] res;
//	logic pump_ena;
//	logic [2:0] drum_vel;
//	logic drum_rev; // m
//	logic drum_fwd; // m
//
//	logic [5:0] current; // m
//	logic hv_lvl;
//	logic hv_ena;
//} sig_out_t;
//
//typedef struct packed {
//	logic [7:0] ratio; // 8
//	logic [7:0] width; // 36
//} gen_t;

typedef union {
    uint32_t data;
    struct {
    	uint32_t voltage_ena:1;
    	uint32_t voltage_level:1;
    	uint32_t current_code:6;

    	uint32_t drum_state:2; // fwd = 1, rev = 2
    	uint32_t drum_vel:3;
    	uint32_t pump_ena:1;
    	uint32_t center_ena:1;
    	uint32_t :1;

    	uint32_t pulse_width:8;
    	uint32_t pulse_ratio:8;
    } field;
} fpga_controls_t;

typedef union {
	uint16_t data;
	struct {
		uint16_t ena:1;
		uint16_t drum_vel:7;

		uint16_t pump_ena:1;
		uint16_t drum_ena:1;
		uint16_t wire_ctrl:1;
		uint16_t :5;
	} field;
} fpga_ind_t;

typedef union {
    uint16_t data;
    struct {
        uint16_t drum_forward:1;
        uint16_t drum_reverse:1;
        uint16_t alarm:1;
        uint16_t wire_ctrl:1;
        uint16_t power_off:1;
        uint16_t soft:1;
    } field;
} fpga_lim_switch_t;

extern fpga_controls_t dato32;
extern size_t current_index;
extern fpga_ind_t dato16;

void gpo_setControlsEnable(uint16_t ena, uint16_t mask);

void gpo_updateIndicator();
void gpo_store();
void gpo_restore();

void gpo_disableVDP();

void gpo_setDrumState(drum_state_t state);
void gpo_setDrumStateReg(drum_state_t state);
void gpo_setDrumStateIf(drum_state_t state);
void gpo_setDrumForward();
void gpo_setDrumReverse();
int gpo_setDrumEnable(BOOL ena);
void gpo_setDrumEnableKey(BOOL ena);
void gpo_switchDrumKey();
void gpo_switchDrumDir();

drum_state_t gpo_getDrumState();
BOOL gpo_getDrumEnable();

void gpo_setDrumVel(uint8_t code);
uint8_t gpo_getDrumVel();
void gpo_drumVelInc();
void gpo_drumVelDec();

void gpo_setCurrentIndex(size_t index);

void gpo_setPulseWidth(uint8_t value);
void gpo_setPulseRatio(uint8_t value);

void gpo_setVoltageLevel(uint8_t code);
int gpo_setVoltageEnable(BOOL value);
void gpo_setVoltageEnableKey(BOOL value);

int gpo_setPumpEnable(BOOL value);
void gpo_setPumpEnableKey(BOOL value);
void gpo_switchPumpKey();
void gpo_setCenterEnable(BOOL value);

void gpo_readControls();
void gpo_apply();
BOOL gpo_getValid();

BOOL gpo_getCncEnable();

BOOL gpo_isRun();

drum_state_t getDrumState(uint32_t value);
BOOL getDrumEnable(uint32_t value);
uint8_t getDrumVel(uint32_t value);
size_t getCurrentIndex(uint32_t value);
uint8_t getPulseRatio(uint32_t value);
uint8_t getPulseWidth(uint32_t value);
uint8_t getVoltageLevel(uint32_t value);
BOOL getVoltageEnable(uint32_t value);
BOOL getPumpEnable(uint32_t value);

// LIM SW MASK
BOOL getWireControlEnable(uint16_t mask);
void gpo_switchWireControl();
void gpo_setWireControlEnable(BOOL ena);
void gpo_setLimSwMask(uint16_t set, uint16_t clear);

void gpo_cncEnable(BOOL value);
BOOL gpo_cncEnabled();

#endif /* INC_FPGA_GPO_H_ */
