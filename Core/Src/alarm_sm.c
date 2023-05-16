#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <ctype.h>
#include <math.h>
#include <limits.h>

#include "cnc_task.h"
#include "alarm_sm.h"
#include "fpga_gpo.h"
#include "pid.h"

typedef enum {ST_ALM_POWER, ST_ALM_WIRE, ST_ALM_ALARM, ST_ALM_SOFT, ST_ALM_EXIT, ST_ALM_NORMAL, ST_ALM_DRUM, ST_ALM_DRUM_ERROR} alarm_state_t;
static alarm_state_t state = ST_ALM_NORMAL;

uint8_t cnc_ena;

void alarm_reset() {
	state = ST_ALM_NORMAL;
}

BOOL alarm_isAlarm()	{ return state != ST_ALM_NORMAL && state != ST_ALM_DRUM; }
BOOL alarm_isIdle()		{ return state == ST_ALM_NORMAL; }
BOOL alarm_drumPermit()	{ return state == ST_ALM_NORMAL || state == ST_ALM_DRUM; }

void alarm_onFailure() {
	state = ST_ALM_POWER;
}

// Alarm state machine
void alarm_sm(uint16_t limsw_flags) {
	limsw_flags &= LIM_SOFT_MSK | LIM_POWER_MSK | LIM_WIRE_MSK | LIM_ALARM_MSK | LIM_REV_MSK | LIM_FWD_MSK;

//		if (limsw_reg) {
//			uint16_t limsw = fpga_getLimSw();
//			printf("F:%x LIM:%04x(REG:%04x)\n", (unsigned)flags, (unsigned)limsw, (unsigned)limsw_reg);
//
//			uint16_t flags = fpga_getFlags() & (FLAG_LIMSW_MASK | FLAG_TIMEOUT_MASK);
//			limsw = fpga_getLimSw();
//			uint16_t limsw_reg = fpga_getLimSwReg();
//			printf("F:%x LIM:%04x(REG:%04x)\n", (unsigned)flags, (unsigned)limsw, (unsigned)limsw_reg);
//		}

//	ena = gpo_getCncEnable();

//		if ((limsw_reg & ~LIM_SOFT_MSK) != 0 && !(limsw_reg | LIM_SOFT_MSK)) // if hard disable and no soft alarm
//			fpga_setSoftAlarm();

	if (limsw_flags)
		cnc_resetSpeed();

	if (limsw_flags & LIM_POWER_MSK) { // fpga auto stop at any ls_flags
		if (state != ST_ALM_POWER) {
			cnc_onFailure();
			pid_clear();
		}

		state = ST_ALM_POWER;

#ifdef PRINT
		printf("Power off\n");
#endif
	}
	else if (limsw_flags & (LIM_WIRE_MSK | LIM_ALARM_MSK | LIM_SOFT_MSK)) {
		if (cnc_isDirectState()) { // for soft pult
			center_reset();
			cnc_exit();
		}

		pid_clear();

		if (state != ST_ALM_POWER && state != ST_ALM_WIRE && state != ST_ALM_ALARM && state != ST_ALM_SOFT && state != ST_ALM_EXIT)
			cnc_onAlarm();

		if (limsw_flags & LIM_WIRE_MSK) {
			state = ST_ALM_WIRE;
#ifdef PRINT
			printf("Wire off\n");
#endif
		}
		else if (limsw_flags & LIM_ALARM_MSK) {
			state = ST_ALM_ALARM;
#ifdef PRINT
			printf("Alarm on\n");
#endif
		}
		else if (limsw_flags & LIM_SOFT_MSK) {
			state = ST_ALM_SOFT;
#ifdef PRINT
			printf("ST_ALM_SOFT\n");
#endif
		}
	}
	else {
		switch (state) {

		case ST_ALM_POWER:
			if (!(limsw_flags & LIM_POWER_MSK)) {
				state = ST_ALM_EXIT;
#ifdef PRINT
				printf("Power on\n");
#endif
			}
			break;

		case ST_ALM_WIRE:
			if (!(limsw_flags & LIM_WIRE_MSK)) {
				state = ST_ALM_EXIT;
#ifdef PRINT
				printf("Wire on\n");
#endif
			}
			break;

		case ST_ALM_ALARM:
			if (!(limsw_flags & LIM_ALARM_MSK)) {
				state = ST_ALM_EXIT;
#ifdef PRINT
				printf("Alarm off\n");
#endif
			}
			break;

		case ST_ALM_SOFT:
			if (cnc_isIdleStop()) {
				state = ST_ALM_EXIT;
				fpga_clearSoftAlarm(); // when set?
				cnc_ena = 1; // It uses for test purposes

#ifdef PRINT
				printf("Soft Alarm off\n");
#endif
			}
			break;

		case ST_ALM_EXIT: // one alarm has finished but other can exist
			state = ST_ALM_NORMAL;
			gpo_restore();

			if (limsw_flags) { // drum
#ifdef PRINT
				printf("Drum Alm exit\n");
#endif
			}
			else {
				gpo_apply();
#ifdef PRINT
				printf("Alm exit\n");
#endif
			}
			// no break, continue

		case ST_ALM_NORMAL:
			if (limsw_flags & (LIM_FWD_MSK | LIM_REV_MSK)) { // only drum
				drum_state_t drum_state = gpo_getDrumState();
				BOOL drum_ena = gpo_getDrumEnable();

				gpo_store();
				gpo_readControls();
				gpo_updateIndicator();

				if (	(limsw_flags & (LIM_FWD_MSK | LIM_REV_MSK)) == (LIM_FWD_MSK | LIM_REV_MSK) ||
						(drum_state == DRUM_FWD && (limsw_flags & LIM_REV_MSK)) ||
						(drum_state == DRUM_REV && (limsw_flags & LIM_FWD_MSK))
				) {
					cnc_stopReq();
					gpo_setPumpEnable(FALSE);

					gpo_setDrumState(DRUM_DIS);
					gpo_apply();
					state = ST_ALM_DRUM_ERROR;
#ifdef PRINT
					printf("Drum LIM ERR1\n");
#endif
				}
				else if (limsw_flags & LIM_FWD_MSK) {
					if (drum_ena) {
						gpo_setDrumState(DRUM_REV);
						gpo_apply();
					}
					else
						gpo_setDrumStateReg(DRUM_REV);

					state = ST_ALM_DRUM;
#ifdef PRINT
					printf("Drum LIM FWD\n");
#endif
				}
				else { // (limsw_reg & LIM_REV_MSK)
					if (drum_ena) {
						gpo_setDrumState(DRUM_FWD);
						gpo_apply();
					}
					else
						gpo_setDrumStateReg(DRUM_FWD);

					state = ST_ALM_DRUM;
#ifdef PRINT
					printf("Drum LIM REV\n");
#endif
				}

				pid_stop();
				cnc_resetSpeed();
			}
			else if (limsw_flags) { // other limit switches. Impossible state
				center_reset();
				cnc_exit();

#ifdef PRINT
				printf("ALM ERROR, %x\n", limsw_flags);
#endif
			}

			break;

		case ST_ALM_DRUM:
		case ST_ALM_DRUM_ERROR:
			if (!(limsw_flags & (LIM_FWD_MSK | LIM_REV_MSK))) { // exit - no limit switches
				state = ST_ALM_NORMAL;
				drum_state_t drum_state = gpo_getDrumState();
	//				uint8_t vel = gpo_getDrumVel();
				gpo_restore();
				gpo_setDrumState(drum_state);
	//				gpo_setDrumVel(vel);
				gpo_apply();
				fpga_clearSoftAlarm();
#ifdef PRINT
				printf("Drum lim on\n");
#endif
			}
			else if ( (limsw_flags & (LIM_FWD_MSK | LIM_REV_MSK)) == (LIM_FWD_MSK | LIM_REV_MSK) ) { // error - both limit switches
				cnc_stopReq();
				gpo_setPumpEnable(FALSE);

				gpo_setDrumState(DRUM_DIS);
				gpo_apply();
				state = ST_ALM_DRUM_ERROR;
#ifdef PRINT
				printf("Drum LIM ERR2\n");
#endif
			}
			else {
				drum_state_t drum_state = gpo_getDrumState();

				if (	(drum_state == DRUM_FWD && (limsw_flags & LIM_FWD_MSK)) ||
						(drum_state == DRUM_REV && (limsw_flags & LIM_REV_MSK))
				) { // incorrect behavior
					cnc_stopReq();
					gpo_setPumpEnable(FALSE);

					gpo_setDrumState(DRUM_DIS);
					gpo_apply();
					state = ST_ALM_DRUM_ERROR;
#ifdef PRINT
					printf("Drum LIM ERR3\n");
#endif
				}
			}
			break;

		default:
			cnc_onFailure();

#ifdef PRINT
			printf("ERROR alm_state\n");
#endif
			break;
		}
	}
}
