#include <stdio.h>
#include "stm32h7xx_hal.h"

#include "key_task.h"
#include "my_types.h"
#include "fpga.h"
#include "fpga_gpo.h"

/*
 * The task works with the MCU pins that according to the pult's buttons (keys) and LEDs
 */

static BOOL init; // pult initialized
static int init_cnt; // counter for printing debug information

key_counter_t key_cnt[KEY_NUM] = {{KEY_CNT_MAX, TRUE}}; // key filter
static keys_t keys, keys_flt, clicked; // key state

BOOL cnc_isInit();

// Reset key task data
void key_task_reset() {
	init = FALSE;
	init_cnt = 0;

	for (int i = 0; i < KEY_NUM; i++) {
		key_cnt[i].cnt = KEY_CNT_MAX;
		key_cnt[i].state = TRUE; // if always 1 it's not key clicked
	}

	keys.data = 0;
	keys_flt.data = 0;
	clicked.data = 0;
}

// Pult is ready to work
BOOL key_isInit() { return init; }

// Function reads keys' state
static keys_t getKeys() {
	keys_t keys;
	keys.data = (uint8_t)((GPIOE->IDR) >> 2) & KEY_MASK;
	return keys;
}

/*
 * Method that implements a filter
 * Input
 *	sig - key state
 * Input-Output
 *	cnt - external counter
 */
// return: clicked
static BOOL key_counter(BOOL sig, key_counter_t* const cnt) {
	if (sig) {
		if (cnt->cnt < KEY_CNT_MAX) // filter only for 2 values
			cnt->cnt++;
		else if (cnt->state == FALSE) {
			cnt->state = TRUE;
			return TRUE;
		}
	}
	else {
		if (cnt->cnt > 0)
			cnt->cnt--;
		else
			cnt->state = FALSE;
	}

	return FALSE;
}

/*
 * The function is processing keys every key period
 * It returns keys that were pressed
 */
keys_t getKeysClicked() {
	static uint32_t tic, toc; // msec from system timers
	static uint32_t vel_cnt;

	toc = HAL_GetTick();

	if (toc - tic >= KEY_PERIOD) {
		tic += KEY_PERIOD;

		keys = getKeys();

		clicked.key.inc = key_counter(keys.key.inc, &key_cnt[0]);
		keys_flt.key.inc = key_cnt[0].state;

		clicked.key.dec = key_counter(keys.key.dec, &key_cnt[1]);
		keys_flt.key.dec = key_cnt[1].state;

		clicked.key.drum = key_counter(keys.key.drum, &key_cnt[2]);
		keys_flt.key.drum = key_cnt[2].state;

		clicked.key.pump = key_counter(keys.key.pump, &key_cnt[3]);
		keys_flt.key.pump = key_cnt[3].state;

		clicked.key.wire = key_counter(keys.key.wire, &key_cnt[4]);
		keys_flt.key.wire = key_cnt[4].state;

		// Automatic key cleaning every 0.5 sec. The user can just hold a key and the drum velocity will be changing
		if (keys_flt.key.inc ^ keys_flt.key.dec) {
			if (vel_cnt >= KEY_VEL_SPEED) {
				vel_cnt = 0;
				key_cnt[0].state = FALSE;
				key_cnt[1].state = FALSE;
			}
			else
				vel_cnt++;
		}
		else
			vel_cnt = 0;

		return clicked;
	}

	keys_t res = {0};
	return res;
}

void clearKeysClicked(keys_t clr) {
	clicked.data &= ~clr.data;
}

/*
 *
 */
void key_task() {
	if (init) {
		keys_t clicked = getKeysClicked();

		if (!clicked.data)
			return;

		clearKeysClicked(clicked);

		if (clicked.key.inc ^ clicked.key.dec) {
			if (clicked.key.inc) {
				gpo_drumVelInc();
#ifdef KEY_PRINT
				printf("DRUM_INC\n");
#endif
			}
			else {
				gpo_drumVelDec();
#ifdef KEY_PRINT
				printf("DRUM_DEC\n");
#endif
			}
		}

		if (clicked.key.drum) {
			gpo_switchDrumKey();
#ifdef KEY_PRINT
			printf("DRUM_ENA\n");
#endif
		}

		if (clicked.key.pump) {
			gpo_switchPumpKey();
#ifdef KEY_PRINT
			printf("SW_PUMP_ENA\n");
#endif
		}

		if (clicked.key.wire) {
			gpo_switchWireControl();
#ifdef KEY_PRINT
			printf("SW_BREAK_ENA\n");
#endif
		}

//		if (clicked & KEY_POWER_OFF_MSK)
//			printf("POWER_OFF\n");

		gpo_apply();

	} else { // wait initialization
		if (cnc_isInit()) {
			keys_t clicked = getKeysClicked();

			if (!keys_flt.data && !clicked.data) { // keys aren't pressed
				init = TRUE;
				gpo_setDrumVel(1); // minimal drum velocity
				gpo_apply(); // filters ready, init regs
#ifdef PRINT
				printf("Key init\n");
#endif
			}
			else {
				if (init_cnt <= 1) {
#ifdef PRINT
					if (init_cnt == 1) // print only once
						printf("Key init ERROR\n");
#endif

					init_cnt++;
				}
			}

			if (clicked.data)
				clearKeysClicked(clicked); // processed
		}
	}
}

void printKey() {
	keys_t key = getKeys();
	printf("PULT:%x\n", (int)key.data);
}
