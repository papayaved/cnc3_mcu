#include <stdio.h>

#include "my_wdt.h"
#include "stm32h7xx_hal.h"
#include "cnc_task.h"

static uint32_t tic, toc, cnt;
static BOOL wdt, enable;
static BOOL m_hard_wdt;

void soft_wdt_reset() {
	tic = toc = cnt = 0;
	wdt = enable = m_hard_wdt = FALSE;
}

BOOL soft_wdt() { return wdt; }

void soft_wdt_task() {
	if (enable) {
		toc = HAL_GetTick();

		if (toc - tic >= PC_WDT_PERIOD) {
			tic += PC_WDT_PERIOD;

			if (cnc_run() && !wdt && cnt) {
				cnc_onLoseConnection();
				wdt = TRUE;
				printf("sWDT");
			}

			cnt = 1;
		}
	}
	else {
		cnt = 0;
		wdt = FALSE;
	}
}

void soft_wdt_clear() {
	tic = HAL_GetTick();
	cnt = 0;
	wdt = FALSE;
}

void soft_wdt_enable(BOOL value) {
	if (value && enable) // already enabled
		return;

	soft_wdt_clear();
	enable = value;
}

BOOL soft_wdt_isEnabled() { return enable; }

BOOL hard_wdt() {
	return m_hard_wdt;
}

void hard_wdt_check() {
	m_hard_wdt = READ_BIT(RCC->RSR, RCC_RSR_WWDG1RSTF_Msk) != 0;

	/* Reset all RSR flags */
	SET_BIT(RCC->RSR, RCC_RSR_RMVF);
}
