#include <stdio.h>
#include <math.h>

#include "stm32h7xx_hal.h"

#include "test.h"
#include "fpga.h"
#include "fpga_gpo.h"
#include "rx_buf.h"
#include "cnc_task.h"
#include "step_dir.h"
#include "cnc_func.h"
#include "uv.h"
#include "line.h"
#include "feedback.h"

void test() {
#ifdef TEST
	printf("CNC %d.%d.%d.%d%s ", FAC_VER, FAC_REV, VER, REV, VER_TYPE == 0 ? "a" : VER_TYPE == 1 ? "b" : VER_TYPE == 2 ? "" : "err" );
	printf("%s %s ", __DATE__, __TIME__);
	printf("at %d Hz\n", (int)SystemCoreClock);

	cnc_version_t ver = fpga_getVersion();
	printf("FPGA %d.%d.%d.%d%s ",\
			ver.field.fac_ver, ver.field.fac_rev, ver.field.ver, ver.field.rev,\
			ver.field.type == 0 ? "a" : ver.field.type == 1 ? "b" : ver.field.type == 2 ? "" : "err" );

	printf("Test\n");
	switch (TEST) {
	case 0:
		fpga_print_u16(0, 31);
		break;
	case 1:
		fpga_print_u32(0, 15);
		break;
	case 2:
		fpga_rw32(0, 15);
		break;
	case 3:
		rx_fifo_tb();
		break;
	case 4:
		uv_tb();
		break;
	case 5:
		line_test();
		break;
	default:
		break;
	}
#endif
}

void fpga_print_u16(uint16_t first_addr, uint16_t last_addr) {
	uint16_t a;

	for (a = first_addr; a <= last_addr; a++) {
		uint16_t d = fpga_read_u16(a);

		if ((a & 0xF) == 0) printf("%04X: ", (unsigned)a);

		printf("%04X", (unsigned)d);

		if ((a & 0xF) == 0xF)
			printf("\n");
		else
			printf(" ");
	}

	if ((a & 0xF) != 0)
		printf("\n");
}

void fpga_print_u32(uint16_t start, uint16_t stop) {
	uint16_t a;

	for (a = start; a <= stop; a++) {
		uint32_t d = fpga_read_u32(a);

		if ((a & 0x7) == 0) printf("%04X: ", (unsigned)a);

		printf("%08X", (unsigned)d);

		if ((a & 0x7) == 0x7)
			printf("\n");
		else
			printf(" ");
	}

	if ((a & 0x7) != 0)
		printf("\n");
}

void fpga_rw32(uint16_t start, uint16_t stop) {
	uint16_t a, i;

	for (a = start, i = 1; a <= stop; a++, i++)
		fpga_write_u32(a, i << 24 | i << 16 | i << 8 | i);

	fpga_print_u32(start, stop);
	fpga_reset();
	fpga_print_u32(start, stop);
}

// TEST TASK
void test_task() {
#ifdef TEST_TASK
	switch (TEST_TASK) {
	case 0: one_step_task(0, 300 * 4); break;
	case 1: step_by_step_task(300 * 4); break;
	case 2: step_by_step_slow_task(); break;
	case 3: dati_task(); break;
	case 4: dato_task(0x8001, 0x0ULL<<40 | 0x24ULL<<32 | 0x8U<<24 | 0x0<<16 | 0x0<<8 | 0x0); break;
	case 5: dato48_task(); break;
	case 6: dato16_task(); break;
	case 7: dati16_task(); break;
	case 8: adc_task(); break;
	}
#endif
}

// vel - steps / sec
void one_step_task(uint8_t i, double vel) {
	static BOOL init = TRUE;
	static uint32_t T = ~0U;

	if (i < MTR_NUM) {
		if (init) {
			fb_enable(FALSE);
			fpga_setSoftPermit(TRUE);
			fpga_setLowThld(0xFFFF);
			fpga_setHighThld(0xFFFF);
			fpga_setMotorOE(TRUE);

			T = (uint32_t)round(FPGA_CLOCK / fabs(vel)) - 1;

			uint16_t low_thld = fpga_getLowThld();
			uint16_t high_thld = fpga_getHighThld();
			BOOL fb_ena = fpga_getFeedbackEnable();
			BOOL soft_permit = fpga_getSoftPermit();
			BOOL permit = fpga_getPermit();
			BOOL oe = fpga_getMotorOE();

			printf("THLD: %04x, %04x FB_ENA: %x SOFT_PERMIT: %x PERMIT: %x OE: %x\n", (unsigned)low_thld, (unsigned)high_thld, (int)fb_ena, (int)soft_permit, (int)permit, (int)oe);

			init = FALSE;
		}

		if (fpga_getStop()) {
			fpga_step(i, 1, T);
			uint16_t run = fpga_getRun();
			int32_t read_N = fpga_getN(i);
			uint32_t read_T = fpga_getT(i);
			BOOL permit = fpga_getPermit();

			printf("MTR: %x N: %d T: %d RUN: 0x%04x PERMIT: %x\n", (unsigned)i, (int)read_N, (unsigned)read_T, (unsigned)run, (unsigned)permit);
		}
	}
}

void step_by_step_task(double vel) {
	static BOOL init = TRUE;
	static uint8_t i;
	static uint32_t T = ~0U;

	if (init) {
		fb_enable(FALSE);
		fpga_setSoftPermit(TRUE);
		fpga_setLowThld(0xFFFF);
		fpga_setHighThld(0xFFFF);
		fpga_setMotorOE(TRUE);

		T = (uint32_t)round(FPGA_CLOCK / fabs(vel)) - 1;

		uint16_t low_thld = fpga_getLowThld();
		uint16_t high_thld = fpga_getHighThld();
		BOOL fb_ena = fpga_getFeedbackEnable();
		BOOL soft_permit = fpga_getSoftPermit();
		BOOL permit = fpga_getPermit();
		BOOL oe = fpga_getMotorOE();

		printf("THLD: %04x, %04x FB_ENA: %x SOFT_PERMIT: %x PERMIT: %x OE: %x\n", (unsigned)low_thld, (unsigned)high_thld, (int)fb_ena, (int)soft_permit, (int)permit, (int)oe);

		init = FALSE;
	}

	if (fpga_getWrRdy()) {
//		uint16_t run = getRun();

		if (i >= MTR_NUM) i = 0;
		fpga_step(i, 1, T);

//		int32_t read_N = getN(i);
//		uint32_t read_T = getT(i);
//		BOOL permit = getPermit();

//		printf("MTR: %x N: %d T: %d RUN: 0x%04x PERMIT: %x\n", (unsigned)i, (int)read_N, (unsigned)read_T, (unsigned)run, (unsigned)permit);

		i++;
	}
}

void step_by_step_slow_task() {
	static uint32_t tic, toc;
	static BOOL enable;
	static uint8_t motor_cnt, step_cnt;
	static BOOL sign;

	if (enable) {
		toc = HAL_GetTick();

		if (toc - tic >= (TEST_PERIOD / 5)) {
			tic += (TEST_PERIOD / 5);

			uint16_t flags = fpga_getStatus();
//			fpga_clearFlags(flags);

			double T = speed_to_period(300.0 / 1000.0 * 60);

			switch (motor_cnt) {
			case 0:
				step_writeXYUV(0, T, sign ? -1 : 1, 0, 0, 0);
				break;
			case 1:
				step_writeXYUV(0, T, 0, sign ? -1 : 1, 0, 0);
				break;
			default:
				break;
			}

			if (step_cnt >= 9) {
				step_cnt = 0;

				if (motor_cnt)
					sign = !sign;

				motor_cnt = !motor_cnt;
			}
			else
				step_cnt++;

			fpga_globalSnapshot();

			int pos_x = fpga_getPos(0);
			int pos_y = fpga_getPos(1);

			printf("Pos:[%d, %d] f:%x\n", pos_x, pos_y, flags);
		}
	}
	else { // initial
		tic = HAL_GetTick();
		enable = TRUE;

		fb_enable(FALSE);
		fpga_setSoftPermit(TRUE);
		fpga_setLowThld(0xFFFF);
		fpga_setHighThld(0xFFFF);
		fpga_setMotorOE(TRUE);

		uint16_t low_thld = fpga_getLowThld();
		uint16_t high_thld = fpga_getHighThld();
		BOOL fb_ena = fpga_getFeedbackEnable();
		BOOL soft_permit = fpga_getSoftPermit();
		BOOL permit = fpga_getPermit();
		BOOL oe = fpga_getMotorOE();

		printf("THLD: %04x, %04x FB_ENA: %x SOFT_PERMIT: %x PERMIT: %x OE: %x\n", (unsigned)low_thld, (unsigned)high_thld, (int)fb_ena, (int)soft_permit, (int)permit, (int)oe);
	}
}

void dati_task() {
	static uint32_t tic, toc;
	static BOOL enable;

	if (enable) {
		toc = HAL_GetTick();

		if (toc - tic >= TEST_PERIOD) {
			tic += TEST_PERIOD;

			uint16_t keys = fpga_getInput();
			printf("DI: %04x\n", (int)keys);
		}
	}
	else { // initial
		tic = HAL_GetTick();
		enable = TRUE;
	}
}

void dato_task(uint16_t ind, uint64_t ctrl) {
	static uint32_t tic, toc;
	static BOOL enable;

	if (enable) {
		toc = HAL_GetTick();

		if (toc - tic >= TEST_PERIOD / 10) {
			tic += TEST_PERIOD;

			uint16_t rd_ind = fpga_getPultOld();
			uint64_t rd_ctrl = fpga_getControlsOld();
			uint64_t keys = fpga_getInput();

			fpga_setPult(ind); // console
			fpga_setControls(ctrl);

			printf("KEYS: %04x\n", (unsigned)keys);

			BOOL OK = rd_ind == ind;
			printf("IND W: %04x R: %04x ERR: %x\n", (unsigned)ind, (unsigned)rd_ind, (int)!OK);

			OK = rd_ctrl == ctrl;
			printf("CTRL W: %04x%08x R: %04x%08x ERR: %x\n", (unsigned)(ctrl>>32), (unsigned)ctrl, (unsigned)(rd_ctrl>>32), (unsigned)rd_ctrl, (int)!OK);
		}
	}
	else { // initial
		tic = HAL_GetTick();
		enable = TRUE;
	}
}

void dato48_task() {
	static uint32_t tic, toc;
	static BOOL enable;
	static uint8_t width = 2, interval = 2, current_idx = 0;
	static uint8_t wire_dir = 0, wire_code = 0;
	static uint8_t hv_level = 0;
	static BOOL hv_ena = FALSE, pump_ena = FALSE;
	static uint8_t motor_cnt = 0;

	if (enable) {
		toc = HAL_GetTick();

		if (toc - tic >= 1000) {
			tic += 1000;

			uint64_t rd_ctrl = fpga_getControlsOld();

			if (width > 99) width = 2;
			if (interval > 32) interval = 2;
			if (current_idx >= CURRENT_CODES) current_idx = 0;
			if (wire_dir > 2) wire_dir = 0;
			if (wire_code > 7) wire_code = 0;
			if (hv_level > 1) hv_level = 0;

//			uint64_t ctrl = (uint64_t)pump_ena << 42 | (uint64_t)hv_ena << 41 | (uint64_t)hv_level << 40 |
//					(uint64_t)width << 32 | (uint64_t)interval << 24 | (uint64_t)current_codes[current_idx] << 16 |
//					wire_code << 2 | wire_dir;

			uint64_t ctrl = (uint64_t)width << 32 | (uint64_t)interval << 24 | (uint64_t)current_codes[current_idx] << 16;

			fpga_setControls(ctrl);

			width++;
			interval++;
			current_idx++;
			wire_code++;
			if (wire_code > 7)
				wire_dir++;

			if (!pump_ena)
				pump_ena = TRUE;
			else {
				if (!hv_ena)
					hv_ena = TRUE;
				else {
					hv_level++;
					if (hv_level > 1) {
						hv_ena = FALSE;
						pump_ena = FALSE;
					}
				}
			}

			printf("CTRL W: %04x%08x R: %04x%08x\n", (unsigned)(ctrl>>32), (unsigned)ctrl, (unsigned)(rd_ctrl>>32), (unsigned)rd_ctrl);

			// Motors test
			uint32_t T = speed_to_period(300.0 / 1000.0 * 60);

			if (motor_cnt > 1) motor_cnt = 0;

			switch (motor_cnt) {
			case 0:
				step_writeXYUV(0, T, 1, 0, 0, 0);
				break;
			case 1:
				step_writeXYUV(0, T, 0, 1, 0, 0);
				break;
			default:
				break;
			}

			motor_cnt++;
		}
	}
	else { // initial
		tic = HAL_GetTick();
		enable = TRUE;
	}
}

void dato16_task() {
	static uint32_t tic, toc;
	static BOOL enable;
	static uint16_t data;

	if (enable) {
		toc = HAL_GetTick();

		if (toc - tic >= 1000) {
			tic += 1000;

			uint16_t rd_ind = fpga_getPultOld();

			fpga_setPult(data);

			printf("IND W:%04x R:%04x\n", (unsigned)data, (unsigned)rd_ind);

			if (data == 0)
				data = 1;
			else if (data == 0x8000)
				data = 0x000ff;
			else if (data == 0x00ff)
				data = 0xff00;
			else if (data == 0xff00)
				data = 0;
			else
				data <<= 1;
		}
	}
	else { // initial
		tic = HAL_GetTick();
		enable = TRUE;
	}
}

void dati16_task() {
	const uint32_t PERIOD = 1000;
	static uint32_t tic, toc;
	static BOOL enable;

	if (enable) {
		toc = HAL_GetTick();

		if (toc - tic >= PERIOD) {
			tic += PERIOD;

			uint32_t keys = fpga_getInput();
			uint32_t up = fpga_getInputFlag();
			uint16_t limsw = fpga_getLimSw();
			uint16_t limsw_reg = fpga_getLimSwFlags();

			uint16_t ind = fpga_getPult();
			uint64_t ctrl = fpga_getControls();

			uint16_t flags = fpga_getStatus();

			printf("KEY:%05x(UP:%05x) LS:%02x(REG:%02x) F:%x IND:%04x CTRL:%04x%08x\n",\
					(unsigned)keys, (unsigned)up, (unsigned)limsw, (unsigned)limsw_reg, (unsigned)flags, (unsigned)ind, (unsigned)(ctrl>>32), (unsigned)ctrl);
		}
	}
	else { // initial
		tic = HAL_GetTick();
		enable = TRUE;
	}
}

void adc_task() {
	static uint32_t tic, toc;
	static BOOL enable;

	if (enable) {
		toc = HAL_GetTick();

		if (toc - tic >= TEST_PERIOD) {
			tic += TEST_PERIOD;

			fpga_adcSnapshot();
			printf("ADC");
			for (int i = 0; i < ADC_NUM; i++) {
				uint16_t adc = fpga_getADC(i);

				printf(" %03x(%d)", (int)adc, (int)adc);
			}
			printf("\n");
		}
	}
	else { // initial
		tic = HAL_GetTick();
		enable = TRUE;
	}
}

//void hv_test_task(double width, double gap, uint8_t code) {
//	if (width < 0) width = 0;
//	if (gap < 0) gap = 0;
//	if (voltage < 0) voltage = 0;
//
//	uint32_t w = (uint32_t)round(width * FPGA_CLOCK);
//	uint32_t g = (uint32_t)round(gap * FPGA_CLOCK);
//}
