#include "cnc_task_tb.h"
#include "cnc_task.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <ctype.h>
#include <fpga_gpo.h>
#include <math.h>
#include <limits.h>
#include "my_wdt.h"

#include "cnc_task.h"
#include "key_task.h"
#include "line.h"
#include "arc.h"
#include "fpga.h"
#include "fpga_gpo.h"
#include "controls.h"
#include "step_dir.h"
#include "imit_fifo.h"
#include "prog_array.h"
#include "my_lib.h"
#include "aux_func.h"
#include "cnc_func.h"
#include "uv.h"
#include "context.h"
#include "backup.h"
#include "feedback.h"
#include "encoder.h"
#include "pid.h"
#include "center.h"
#include "cnc_param.h"
#include "center.h"
#include "enc_recalc_pos.h"

void cnc_task_tb() {
#ifndef RB_TEST
#if TEST_NUM == 0
	const char* test[] = {
		"%",
		"G92 X0 Y0",
//		"G01 X0.003 Y0.003",
//		"G01 X0.003 Y-0.003",
//		"G01 X-0.003 Y-0.003",
		"G01 X-0.003 Y0.003",
		"M02",
		"%"
	};
#elif TEST_NUM == 1
	const char* test[] = {
		"%",
		"G92 X0 Y0",
//		"G01 X0.003 Y0.001",
//		"G01 X0.001 Y-0.003",
//		"G01 X-0.001 Y-0.003",
		"G01 X0.001 Y0.003",
		"M02",
		"%"
	};
#elif TEST_NUM == 2
	// square with 45 degrees sides
	const char* test[] = {
		"%",
		"G92 X0 Y0",
		"G01 X0.003 Y0.003",
		"G01 X0.006 Y0.000",
		"G01 X0.003 Y-0.003",
		"G01 X0.000 Y0.000",
		"M02",
		"%"
	};
#elif TEST_NUM == 3
	// square with 90 degrees sides
	const char* test[] = {
		"%",
		"G92 X0 Y0",
		"G01 Y0.003",
		"G01 X0.003",
		"G01 Y0.000",
		"G01 X0.000",
		"M02",
		"%"
	};
#elif TEST_NUM == 4
	// square
	const char* test[] = {
		"%",
		"G92 X0 Y0",
		"G1 X0.006 Y0.002 F0.060",
		"G1 X0.008 Y-0.004",
		"G1 X0.002 Y-0.006",
		"G1 X0 Y0",
		"M2",
		"%"
	};
#elif TEST_NUM == 5
	const char* test[] = {
		"%",
		"G92 X0 Y0",
		"G2 X0.010 Y-0.010 J-0.010",
		"G2 X0 Y-0.020 I-0.010",
		"G2 X-0.010 Y-0.010 J0.010",
		"G2 X0 Y0 I0.010",
		"M02",
		"%"
	};
#elif TEST_NUM == 6
	const char* test[] = {
		"%",
		"G92 X0 Y0",
		"G3 X-0.010 Y-0.010 J-0.010",
		"G3 X0 Y-0.020 I0.010",
		"G3 X0.010 Y-0.010 J0.010",
		"G3 X0 Y0 I-0.010",
		"M02",
		"%"
	};
#elif TEST_NUM == 7
	const char* test[] = {
		"%",
		"G92 X0 Y0",
		"G3 X0 Y0.020 J0.010",
		"G3 X0 Y0 J-0.010",
		"M02",
		"%"
	};
#elif TEST_NUM == 8
	const char* test[] = {
		"%",
		"G92 X0 Y0",
		"G2 X0 Y0.020 J0.010",
		"G2 X0 Y0 J-0.010",
		"M02",
		"%"
	};
#elif TEST_NUM == 9
	const char* test[] = {
		"%",
		"G92 X0 Y0",
		"G2 X0 Y0 J0.010",
		"M02",
		"%"
	};
#elif TEST_NUM == 10
	const char* test[] = {
		"%",
		"G92 X0 Y0",
		"M40", // pump ena
		"G4 P30000", // 1 sec
		"M105 P3 M82", // drum vel, drum ena
		"G1 X0.003 Y0.003",
//		"G01 X0.003 Y-0.003",
//		"G01 X-0.003 Y-0.003",
//		"G01 X-0.003 Y0.003",
		"M2",
		"%"
	};
#elif TEST_NUM == 11
	const char* test[] = {
		"%",
		"G92 X0 Y0",
		"G1 X0.010 Y0",
		"G2 X0.020 Y0.010 I0.010",
		"M02",
		"%"
	};
#elif TEST_NUM == 12
	const char* test[] = {
		"%",
		"M106 P150 Q50",
		"M107 P30",
		"G92 X0 Y0 U0 V0",
		"G2 X0.010 Y-0.010 J-0.010 G1 U0.010 V-0.010",
		"G2 X0 Y-0.020 I-0.010 G1 U0 V-0.020",
		"G2 X-0.010 Y-0.010 J0.010 G1 U-0.010 V-0.010",
		"G2 X0 Y0 I0.010 G1 U0 V0",
		"M02",
		"%"
	};
#elif TEST_NUM == 13
	const char* test[] = {
			"%",
			"M106 P250 Q50",
			"M107 P30",
			"G92 X0 Y0 U0 V0",
			"G1 X0.010 Y0 G1 U0.011 V0",
			"G2 X0.020 Y0.010 I0.010 J0 G2 U0.020 V0.009 I0.009 J0",
			"M02",
			"%"
	};
#else
	const char* test[] = {
		"G-code",
		"G92 X0 Y0",
		"G01 X0.3 Y0.1",
//		"G01 X0.04 Y-0.02",
//		"G01 X0.01 Y-0.03",
//		"G01 X0.000 Y0.000",
		"M02"
	};
#endif
#else
#if RB_TEST == 0
	const char* test[] = {
		"%",
		"G92 X0 Y0",
		"G1 X0.01 Y0.01",
		"G1 X0.02 Y0",
		"M2",
		"%"
	};

	cnc_setRollbackLength(0.015);
#endif
#endif

	step_setImitEna(TRUE);
	pa_clear();

#ifdef TEST_REVERSE
	rev_req = TRUE;
#endif

	pa_setBegin(0);

	for (int i = 0; i < sizeof(test)/sizeof(char*); i++)
		pa_write(test[i]);

	pa_print();

	fb_enable(FALSE);
//	fpga_setInputLevel(0x300);
	fpga_setInputLevel(0);
	cnc_runReq();
}

BOOL cnc_test_rev() {
	static arc_t arc;
	static line_t uv_line;
	const double step = 1;
	const fpoint_t A = {10,0}, B = {25,15}, C = {25,0};

	uv_setL( 150 );
	uv_setH( 50 );
	uv_setT( 30 );
	uv_enableLHT(TRUE);

	arc_initCenter(&arc, &A, &B, &C, FALSE, step, cnc_scaleXY());
	line_init(&uv_line, &A, &B, FALSE, step);
	line_setStepRatio(&uv_line, arc_length(&arc));

	BOOL is_last, valid;
	size_t step_id = 3108;

	fpoint_t xy_mm = arc_getPoint(&arc, step_id, &is_last, &valid);
	fpoint_t uv_mm = line_getPoint(&uv_line, step_id, &is_last, &valid);

	fpoint_t mtr_mm = uv_XY_to_motors(&xy_mm, &uv_mm);
	fpoint_t mtr_uv_mm = uv_UV_to_motors(&xy_mm, &uv_mm);

	// Reverse
	// Read and init
	arc_initCenter(&arc, &A, &B, &C, FALSE, step, cnc_scaleXY());
	line_init(&uv_line, &A, &B, FALSE, step);
	line_setStepRatio(&uv_line, arc_length(&arc));

	arc_swap(&arc);
	line_swap(&uv_line);

	fpoint_t xy_mm2 = uv_motors_to_XY(&mtr_mm, &mtr_uv_mm);
	step_id = arc_getPos(&arc, &xy_mm2, cnc_scaleXY(), &is_last);

	fpoint_t rev_xy_mm = arc_getPoint(&arc, step_id, &is_last, &valid);
	fpoint_t rev_uv_mm = line_getPoint(&uv_line, step_id, &is_last, &valid);

	fpoint_t rev_mtr_mm = uv_XY_to_motors(&rev_xy_mm, &rev_uv_mm);
	fpoint_t rev_mtr_uv_mm = uv_UV_to_motors(&rev_xy_mm, &rev_uv_mm);

	return rev_mtr_mm.x == mtr_mm.x && rev_mtr_uv_mm.x == mtr_uv_mm.x;
}

int32_t ms_to_tick(double value) {
	value *= (FPGA_CLOCK / 1000);

	if (value >= (int32_t)INT32_MAX)
		return (int32_t)INT32_MAX;

	if (value <= (int32_t)INT32_MIN)
		return (int32_t)INT32_MIN;

	return (int32_t)value;
}

extern int32_t param_reg[2*MOTORS]; // motors + encoders
extern uint8_t direct_valid;

void cnc_test_direct() {
	fb_enable(FALSE);
	fpga_setSoftPermit(TRUE);
	fpga_setInputLevel(0x000);
	fpga_clearLimSwFlags(0xFFFF);
	fpga_setMotorOE(TRUE); // Hold Off

	uint16_t res = fpga_getRun();
	printf("RUN:%04x\n", res);

	uint16_t limsw_reg = fpga_getLimSwFlags();
	printf("limsw_reg:%02x\n", limsw_reg);

	while (limsw_reg) {
		fpga_clearLimSwFlags(limsw_reg);
		limsw_reg = fpga_getLimSwFlags();
		printf("limsw_reg:%02x\n", limsw_reg);
	}

	uint16_t adc_state = fpga_getADCStatus();
	printf("adc_state:%x\n", adc_state);

	param_reg[0] = 10;
	param_reg[1] = ms_to_tick(1000);
	param_reg[2] = 20;
	param_reg[3] = ms_to_tick(1000);
	param_reg[4] = 6;
	param_reg[5] = ms_to_tick(1000);
	param_reg[6] = 12;
	param_reg[7] = ms_to_tick(1000);

	direct_valid = 0xff;
	cnc_reqG1();

	res = fpga_getRun();
	printf("RUN:%04x\n", res);

	limsw_reg = fpga_getLimSwFlags();
	printf("limsw_reg:%04x\n", limsw_reg);

	adc_state = fpga_getADCStatus();
	printf("adc_state:%x\n", adc_state);

	cnc_printState();
}

void cnc_direct_test_task() {
	static BOOL init = FALSE;
	static int i = 0;

	if (!init) {
		init = TRUE;
		fb_enable(FALSE);
		fpga_setInputLevel(0xFFFF);
		fpga_clearLimSwFlags(0xFFFF);
		fpga_setSoftPermit(TRUE);
	}

	uint16_t res = fpga_getRun();
	printf("RUN:%04x\n", res);

	uint16_t limsw_reg = fpga_getLimSwFlags();
	printf("limsw_reg:%04x\n", limsw_reg);

	uint16_t adc_state = fpga_getADCStatus();
	printf("adc_state:%x\n", adc_state);

	cnc_printState();

	memset(param_reg, 0, sizeof(param_reg));

	param_reg[2*i] = 1;
	param_reg[2*i + 1] = T_MIN;

	if (++i > 3)
		i = 0;

	direct_valid = 0xff;
	cnc_reqG1();
}

void cnc_enc_cut_tb() {
	const char* test[] = {
		"%",
		"G92 X0 Y0",
		"G01 X0.050 Y0.025",
		"G01 X0.025 Y0.075",
		"G01 X-0.025 Y0.050",
		"G01 X0.000 Y0.000",
		"M02",
		"%"
	};

	step_setImitEna(TRUE);
	pa_clear();

#ifdef TEST_REVERSE
	rev_req = TRUE;
#endif

	pa_setBegin(0);

	for (int i = 0; i < sizeof(test)/sizeof(char*); i++)
		pa_write(test[i]);

	pa_print();

	fb_enable(FALSE);
	enc_setEncMode(TRUE);
	fpga_setInputLevel(0x300); // Debug
//	fpga_setInputLevel(0);
	cnc_runReq();
}
