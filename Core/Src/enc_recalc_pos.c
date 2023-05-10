#include "enc_recalc_pos.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <ctype.h>
#include <math.h>
#include <limits.h>

#include "cnc_task.h"

#include "line.h"
#include "arc.h"
#include "fpga.h"
#include "prog_array.h"
#include "my_lib.h"
#include "aux_func.h"
#include "cnc_func.h"
#include "uv.h"
#include "encoder.h"
#include "cnc_param.h"

BOOL enc_mode = FALSE; // TRUE - encoder mode is enabled

union {
	uint8_t data;
	struct {
		uint8_t x:1;
		uint8_t y:1;
		uint8_t :6;
	} flag;
} enc_reg_valid = {0}; // values "enc_reg" and "pos_enc_reg" are valid

static int32_t enc_reg[2] = {0, 0};
static int32_t pos_enc_reg[2] = {0, 0};

void enc_clearEncModeRegs() {
	enc_reg_valid.data = 0;
	enc_reg[0] = enc_reg[1] = 0;
	pos_enc_reg[0] = pos_enc_reg[1] = 0;
}

void enc_resetEncMode() {
	enc_mode = FALSE;
	enc_clearEncModeRegs();
}

void enc_setEncMode(BOOL ena) {
	if ( cnc_isIdle() ) {
		enc_mode = ena;
		enc_clearEncModeRegs();
	}
}

BOOL enc_isEncMode() { return enc_mode; }

/*
 * return - corrected step_id
 */
size_t enc_recalc_pos(point_t* const p_mtr_pt, const point_t* const p_mtr_uv_pt, const line_t* const p_line, const arc_t* const p_arc, size_t step_id, BOOL* const is_last) {
	static uint8_t enc_flags;
	static int32_t pos_x, pos_x_enc, pos_y, pos_y_enc, enc_x, enc_y;
	static int32_t d_enc_x, d_enc_y, d_pos_x_enc, d_pos_y_enc, dx, dy;

	enc_flags = enc_flagsAutoClear(); // triggered encoders

	// An encoder has a new value - recalculate step_id
	if (enc_flags != 0) {
		fpga_globalSnapshot();
		pos_x = fpga_getPosX();
		pos_x_enc = fpga_getPosXEnc(); // get a position when the encoder triggered
		dx = pos_x - pos_x_enc; // Should be 0 for a slow stepper motor

		pos_y = fpga_getPosY();
		pos_y_enc = fpga_getPosYEnc();
		dy = pos_y - pos_y_enc;

		enc_x = enc_getX();
		enc_y = enc_getY();

		d_enc_x = enc_reg_valid.flag.x ? enc_x - enc_reg[0] : 0; // Should be -1, 0, 1
		d_enc_y = enc_reg_valid.flag.y ? enc_y - enc_reg[1] : 0; // -1, 0, 1

		d_pos_x_enc = enc_reg_valid.flag.x ? pos_x_enc - pos_enc_reg[0] : 0; // Encoder step (5, -5)
		d_pos_y_enc = enc_reg_valid.flag.y ? pos_y_enc - pos_enc_reg[1] : 0;
#ifdef PRINT
		printf("ID %d Mtr (%d %d) Enc (%d %d) EncReg (%d %d) xyEnc (%d %d)\n",\
				(int)step_id, (int)p_mtr_pt->x, (int)p_mtr_pt->y, (int)enc_x, (int)enc_y, (int)enc_reg[0], (int)enc_reg[1], (int)pos_x_enc, (int)pos_y_enc);
#endif

//		BOOL cond_x = !enc_reg_valid.flag.x || ((d_enc_x && d_pos_x_enc) || abs(d_enc_x) > 1); // There were encoder and steps
//		BOOL cond_y = !enc_reg_valid.flag.y || ((d_enc_y && d_pos_y_enc) || abs(d_enc_y) > 1); // ??

		BOOL cond_x = (d_enc_x && d_pos_x_enc) || abs(d_enc_x) > 1; // There were encoder and steps
		BOOL cond_y = (d_enc_y && d_pos_y_enc) || abs(d_enc_y) > 1;

		if (cond_x || cond_y) {
			static fpoint_t mtr_mm, xy_mm;

			if ((enc_flags & 1) && cond_x) {
				mtr_mm.x = cnc_enc2mmX(enc_x) + cnc_steps2mmX(dx);
				p_mtr_pt->x = cnc_mm2StepsX(mtr_mm.x); // int??
			}
			else // use current position
				mtr_mm.x = cnc_steps2mmX(p_mtr_pt->x);

			if ((enc_flags & 2) && cond_y) {
				mtr_mm.y = cnc_enc2mmY(enc_y) + cnc_steps2mmY(dy);
				p_mtr_pt->y = cnc_mm2StepsY(mtr_mm.y);
			}
			else
				mtr_mm.y = cnc_steps2mmY(p_mtr_pt->y);

			if (pa_plane() == PLANE_XYUV) {
				fpoint_t mtr_uv_mm = steps_to_fpoint_mm(p_mtr_uv_pt, cnc_scaleUV());
				xy_mm = uv_motors_to_XY(&mtr_mm, &mtr_uv_mm);
			} else {
				xy_mm = mtr_mm;
			}

			if (p_line->valid) {
				step_id = line_getPos(p_line, &xy_mm, is_last);
			} else if (p_arc->flag.valid) {
				step_id = arc_getPos(p_arc, &xy_mm, cnc_scaleXY(), is_last);
			}
#ifdef PRINT
			printf("New (%d %d) ID %d\n", (int)p_mtr_pt->x, (int)p_mtr_pt->y, (int)step_id);
#endif
		}

		if (enc_flags & 1) {
			enc_reg[0] = enc_x;
			pos_enc_reg[0] = pos_x_enc;
			enc_reg_valid.flag.x = 1;
		}
		if (enc_flags & 2) {
			enc_reg[1] = enc_y;
			pos_enc_reg[1] = pos_y_enc;
			enc_reg_valid.flag.y = 1;
		}
	}

	return step_id;
}
