#include "my_types.h"
#include <stdio.h>

#include "aux_func.h"

void point_clear(point_t* const pt) {
	memset(pt, 0, sizeof(point_t));
}

void fpoint_clear(fpoint_t* const pt) {
	memset(pt, 0, sizeof(fpoint_t));
}

void path_clear(path_t* const path) {
	memset(path, 0, sizeof(path_t));
}

void dir_clear(dir_t* const dir) {
	memset(dir, 0, sizeof(dir_t));
}

void context_clear(context_t* const ctx) {
	memset(ctx, 0, sizeof(context_t));
}

void pos_clear(pos_t* const pos) {
	memset(pos, 0, sizeof(pos_t));
}

void point_print(const point_t* const pt) {
	printf("(%d, %d)\n", (int)pt->x, (int)pt->y);
}

void fpoint_print(const fpoint_t* const pt) {
	double x = pt->x * 1000; // um
	double y = pt->y * 1000;

	decimal_t x2 = float2fix(x);
	decimal_t y2 = float2fix(y);

	printf("(%s%d.%03d, %s%d.%03d)\n", x2.sign ? "-" : "", x2.value, x2.rem, y2.sign ? "-" : "", y2.value, y2.rem);
}

void print_cnc_context(const cnc_context_t* const ctx) {
	printf("pump:%x drumSt:%x wire:%x hv:%x hold:%x\n", ctx->field.pump_ena, ctx->field.drum_state, ctx->field.wire_ena, ctx->field.voltage_ena, ctx->field.hold_ena);
	printf("drumVel:%x\n", ctx->field.drum_vel);
	printf("uv:%x rev:%x enc:%x\n", ctx->field.uv_ena, ctx->field.rev, ctx->field.enc_mode);
	printf("St:%x\n", ctx->field.state);
	printf("pW:%d pR:%d hvLvl:%x I:%x\n", ctx->field.pulse_width, ctx->field.pulse_ratio, ctx->field.voltage_level, ctx->field.current_index);
	printf("ID:%d X:%d Y:%d U:%d V:%d\n", (int)ctx->field.id, (int)ctx->field.x, (int)ctx->field.y, (int)ctx->field.u, (int)ctx->field.v);
	printf("encX:%d encY:%d\n", (int)ctx->field.enc_x, (int)ctx->field.enc_y);

	double F = (1e3 * FPGA_CLOCK) / ctx->field.T; // um / sec
	decimal_t F_dec = float2fix(F);
	printf("F:%s%d.%03d\n", F_dec.sign ? "-" : "", F_dec.value, F_dec.rem);

	decimal_t st_dec = float2fix(ctx->field.step);
	printf("T:%s%d.%03d\n", st_dec.sign ? "-" : "", st_dec.value, st_dec.rem);

	printf("bkp:%x read:%x\n", ctx->field.backup_valid, ctx->field.read_valid);
}
