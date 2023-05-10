#include "gcmd.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <ctype.h>
#include <fpga_gpo.h>
#include <math.h>
#include <limits.h>

#include "my_lib.h"
#include "aux_func.h"

void gcmd_clear(gcmd_t* const cmd) {
	cmd->type = GTYPE_EMPTY;
	cmd->valid.data = 0;
}

BOOL gcmd_empty(const gcmd_t* const cmd) { return cmd->valid.data == 0; }

BOOL gcmd_isGMValid(double G) { return G >= 0 && G <= UINT8_MAX; }
BOOL gcmd_hasGCoord(int G) { return (G >= 0 && G <= 3) || G == 92; }

BOOL gcmd_setG(gcmd_t* const cmd, double value) {
	if (gcmd_isGMValid(value) && (cmd->type == GTYPE_EMPTY || cmd->type == GTYPE_G) && !cmd->valid.flag.G) {
		cmd->type = GTYPE_G;
		cmd->valid.flag.G = 1;
		cmd->G = (uint8_t)value;
		return TRUE;
	}
	return FALSE;
}

BOOL gcmd_setG2(gcmd_t* const cmd, double value) {
	if (gcmd_isGMValid(value) && (cmd->type == GTYPE_EMPTY || cmd->type == GTYPE_G) && !cmd->valid.flag.G2) {
		cmd->type = GTYPE_G;
		cmd->valid.flag.G2 = 1;
		cmd->G2 = (uint8_t)value;
		return TRUE;
	}
	return FALSE;
}

BOOL gcmd_setM(gcmd_t* const cmd, double value) {
	if (gcmd_isGMValid(value) && cmd->type == GTYPE_EMPTY) {
		cmd->type = GTYPE_M;
		cmd->valid.flag.M = 1;
		cmd->G = (uint8_t)value;
		return TRUE;
	}
	return FALSE;
}

BOOL gcmd_setX(gcmd_t* const cmd, double value) {
	if (cmd->type == GTYPE_EMPTY || cmd->type == GTYPE_G) {
		cmd->valid.flag.X = 1;
		cmd->X = value;
		return TRUE;
	}
	return FALSE;
}
BOOL gcmd_setY(gcmd_t* const cmd, double value) {
	if (cmd->type == GTYPE_EMPTY || cmd->type == GTYPE_G) {
		cmd->valid.flag.Y = 1;
		cmd->Y = value;
		return TRUE;
	}
	return FALSE;
}

BOOL gcmd_setP(gcmd_t* const cmd, double value) {
//	if (cmd->type == GTYPE_EMPTY || cmd->type == GTYPE_M) {
		cmd->valid.flag.P = 1;
		cmd->X = value;
		return TRUE;
//	}
//	return FALSE;
}
BOOL gcmd_setQ(gcmd_t* const cmd, double value) {
//	if (cmd->type == GTYPE_EMPTY || cmd->type == GTYPE_M) {
		cmd->valid.flag.Q = 1;
		cmd->Y = value;
		return TRUE;
//	}
//	return FALSE;
}

BOOL gcmd_setI(gcmd_t* const cmd, double value) {
	if (!cmd->valid.flag.R) {
		cmd->valid.flag.I = 1;
		cmd->I = value;
		return TRUE;
	}
	return FALSE;
}
BOOL gcmd_setJ(gcmd_t* const cmd, double value) {
	if (!cmd->valid.flag.R) {
		cmd->valid.flag.J = 1;
		cmd->J = value;
		return TRUE;
	}
	return FALSE;
}
BOOL gcmd_setR(gcmd_t* const cmd, double value) {
	if (!cmd->valid.flag.I && !cmd->valid.flag.J) {
		cmd->valid.flag.R = 1;
		cmd->I = value;
		return TRUE;
	}
	return FALSE;
}

void gcmd_setU(gcmd_t* const cmd, double value) { cmd->valid.flag.U = 1; cmd->U = value; }
void gcmd_setV(gcmd_t* const cmd, double value) { cmd->valid.flag.V = 1; cmd->V = value; }

BOOL gcmd_setI2(gcmd_t* const cmd, double value) {
	if (!cmd->valid.flag.R2) {
		cmd->valid.flag.I2 = 1;
		cmd->I2 = value;
		return TRUE;
	}
	return FALSE;
}
BOOL gcmd_setJ2(gcmd_t* const cmd, double value) {
	if (!cmd->valid.flag.R2) {
		cmd->valid.flag.J2 = 1;
		cmd->J2 = value;
		return TRUE;
	}
	return FALSE;
}
BOOL gcmd_setR2(gcmd_t* const cmd, double value) {
	if (!cmd->valid.flag.I2 && !cmd->valid.flag.J2) {
		cmd->valid.flag.R2 = 1;
		cmd->I2 = value;
		return TRUE;
	}
	return FALSE;
}

void gcmd_setF(gcmd_t* const cmd, double value) { cmd->valid.flag.F = 1; cmd->F = value; }

BOOL gcmd_isXY(const gcmd_t* const cmd) { return cmd->valid.flag.X || cmd->valid.flag.Y || cmd->valid.flag.I || cmd->valid.flag.J || cmd->valid.flag.R; }
BOOL gcmd_isUV(const gcmd_t* const cmd) { return cmd->valid.flag.U || cmd->valid.flag.V || cmd->valid.flag.I2 || cmd->valid.flag.J2 || cmd->valid.flag.R2; }
BOOL gcmd_isXYUV(const gcmd_t* const cmd) { return gcmd_isXY(cmd) || gcmd_isUV(cmd); }

uint8_t gcmd_M(const gcmd_t* const cmd) { return cmd->G; }
double gcmd_P(const gcmd_t* const cmd) { return cmd->X; }
double gcmd_Q(const gcmd_t* const cmd) { return cmd->Y; }
double gcmd_R(const gcmd_t* const cmd) { return cmd->I; }
double gcmd_R2(const gcmd_t* const cmd) { return cmd->I2; }

void gcmd_print(const gcmd_t* const cmd) {
	if (cmd->valid.flag.M)
		printf("M%d ", (int)gcmd_M(cmd));

	if (cmd->valid.flag.G)
		printf("G%d ", (int)cmd->G);

	if (cmd->valid.flag.P) {
		decimal_t res = float2fix( gcmd_P(cmd) );
		printf("P%s%d.%03d ", res.sign ? "-" : "", res.value, res.rem);
	}

	if (cmd->valid.flag.Q) {
		decimal_t res = float2fix( gcmd_Q(cmd) );
		printf("Q%s%d.%03d ", res.sign ? "-" : "", res.value, res.rem);
	}

	if (cmd->valid.flag.X) {
		decimal_t res = float2fix(cmd->X);
		printf("X%s%d.%03d ", res.sign ? "-" : "", res.value, res.rem);
	}

	if (cmd->valid.flag.Y) {
		decimal_t res = float2fix(cmd->Y);
		printf("Y%s%d.%03d ", res.sign ? "-" : "", res.value, res.rem);
	}

	if (cmd->valid.flag.I) {
		decimal_t res = float2fix(cmd->I);
		printf("I%s%d.%03d ", res.sign ? "-" : "", res.value, res.rem);
	}

	if (cmd->valid.flag.J) {
		decimal_t res = float2fix(cmd->J);
		printf("J%s%d.%03d ", res.sign ? "-" : "", res.value, res.rem);
	}

	if (cmd->valid.flag.R) {
		decimal_t res = float2fix( gcmd_R(cmd) );
		printf("L%s%d.%03d ", res.sign ? "-" : "", res.value, res.rem);
	}

	if (cmd->valid.flag.G2)
		printf("G%d(2) ", (int)cmd->G2);

	if (cmd->valid.flag.U) {
		decimal_t res = float2fix(cmd->U);
		printf("U%s%d.%03d ", res.sign ? "-" : "", res.value, res.rem);
	}

	if (cmd->valid.flag.V) {
		decimal_t res = float2fix(cmd->V);
		printf("V%s%d.%03d ", res.sign ? "-" : "", res.value, res.rem);
	}

	if (cmd->valid.flag.I2) {
		decimal_t res = float2fix(cmd->I2);
		printf("K%s%d.%03d ", res.sign ? "-" : "", res.value, res.rem);
	}

	if (cmd->valid.flag.J2) {
		decimal_t res = float2fix(cmd->J2);
		printf("L%s%d.%03d ", res.sign ? "-" : "", res.value, res.rem);
	}

	if (cmd->valid.flag.R2) {
		decimal_t res = float2fix( gcmd_R2(cmd) );
		printf("L%s%d.%03d ", res.sign ? "-" : "", res.value, res.rem);
	}

	if (cmd->valid.flag.F) {
		decimal_t res = float2fix(cmd->F);
		printf("F%s%d.%03d ", res.sign ? "-" : "", res.value, res.rem);
	}

	if (cmd->valid.flag.PCT)
		printf("%% ");

	if (cmd->valid.flag.EoF)
		printf("EOF");

	printf("\n");
}

void gcmd_printArray(const gcmd_t cmd[], int size) {
	for (int i = 0; i < size; i++) {
		gcmd_print(&cmd[i]);

		if (cmd[i].valid.flag.EoF)
			break;
	}
}
