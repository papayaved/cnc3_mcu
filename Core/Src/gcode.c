#include "gcode.h"

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

BOOL gcode_parse(const char** p_str, gcmd_t* const cmd, int* const G) {
	char ch;
	BOOL OK;
	int G_reg;
	enum {NO_AXIS, XY_AXIS, UV_AXIS} axis_ena = NO_AXIS;

	gcmd_clear(cmd);

	if (!p_str || !*p_str)
		return FALSE;

	if (G)
		if (*G >= 1 && *G <= 3)
			G_reg = *G;
		else
			G_reg = *G = -1;
	else
		G_reg = -1;

	while (1) {
		if (!gcmd_empty(cmd)) { // try next
			const char* str_reg = *p_str;

			ch = read_char(&str_reg, &OK);
			if (!OK) return FALSE;

			switch (ch) {
			case '\0': // main exit
				cmd->valid.flag.EoF = 1;
				*p_str = str_reg;
				return TRUE;
			case 'M':
				return TRUE;
			case 'G':
				{
					unsigned int g = read_uint(&str_reg, &OK);
					if (!OK) return FALSE;

					if ( !(gcmd_isXY(cmd) && !gcmd_isUV(cmd) && g >= 1 && g <= 3) ) // not UV part - incorrect code
						return TRUE;
				}
				break; // UV part
			default:
				break;
			}
		}

		ch = read_char(p_str, &OK);
		if (!OK) return FALSE;

		switch (ch) {
		case '\0': // empty frame - incorrect code
			cmd->valid.flag.EoF = 1;
			return TRUE;
		case 'G':
		{
			unsigned int g = read_uint(p_str, &OK);
			if (!OK) return FALSE;

			if (gcmd_empty(cmd))
				gcmd_setG(cmd, g);
			else if (gcmd_isXY(cmd) && !gcmd_isUV(cmd) && g >= 1 && g <= 3)
				gcmd_setG2(cmd, g);
			else
				gcmd_setG(cmd, g);

			G_reg = g;
			if (G) *G = G_reg;
		}
		break;

		case 'M':
		{
			unsigned int m = read_uint(p_str, &OK);
			if (!OK) return FALSE;

			gcmd_setM(cmd, m);
		}
			break;

		case 'P':
		{
			double res = read_double(p_str, &OK);
			if (!OK) return FALSE;

			gcmd_setP(cmd, res);
		}
			break;

		case 'Q':
		{
			double res = read_double(p_str, &OK);
			if (!OK) return FALSE;

			gcmd_setQ(cmd, res);
		}
			break;

		case 'X':
		{
			double res = read_double(p_str, &OK);
			if (!OK) return FALSE;

			axis_ena = XY_AXIS;
			gcmd_setX(cmd, res);
			gcmd_setG(cmd, G_reg);
		}
			break;

		case 'Y':
		{
			double res = read_double(p_str, &OK);
			if (!OK) return FALSE;

			axis_ena = XY_AXIS;
			gcmd_setY(cmd, res);
			gcmd_setG(cmd, G_reg);
		}
			break;

		case 'U':
		{
			double res = read_double(p_str, &OK);
			if (!OK) return FALSE;

			axis_ena = UV_AXIS;
			gcmd_setU(cmd, res);
			gcmd_setG2(cmd, G_reg);
		}
			break;

		case 'V':
		{
			double res = read_double(p_str, &OK);
			if (!OK) return FALSE;

			axis_ena = UV_AXIS;
			gcmd_setV(cmd, res);
			gcmd_setG2(cmd, G_reg);
		}
			break;

		case 'I':
		{
			double res = read_double(p_str, &OK);
			if (!OK) return FALSE;

			if (axis_ena == XY_AXIS) {
				OK = gcmd_setI(cmd, res);

				if (OK && (G_reg == 2 || G_reg == 3))
					gcmd_setG(cmd, G_reg);
			}
			else if (axis_ena == UV_AXIS) {
				OK = gcmd_setI2(cmd, res);

				if (OK && (G_reg == 2 || G_reg == 3))
					gcmd_setG2(cmd, G_reg);
			}
		}
			break;

		case 'J':
		{
			double res = read_double(p_str, &OK);
			if (!OK) return FALSE;

			if (axis_ena == XY_AXIS) {
				OK = gcmd_setJ(cmd, res);

				if (OK && (G_reg == 2 || G_reg == 3))
					gcmd_setG(cmd, G_reg);
			}
			else if (axis_ena == UV_AXIS) {
				OK = gcmd_setJ2(cmd, res);

				if (OK && (G_reg == 2 || G_reg == 3))
					gcmd_setG2(cmd, G_reg);
			}
		}
			break;

		case 'F':
		{
			double res = read_double(p_str, &OK);
			if (!OK) return FALSE;

			gcmd_setF(cmd, res);
		}
			break;

		case '%':
			cmd->valid.flag.PCT = 1;
			break;
		default: break;
		}
	}

	return FALSE;
}
