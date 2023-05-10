#include <stdio.h>

#include "prog_array.h"

//#include "quadspi.h"
//#include "qspi_flash.h"

#include "my_types.h"
#include "my_lib.h"
#include "aux_func.h"
#include "cnc_func.h"

__attribute__ ((section(".buffers"), used)) static volatile char data[PA_SIZE];

static pa_control_t pa = {0, 0, 0, 0, 0, PLANE_UNKNOWN};

/* Add string in tail of program array
 * return:
 * 0 - OK
 * 1 - error
 */
int pa_write(const char* string) {
	size_t len = strlen(string) + 1;

	if (pa.wraddr + len < sizeof(data)) {
		memcpy((void*)&data[pa.wraddr], string, len);
		pa.wraddr += len;
		return 0;
	}
	return -1;
}

int pa_gotoBegin() {
	pa.N = 0;
	pa.plane = PLANE_UNKNOWN;
	pa.s = NULL;

	if (pa.begin <= pa.wraddr) {
		pa.rdaddr = pa.begin;
		return 0;
	}
	return -1;
}

int pa_gotoEnd() {
	pa.N = 0;
	pa.s = NULL;

	if (pa.wraddr != 0 && data[pa.wraddr - 1] == '\0' && pa.wraddr - 1 >= pa.begin) {
		pa.rdaddr = pa.wraddr;
		return 0;
	}
	return -1;
}

int pa_init() {
	return pa_gotoBegin();
}

int pa_gotoPos(int pos) {
	pa.s = NULL;

	if (pos >= pa.begin && pos <= pa.wraddr) {
		pa.rdaddr = pos;
		return 0;
	}
	else
		return -1;
}

int pa_setWraddr(int value) {
	if (value < sizeof(data) && value >= pa.begin && value >= pa.rdaddr) {
		pa.wraddr = value;
		return 0;
	}
	return -1;
}

int pa_getWraddr() { return pa.wraddr; }
int pa_getPos() { return pa.rdaddr; }

int pa_setBegin(int value) {
	if (value < sizeof(data) && value <= pa.wraddr) {
		pa.begin = value;
		return 0;
	}
	return -1;
}

int pa_getBegin() { return pa.begin; }

const char* pa_current() {
	return (char*)data + pa.rdaddr;
}

const char* pa_next() {
	int len = pa_strlen(pa.rdaddr) + 1;
	pa.s = NULL;

	if (len > 0) {
		pa.N++;
		pa.rdaddr += len;
		return (char*)data + pa.rdaddr;
	}

	return NULL;
}

const char* pa_prev() {
	int len = pa_strlen_rev(pa.rdaddr) + 1;
	pa.s = NULL;

	if (len > 0 && pa.N > 0) {
		pa.N--;
		pa.rdaddr -= len;
		return (char*)data + pa.rdaddr;
	}

	return NULL;
}

const char* pa_goto(int str_num) {
	const char* res = NULL;
	pa.s = NULL;

	if (str_num < 0)
		str_num = 0;

	if (str_num != pa.N) {
		if (pa.N < str_num) {
			do {
				res = pa_next();
			} while (pa.N < str_num && res != NULL);
		}
		else {
			do {
				res = pa_prev();
			} while (pa.N > 0 && pa.N > str_num && res != NULL);
		}
	}
	else
		res = pa_current();

	return res;
}

static int rdaddr_reg;
static int N_reg;

static void __pa_store() {
	rdaddr_reg = pa.rdaddr;
	N_reg = pa.N;
}

static void __pa_restore() {
	pa.rdaddr = rdaddr_reg;
	pa.N = N_reg;
	pa.s = NULL;
}

const char* pa_try(int str_num) {
	const char* res;
	__pa_store();

	res = pa_goto(str_num);

	__pa_restore();
	return res;
}

int pa_strlen(int addr) {
	int i;

	if (addr >= pa.begin) {
		for (i = addr; i < pa.wraddr && data[i] != '\0'; i++);

		if (i <= pa.wraddr)
			return i - addr;
	}

	return -1;
}

// Address must pointed to index after last string
int pa_strlen_rev(int addr) {
	int i;

	if (addr == pa.begin)
		return 0;
	else {
		addr--;
		if (addr < pa.wraddr && addr > pa.begin && data[addr] == '\0') {
			addr--;
			for (i = addr; i >= pa.begin && data[i] != '\0'; i--);

			return addr - i;
		}
		else if (addr == pa.begin)
			return 0;
	}

	return -1;
}

void pa_readBytes(uint32_t addr, size_t len, uint8_t buf[], size_t N, size_t pos) {
	if (addr < PA_SIZE)
		if (addr + len <= PA_SIZE)
			memcpy(&buf[pos], (void*)&(data[addr]), len);
		else {
			memcpy(&buf[pos], (void*)&data[addr], PA_SIZE - addr);
			memset(&buf[pos], 0, addr + len - PA_SIZE);
		}
	else
		memset(&buf[pos], 0, len);
}

void pa_writeBytes(uint32_t addr, size_t len, const uint8_t buf[], size_t N, size_t pos) {
	len = pos + len <= N ? len : N - pos;

	if (addr + len <= PA_SIZE)
		memcpy((void*)&data[addr], &buf[pos], len);
}

int pa_count() {
//	return pa.rev ? pa.rdaddr - pa.begin : pa.wraddr - pa.rdaddr;
	return pa.wraddr - pa.rdaddr;
}

void pa_clear() {
	memset(&pa, 0, sizeof(pa));
}

int pa_getStrNum() { return pa.N; }

BOOL pa_getGCmd(gcmd_t* const cmd, int* const G) {
	if (!pa.s || *pa.s == '\0' || pa.s < data + pa.rdaddr || pa.s >= data + pa.wraddr)
		pa.s = pa_current();

	BOOL OK = gcode_parse(&pa.s, cmd, G);
	gcmd_print(cmd);
	return OK;
}

/*	Read frame from PA
 *  note: don't miss G0, G1, G2, G3 (no modality)
 * only M commands can be in one frame together
 */
BOOL pa_getSegment(gcmd_t* const cmd, gline_t* const gline, gline_t* const uv_gline, garc_t* const garc, garc_t* const uv_garc) {
	static gcmd_t cmd_prev;
	static fpoint_t A, A2, B, B2, C, C2;

	gline_clear(gline);
	gline_clear(uv_gline);
	garc_clear(garc);
	garc_clear(uv_garc);

	BOOL OK = pa_getGCmd(cmd, NULL);

	if (pa.plane == PLANE_UNKNOWN) {
		if (gcmd_isUV(cmd))
			pa.plane = PLANE_XYUV;
		else if (gcmd_isXY(cmd))
			pa.plane = PLANE_XY;
	}

	BOOL uv_ena = pa.plane == PLANE_XYUV;

	if (OK && cmd->valid.flag.G && cmd->G <= 3) {
		struct {
			uint8_t x:1;
			uint8_t y:1;
			uint8_t u:1;
			uint8_t v:1;
		} valid = {0,0,0,0};

		A.x = 0;
		A.y = 0;
		A2.x = 0;
		A2.y = 0;

		__pa_store();

		while (1) {
			if (pa_prev() == NULL)
				break;
			else {
				OK = pa_getGCmd(&cmd_prev, NULL); // note: for rapid work G frames must don't have any others commands

				if (OK && cmd->valid.flag.G && (cmd->G <= 3 || cmd->G == 92)) {
					if (!valid.x && cmd_prev.valid.flag.X) {
						A.x = cmd_prev.X;
						valid.x = 1;
					}
					if (!valid.y && cmd_prev.valid.flag.Y) {
						A.y = cmd_prev.Y;
						valid.y = 1;
					}
					if (!valid.u && cmd_prev.valid.flag.U) {
						A2.x = cmd_prev.U;
						valid.u = 1;
					}
					if (!valid.v && cmd_prev.valid.flag.V) {
						A2.y = cmd_prev.V;
						valid.v = 1;
					}
				}

				if ((!uv_ena && valid.x && valid.y) || (uv_ena && valid.x && valid.y && valid.u && valid.v))
					break;
			}
		}

		__pa_restore();

		B = A;
		B2 = A2;

		if (cmd->valid.flag.X)
			B.x = cmd->X;

		if (cmd->valid.flag.Y)
			B.y = cmd->Y;

		if (cmd->valid.flag.U)
			B2.x = cmd->U;

		if (cmd->valid.flag.V)
			B2.y = cmd->V;

		C = A;
		C2 = A2;

		if (cmd->valid.flag.I)
			C.x += cmd->I;

		if (cmd->valid.flag.J)
			C.y += cmd->J;

		if (cmd->valid.flag.I2)
			C2.x += cmd->I2;

		if (cmd->valid.flag.J2)
			C2.y += cmd->J2;


		switch (cmd->G) {
		case 0: case 1:
			gline->valid = TRUE;
			gline->A = A;
			gline->B = B;
			break;

		case 2: case 3:
			garc->flag.valid = 1;
			garc->flag.ccw = cmd->G == 3;
			garc->flag.R = 0;
			garc->A = A;
			garc->B = B;
			garc->C = C;
			break;

		default:
			break;
		}

		if (uv_ena && cmd->valid.flag.G2 && cmd->G2 <= 3)
			switch (cmd->G2) {
			case 0: case 1:
				uv_gline->valid = TRUE;
				uv_gline->A = A2;
				uv_gline->B = B2;
				break;

			case 2: case 3:
				uv_garc->flag.valid = 1;
				uv_garc->flag.ccw = cmd->G2 == 3;
				uv_garc->flag.R = 0;
				uv_garc->A = A2;
				uv_garc->B = B2;
				uv_garc->C = C2;
				break;

			default:
				break;
			}

		return TRUE;
	}

	return OK;
}

PLANE_T pa_plane() { return pa.plane; }

void pa_enableUV(BOOL value) {
	pa.plane = value ? PLANE_XYUV : PLANE_XY;
}

// Print program array as string array
void pa_print() {
	for (int i = pa.begin; i < pa.wraddr && i < sizeof(data); i++) {
		if (data[i] == '\0')
			printf("\n");
		else
			printf("%c", data[i]);
	}

	if (data[pa.wraddr - 1] != '\0')
		printf("\nwraddr ERROR!\n");
}

// Print current segment
void pa_printSegment() {
	static gcmd_t cmd;
	static gline_t gline, uv_gline;
	static garc_t garc, uv_garc;

	gcmd_clear(&cmd);

	while (!cmd.valid.flag.EoF) {
		BOOL OK = pa_getSegment(&cmd, &gline, &uv_gline, &garc, &uv_garc);

		if (!OK) {
			printf("Seg Error\n");
			return;
		}

		if (gline.valid)
			gline_print(&gline);
		if (garc.flag.valid)
			garc_print(&garc);
		if (uv_gline.valid)
			gline_print(&uv_gline);
		if (uv_garc.flag.valid)
			garc_print(&uv_garc);
	}
}

#ifdef PA_TEST
void pa_test() {
	const char* test[] = {
		"%",
		"G92 X0 Y0",
		"G1 Y-150",
		"G3 X50 Y-200 I50",
		"G1 X150",
		"G3 X200 Y-150 J50",
		"G1 Y-50",
		"G3 X150 Y0 I-50",
		"G1 X50",
		"G3 X0 Y-50 J-50",
		"M2",
		"%"
	};

	pa_clear();
	pa_setBegin(0);

	for (int i = 0; i < sizeof(test)/sizeof(char*); i++)
		pa_write(test[i]);

	pa_print();

	{
		decimal_t x = float2fix(10.02);
		printf("%d.%03d\n", x.value, x.rem);
	}

	{
		line_t line;
		line_init(&line, 10, 20, 30, 40, FALSE);
		line_print(&line);
	}

	{
		arc_t arc;
		arc_initCenter(&arc, 10, 10, 30, 10, 20, 10, FALSE);
		arc_print(&arc);

		arc_initCenter(&arc, 10, 10, 30, 10, 20, 10, TRUE);
		arc_print(&arc);

		arc_initCenter(&arc, 10, 10, 20, 20, 20, 10, FALSE);
		arc_print(&arc);

		arc_initCenter(&arc, 10, 10, 20, 20, 20, 10, TRUE);
		arc_print(&arc);
	}

	int N = 0;
	while (pa_goto(N++) != NULL) {
		pa_printSegment();
	}
}
#endif
