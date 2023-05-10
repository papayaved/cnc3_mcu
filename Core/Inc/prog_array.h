#ifndef INC_PROG_ARRAY_H_
#define INC_PROG_ARRAY_H_

#include "my_types.h"
#include "defines.h"
#include "gcode.h"
#include "line.h"
#include "arc.h"

#define PA_SIZE (512 * 1024)
//#define PA_TEST

//typedef enum {PLANE_UNKNOWN, PLANE_XY, PLANE_XYUV} PLANE_T;

typedef struct {
//	char data[PA_SIZE];
	const char* s;
	int begin; // 0 - don't need
	int wraddr, rdaddr;
	int N; // current string number
	PLANE_T plane;
} pa_control_t;

void pa_print();
int pa_write(const char* string);
int pa_getPos();

int pa_gotoBegin();
int pa_gotoEnd();
int pa_init();

int pa_setWraddr(int value);
int pa_getWraddr();
int pa_setBegin(int value);
int pa_getBegin();

//void pa_setRev(BOOL reverse_ena);

void pa_readBytes(uint32_t addr, size_t len, uint8_t buf[], size_t N, size_t pos);
void pa_writeBytes(uint32_t addr, size_t len, const uint8_t buf[], size_t N, size_t pos);

int pa_count();
void pa_clear();

int pa_getStrNum();

int pa_strlen(int addr);
int pa_strlen_rev(int addr);

const char* pa_goto(int str_num);
const char* pa_current(); // frame
const char* pa_next();
const char* pa_prev();

BOOL pa_getGCmd(gcmd_t* const cmd, int* const G);
BOOL pa_getSegment(gcmd_t* const cmd, gline_t* const gline, gline_t* const uv_gline, garc_t* const garc, garc_t* const uv_garc);

PLANE_T pa_plane();
void pa_enableUV(BOOL value);

void pa_test();

#endif /* INC_PROG_ARRAY_H_ */
