#ifndef INC_FLASH_PROG_ARRAY_H_
#define INC_FLASH_PROG_ARRAY_H_

#include "my_types.h"
#include "defines.h"
#include "gcode.h"
#include "line.h"
#include "arc.h"

#define PA_BUF_SIZE (256)

#define FLASH_PA_TEST (1)

typedef struct {
	uint8_t buf[PA_BUF_SIZE];
	BOOL flash_init, buf_valid;
	int wraddr, rdaddr;
	int N; // current string number
	PLANE_T plane;
} flash_prog_array_t;

size_t flash_pa_cap();
size_t flash_pa_write(const char* string);
void flash_pa_clear();
BOOL flash_pa_init();
void flash_pa_gotoBegin();

int flash_pa_count();
int flash_pa_getWraddr();
int flash_pa_getRdaddr();
int flash_pa_getStrNum();
PLANE_T flash_pa_plane();

BOOL flash_pa_setWraddr(int value);
const char* flash_pa_current();
uint8_t* flash_pa_current_rev();

BOOL flash_pa_next();
BOOL flash_pa_prev();

void flash_pa_goto(int str_num);
const char* flash_pa_try(int str_num);

int flash_buf_strlen();
int flash_buf_strlen_rev();

void flash_pa_readBytes(uint32_t addr, size_t len, uint8_t buf[], size_t N, size_t pos);
void flash_pa_writeBytes(uint32_t addr, size_t len, uint8_t buf[], size_t N, size_t pos);

BOOL flash_pa_erase(uint32_t addr);
void flash_pa_writeBytesStream(uint32_t addr, size_t len, uint8_t buf[], size_t N, size_t pos);

BOOL flash_pa_getGFrame(gcmd_t* const cmd);
BOOL flash_pa_getSegment(gcmd_t* const cmd, gline_t* const gline, gline_t* const uv_gline, garc_t* const garc, garc_t* const uv_garc);

void flash_pa_print();
void flash_pa_printSegment();

void flash_pa_test();

#endif /* INC_FLASH_PROG_ARRAY_H_ */
