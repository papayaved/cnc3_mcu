#include <stdio.h>

#include "encoder.h"
#include "fpga.h"

uint8_t enc_flags() {
	return fpga_read_u16(ENC_FLAGS>>1) & 0xFF;
}

void enc_clearFlags(uint8_t flags) {
	fpga_write_u16(ENC_FLAGS>>1, (uint16_t)flags);
}

uint8_t enc_flagsAutoClear() {
	uint16_t flags = fpga_read_u16(ENC_FLAGS>>1);
	fpga_write_u16(ENC_FLAGS>>1, flags);
	return flags & 0xFF;
}

void enc_clear() {
	fpga_write_u16(ENC_CLEAR>>1, (uint16_t)~0U);
}

uint16_t enc_getStatus() {
	return fpga_read_u16(ENC_STATUS>>1);
}

BOOL enc_getError() {
	return fpga_read_u16(ENC_STATUS>>1) != 0;
}

void enc_enable() {
	fpga_write_u16(ENC_ENABLE>>1, (uint16_t)~0U);
}

void enc_disable() {
	fpga_write_u16(ENC_ENABLE>>1, 0);
}

uint16_t enc_isEnabled() {
	return fpga_read_u16(ENC_ENABLE>>1);
}

void enc_setDir(uint16_t value) {
	fpga_write_u16(ENC_DIR>>1, value);
}

uint16_t enc_getDir() {
	return fpga_read_u16(ENC_DIR>>1);
}

//void enc_snap() {
//	fpga_write_u16(ENC_SNAPSHOT>>1, (uint16_t)~0U);
//}

int32_t enc_get(size_t i) {
	switch (i) {
	case 0: return fpga_read_s32(ENC32_0>>2);
	case 1: return fpga_read_s32(ENC32_1>>2);
	default: return 0;
	}
}

int32_t enc_getX() { return fpga_read_s32(ENC32_0>>2); }
int32_t enc_getY() { return fpga_read_s32(ENC32_1>>2); }

void enc_getXY(int32_t* const x, int32_t* const y) {
	if (x) *x = fpga_read_s32(ENC32_0>>2);
	if (y) *y = fpga_read_s32(ENC32_1>>2);
}

void enc_set(size_t i, int32_t value) {
	enc_disable();
	switch (i) {
	case 0: fpga_write_s32(ENC32_0>>2, value); break;
	case 1: fpga_write_s32(ENC32_1>>2, value); break;
	default: break;
	}
	enc_enable();
}

void enc_setXY(int32_t x, int32_t y) {
	enc_disable();
	fpga_write_s32(ENC32_0>>2, x);
	fpga_write_s32(ENC32_1>>2, y);
	enc_enable();
}

void enc_print() {
	static BOOL init;

	if (!init) {
		init = TRUE;
		enc_clear();
		int ena = enc_isEnabled();
		int dir = enc_getDir();
		printf("ENC ena:%04x dir:%04x\n", ena, dir);
	}

	fpga_globalSnapshot();
	int x = enc_getX();
	int y = enc_getY();
	uint16_t s = enc_getStatus();
	printf("ENC x:%d y:%d s:%04x\n", (int)x, (int)y, (int)s);
}
