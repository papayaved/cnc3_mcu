#ifndef INC_GCMD_H_
#define INC_GCMD_H_

#include "my_types.h"

typedef enum { GTYPE_EMPTY, GTYPE_G, GTYPE_M } gtype_t;

typedef union {
    uint32_t data;
    struct {
    	uint32_t G:1;
    	uint32_t X:1; // P
    	uint32_t Y:1; // Q
    	uint32_t I:1; // R
    	uint32_t J:1;
    	uint32_t R:1;

    	uint32_t G2:1;
    	uint32_t U:1;
    	uint32_t V:1;
    	uint32_t I2:1; // R2
    	uint32_t J2:1;
    	uint32_t R2:1;

    	uint32_t M:1;
    	uint32_t P:1;
    	uint32_t Q:1;
    	uint32_t F:1;

    	uint32_t PCT:1;
    	uint32_t EoF:1; // End of Frame
    } flag;
} gvalid_t;

typedef struct {
	uint8_t G, G2;
	gtype_t type;
	gvalid_t valid;
	double X, Y, I, J, U, V, I2, J2, F;
} gcmd_t;

void gcmd_clear(gcmd_t* const cmd);
BOOL gcmd_empty(const gcmd_t* const cmd);

BOOL gcmd_isGMValid(double G);
BOOL gcmd_hasGCoord(int G);

BOOL gcmd_setG(gcmd_t* const cmd, double value);
BOOL gcmd_setG2(gcmd_t* const cmd, double value);
BOOL gcmd_setM(gcmd_t* const cmd, double value);
BOOL gcmd_setX(gcmd_t* const cmd, double value);
BOOL gcmd_setY(gcmd_t* const cmd, double value);
BOOL gcmd_setP(gcmd_t* const cmd, double value);
BOOL gcmd_setQ(gcmd_t* const cmd, double value);
BOOL gcmd_setI(gcmd_t* const cmd, double value);
BOOL gcmd_setJ(gcmd_t* const cmd, double value);
BOOL gcmd_setR(gcmd_t* const cmd, double value);
void gcmd_setU(gcmd_t* const cmd, double value);
void gcmd_setV(gcmd_t* const cmd, double value);
BOOL gcmd_setI2(gcmd_t* const cmd, double value);
BOOL gcmd_setJ2(gcmd_t* const cmd, double value);
BOOL gcmd_setR2(gcmd_t* const cmd, double value);
void gcmd_setF(gcmd_t* const cmd, double value);

BOOL gcmd_isXY(const gcmd_t* const cmd);
BOOL gcmd_isUV(const gcmd_t* const cmd);
BOOL gcmd_isXYUV(const gcmd_t* const cmd);

uint8_t gcmd_M(const gcmd_t* const cmd);
double gcmd_P(const gcmd_t* const cmd);
double gcmd_Q(const gcmd_t* const cmd);
double gcmd_R(const gcmd_t* const cmd);
double gcmd_R2(const gcmd_t* const cmd);

void gcmd_print(const gcmd_t* const cmd);
void gcmd_printArray(const gcmd_t cmd[], int size);

#endif /* INC_GCMD_H_ */
