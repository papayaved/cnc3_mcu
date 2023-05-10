#ifndef INC_CNC_PARAM_H_
#define INC_CNC_PARAM_H_

#include "my_types.h"
#include "defines.h"
#include "feedback.h"

#define VNOM (100.0) // V
#define ACC ( 1.0 * COE_UMSEC2_TO_MMTICK2 ) // 100 um/sec2/V
#define DEC ( 1.0 * COE_UMSEC2_TO_MMTICK2 ) // 100 um/sec2/V

float velocity(float v0, float acc, float d);

void cnc_resetParam();

void cnc_setStep(double value);
double cnc_step();

void cnc_setAcc(float value);
void cnc_setDec(float value);

float cnc_getAcc();
float cnc_getDec();

float cnc_acc();
float cnc_dec();

float cnc_pidAcc();
float cnc_pidDec();

const scale_t* cnc_scaleXY();
double cnc_scaleX();
double cnc_scaleY();
double cnc_getScale(AXIS_T axis);

const scale_t* cnc_scaleUV();
double cnc_scaleU();
double cnc_scaleV();

const scale_t* cnc_scaleEnc();
double cnc_scaleEncX();
double cnc_scaleEncY();

void cnc_setScaleX(double vlaue);
void cnc_setScaleY(double vlaue);
void cnc_setScaleU(double vlaue);
void cnc_setScaleV(double vlaue);
void cnc_setScaleEncX(double vlaue);
void cnc_setScaleEncY(double vlaue);

//
double cnc_ratioStepsEncX();
double cnc_ratioStepsEncY();

double cnc_steps2mmX(int32_t cnt);
double cnc_steps2mmY(int32_t cnt);
double cnc_steps2mmU(int32_t cnt);
double cnc_steps2mmV(int32_t cnt);
double cnc_enc2mmX(int32_t cnt);
double cnc_enc2mmY(int32_t cnt);

int32_t cnc_mm2StepsX(double value);
int32_t cnc_mm2StepsY(double value);
int32_t cnc_mm2StepsU(double value);
int32_t cnc_mm2StepsV(double value);

#endif /* INC_CNC_PARAM_H_ */
