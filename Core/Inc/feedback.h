#ifndef SPEED_ADJ_H_
#define SPEED_ADJ_H_

#include "my_types.h"
#include "fpga.h"

#define _GAIN_AMP_POS(Rpos, Rpfb, Rneg, Rnfb) ( (double)Rpfb / ((double)Rpos + (double)Rpfb) * (1.0 + (double)Rnfb / (double)Rneg) )

#define _GAIN_RES_DIV(R1, R2, R3, R4) ( _GAIN_AMP_POS(R3, R4, R3, R4) / (1.0 + (double)R1 / (double)R2 + (double)R1 / ((double)R3 + (double)R4)) )

#define GAIN_RES_DIV ( _GAIN_RES_DIV(100e3, 3.01e3, 100e3, 100e3) )

//#define DIFF_GAIN (1.0 / 61.04)
//#define K_DIFF (1.638 / 100)
#define K_DIFF ( _GAIN_RES_DIV(100e3, 3.01e3, 100e3, 100e3) )
#define V_DIOD (0.167) // V

#define V_REF (4.096) // V
#define ADC_LSB (V_REF / (1<<ADC_WIDTH))

#define FB_NOM (100) // V

//#define COE_DIFF(_GAIN, _VREF, _ADC_WIDTH)			( (_VREF) / ( (_GAIN) * (1<<(_ADC_WIDTH)) ) )
//#define COE_DIV(_RH, _RL, _GAIN, _VREF, _ADC_WIDTH)	( (_VREF) * ((_RH) + (_RL)) / ( (1<<(_ADC_WIDTH)) * (_RL) * (_GAIN) ) )

#define FB_PERIOD (3)

#define COE_DCODE_TO_VOLT (ADC_LSB / K_DIFF)
#define COE_VOLT_TO_DCODE (K_DIFF / ADC_LSB)

#define FB_VOLT_TO_ADC(VOLT) ( (VOLT * K_DIFF - V_DIOD) * (1.0 / ADC_LSB) )

#define ADC_HIGH	FB_VOLT_TO_ADC(80)
#define ADC_LOW		FB_VOLT_TO_ADC(70)

void fb_reset();

void fb_enableRestore();

void fb_enableForce(BOOL value);
void fb_enable(BOOL value);

BOOL fb_isEnabledForce();
BOOL fb_isEnabled();

void fb_setThld(uint16_t low, uint16_t high);
void fb_setThld_Volt(double low, double high);

uint16_t fb_lowThld();
uint16_t fb_highThld();

void fb_setRollbackSpeed(float F);
float fb_getRollbackSpeed();
double fb_Trb();

float fb_getAvarageF();
void fb_clearAvarageSpeed();
double fb_T(double Tnom, uint16_t adc);
double fb_lastT(double Tnom);

uint16_t fb_getADC10mv(uint8_t i);

void fb_setRollbackTimeout(uint32_t value);
uint32_t fb_getRollbackTimeout();

uint16_t volt_to_code(double volt);

void fb_speed_task();

#endif /* SPEED_ADJ_H_ */
