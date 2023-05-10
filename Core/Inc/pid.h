#ifndef INC_PID_H_
#define INC_PID_H_
#include "my_types.h"
#include "cnc_param.h"

float pid(uint16_t adc, float Tnom, float dL);

void pid_setAcc(float value);
float pid_getAcc();

void pid_setDec(float value);
float pid_getDec();

void pid_clear();
void pid_stop();
void pid_reset();

#endif /* INC_PID_H_ */
