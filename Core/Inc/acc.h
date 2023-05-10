#ifndef INC_ACC_H_
#define INC_ACC_H_

#include "my_types.h"

#define ACC_MIN (10.0f) // um/sec^2
#define ACC_MAX (300.0f) // um/sec^2

void acc_reset();
void acc_enable(BOOL ena);
BOOL acc_enabled();

void acc_setAcc(float acc);
void acc_setDec(float dec);
float acc_getAcc();
float acc_getDec();

double acc_acc(double T, double Tnom, float dL);

BOOL acc_brake(float T, float rem);
double acc_dec(double T, float rem, float dL);

#endif /* INC_ACC_H_ */
