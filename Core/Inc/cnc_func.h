#ifndef INC_CNC_FUNC_H_
#define INC_CNC_FUNC_H_

#include "my_types.h"

double fmm_to_steps(double value, double scale);
int32_t mm_to_steps(double value, double scale);

double fsteps_to_mm(double value, double scale);
double steps_to_mm(int32_t value, double scale);
double steps_to_mm2(int32_t nx, int32_t ny, const scale_t* const scale_xy);

fpoint_t steps_to_fpoint_mm(const point_t* const mtr_pt, const scale_t* const scale);
point_t fpoint_mm_to_steps(const fpoint_t* const pt_mm, const scale_t* const scale);

double speed_to_period(double F);
double period_to_speed(double T);
float period_to_ums(float T);

double mmm_to_mmclock(double F);
double mmclock_to_mmm(double F);

double mmm_to_ums(double mmm);

double iflength(int32_t nx, int32_t ny);

float speed_range(float value);
double T_range(double value);

#endif /* INC_CNC_FUNC_H_ */
