#ifndef INC_TOUCH_H_
#define INC_TOUCH_H_

#include "my_types.h"
//#include "fpga.h"

#define TOUCH_RB (0.1) // mm

void touch_init(double x, double y, uint8_t touches, double F, double F_fine, double fine_share);
void touch_reset();
int touch_next(float* const T);

point_t touch_result();
int32_t touch_res_x();
int32_t touch_res_y();
BOOL touch_valid();

uint8_t touch();
uint8_t touches();
int touch_getState();

#endif /* INC_TOUCH_H_ */
