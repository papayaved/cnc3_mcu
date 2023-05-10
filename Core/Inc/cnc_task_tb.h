#ifndef INC_CNC_TASK_TB_H_
#define INC_CNC_TASK_TB_H_

#include "my_types.h"

void cnc_task_tb();
BOOL cnc_test_rev();
int32_t ms_to_tick(double value);
void cnc_test_direct();
void cnc_direct_test_task();
void cnc_enc_cut_tb();

#endif /* INC_CNC_TASK_TB_H_ */
