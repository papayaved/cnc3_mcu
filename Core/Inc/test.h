#ifndef INC_TEST_H_
#define INC_TEST_H_
#include "my_types.h"

//#define TEST (2) // 2
//#define TEST_TASK (7)

#define TEST_PERIOD (5000)

void test();
void fpga_print_u32(uint16_t start, uint16_t end);
void fpga_print_u16(uint16_t start, uint16_t end);
void fpga_rw32(uint16_t start, uint16_t end);

void test_task();
void one_step_task(uint8_t i, double vel);
void step_by_step_task(double vel);
void step_by_step_slow_task();
void dati_task();
void dato_task(uint16_t ind, uint64_t ctrl);
void dato48_task();
void dato16_task();
void dati16_task();
void adc_task();

#endif /* INC_TEST_H_ */
