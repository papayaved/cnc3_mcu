#ifndef INC_CENTER_H_
#define INC_CENTER_H_

#include "my_types.h"
#include "fpga.h"

#define CENTER_ATTEMPTS_MAX (5)
#define CENTER_RADIUS_MAX (1000.0f) // mm

typedef enum {CENTER_NO, CENTER_X, CENTER_Y, CENTER_CIRCLE_4R, CENTER_CIRCLE_3ADJ} CENTER_MODE_T;

typedef struct {
	uint8_t touches, attempts, drum_vel;
	uint16_t thld;
	CENTER_MODE_T mode;
	double R, rollback, F, F_fine, fine_share;
	double angle[3];
} center_t;

int center_init(const center_t* const center);
int center_next(float* const T);
void center_reset();

int center_getState();
uint8_t center_attempt();
uint8_t center_attempts();

// mm
float center_D();

void center_test();

#endif /* INC_CENTER_H_ */
