#ifndef INC_MOVING_AVG_H_
#define INC_MOVING_AVG_H_

#define MOVING_AVG_LEN (1000)

void mavg_clear();
void mavg_add(float value);
int mavg_empty();
int mavg_valid();
int mavg_ready(float lim);
float mavg_get();

void mavg_tb();

#endif /* INC_MOVING_AVG_H_ */
