#ifndef LED_TASK_H_
#define LED_TASK_H_

#define LED_PERIOD (500)
#define LED_HALF_PERIOD (LED_PERIOD / 2)

void led_task();
void led_reset();

#endif /* LED_TASK_H_ */
