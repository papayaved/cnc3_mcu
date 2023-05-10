#include "my_wdt.h"
#include "stm32h7xx_hal.h"
#include "led_task.h"
#include "my_types.h"
#include "sem_led.h"
#include "fpga.h"
#include "encoder.h"
#include <stdio.h>

static uint8_t cnt;
static uint32_t tic, toc;

void led_reset() {
	cnt = 0;
	tic = toc = 0;
}

//void ledEnable0(BOOL ena) {
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, ena);
//}
//
//void ledEnable1(BOOL ena) {
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, ena);
//}

void ledEnable0(BOOL ena) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, !ena);
}

void ledEnable1(BOOL ena) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, !ena);
}

void adc_test();
void enc_test();

void led_task() {
	toc = HAL_GetTick();

	if (toc - tic >= LED_HALF_PERIOD) {
		ledEnable0( ((cnt & 1) != 0) || hard_wdt());
		ledEnable1( ((cnt & 2) != 0) || soft_wdt());
		cnt++;
		tic += LED_HALF_PERIOD;

//		sem_next(); // semaphore test
//		adc_test();
//		enc_test();
	}
}

void adc_print() {
	printf("ADC");
	for (int i = 0; i < ADC_NUM; i++) {
		uint16_t adc = fpga_getADC(i);
		printf(" %03x", (int)adc);
	}
	printf("\n");
}

void adc_test() {
	static int cnt;
	static BOOL first = TRUE;

	if ((cnt & 0xf) == 0) {
		if (first) {
			adc_print();
			first = FALSE;
		}
		else {
			fpga_adcSnapshot();
			adc_print();
		}
	}
	cnt++;
}

void enc_test() {
	static int cnt;
	static BOOL first = TRUE;

	if ((cnt & 0xf) == 0) {
		if (first) {
			first = FALSE;
			enc_print();
		}
		else
			enc_print();
	}
	cnt++;
}
