#ifndef CDC_RX_TASK_H_
#define CDC_RX_TASK_H_

//extern volatile BOOL uart_reset;

void cdc_rx_task();

void cdc_rx_irq();
void cdc_rx_timer_task();

//void flush_rx();

#endif /* CDC_RX_TASK_H_ */
