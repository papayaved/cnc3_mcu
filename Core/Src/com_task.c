#include "my_wdt.h"
#include <stdio.h>
#include <usbd_def.h>

#include "my_types.h"
#include "cdc_rx_task.h"
#include "aux_func.h"
#include "rx_buf.h"
#include "tx_buf.h"
#include "rw_ad.h"
#include "prog_array.h"

static BOOL burst_read; // Flag, burst reading in progress
static uint32_t burst_rdaddr; // Current burst read address
static uint32_t burst_size; // Remained data size in the burst read mode
static uint32_t burst_cnt; // Sent data in the burst read mode, that is used for debug printing only
static volatile BOOL soft_wdt_clear_req; // Request to clear the software watchdog timer that sent from the burst write interrupt

// reset burst reading
void burst_read_reset() {
	burst_read = FALSE;
	burst_rdaddr = 0;
	burst_size = 0;
	burst_cnt = 0;
}

// Send an error message
static void __error(COMMAND_T cmd, uint32_t addr) {
	tx_error(cmd, addr);
	rx_buf_rdack();
	rx_fifo_flush();
	burst_read_reset();
}

//#define READ_MAX ( (256 - 9) & 0xFC ) // align by 4 bytes
#define READ_MAX (252) // align by 4 bytes

/*
 * This task implements the received commands except the burst write into the program array
 * because the burst writing should delay USB traffic.
 */
void com_task() {
	size_t size = rx_buf_size();

	if (soft_wdt_clear_req) {
		soft_wdt_clear_req = FALSE;
		soft_wdt_clear();
	}

	if (size) {
		soft_wdt_clear();

		COMMAND_T cmd = rx_buf_cmd();
		uint32_t addr = rx_buf_addr();
		size_t len = rx_buf_data_size();
		const uint8_t* const bytes = (uint8_t*)rx_buf.array;

//		printf("%d %x %x %d\n", (int)size, (int)cmd, (int)addr, (int)len);

		switch (cmd) {
		case CMD_WRITE: //case CMD_WRITE_BURST:
			if (size == len + 9) {
				uint32_t calc_crc = crc32(bytes, len + 5);
				uint32_t rx_crc = read_u32_rev(bytes, size, len + 5);

				if (calc_crc == rx_crc) {
					ad_writeRegs(addr, len, bytes, sizeof(rx_buf.array), cmd == CMD_BURST_WRITE);
					rx_buf_rdack();
				}
				else {
//					printf("A%x L%d:\n", (int)addr, (int)len);
//					printf("CRC ERR %x (exp %x)\n", (int)rx_crc, (int)calc_crc);
					__error(cmd, addr);
				}
			}
			else
				__error(cmd, addr);

			break;

		case CMD_READ: case CMD_READ_FIFO:
			if (size == 9) {
				uint32_t calc_crc = crc32(bytes, 5);
				uint32_t rx_crc = read_u32_rev(bytes, size, 5);

				if (calc_crc == rx_crc) {
					cmd == CMD_READ ? ad_readRegs(addr, len, FALSE) : ad_readFifo(addr, len);
					rx_buf_rdack();
				}
				else {
//					printf("C%d A%x L%d\n", rdevent.cmd, (int)rdevent.addr, (int)rdevent.len);
//					printf("CRC ERR %x (exp %x)\n", (int)rx_crc, (int)calc_crc);
					__error(cmd, addr);
				}
			}
			else
				__error(cmd, addr);

			break;

		case CMD_BURST_READ:
			if (size == 13) {
				uint32_t calc_crc = crc32(bytes, 9);
				uint32_t rx_crc = read_u32_rev(bytes, size, 9);

				if (calc_crc == rx_crc) {
					burst_rdaddr = addr;
					burst_size = read_u32(bytes, size, 5);
					burst_read = burst_size != 0 && burst_size <= PA_SIZE;
					burst_cnt = 0;
					rx_buf_rdack();
				}
				else {
//					printf("C%d A%x L%d\n", rdevent.cmd, (int)rdevent.addr, (int)rdevent.len);
//					printf("CRC ERR %x (exp %x)\n", (int)rx_crc, (int)calc_crc);
					__error(cmd, addr);
				}
			}
			else
				__error(cmd, addr);

			break;

		default:
			rx_buf_rdack();
			break;
		}
	}

	// unblocked
	if (burst_read) {
		uint32_t len = burst_size;

		if (len > READ_MAX)
			len = READ_MAX;
		else
			burst_read = FALSE;

		uint8_t result = ad_readRegs(burst_rdaddr, len, TRUE);

		switch (result) {
		case USBD_OK:
			burst_cnt += len;
//			printf("%d (%d)\n", (int)burst_cnt, (int)len);

			if (burst_read) {
				burst_rdaddr += len;
				burst_size -= len;
			} else {
				burst_rdaddr = 0;
				burst_size = 0;
				burst_cnt = 0;
			}

			break;

		case USBD_BUSY:
			burst_read = TRUE;
//			printf("USBD_BUSY\n");
			break;

		default:
			burst_read_reset();
			printf("Burst read err %d\n", (int)result);
			break;
		}
	}
}

/*
* The function processes burst write commands.
* It must be used inside an USB interrupt and must delay the USB packets while running.
* The function does not send error messages.
 */
void burst_write_irq() {
	const uint8_t* const bytes = (uint8_t*)rx_buf.array;

	size_t size = rx_buf_size();
	if (!size)
		return;

	soft_wdt_clear_req = TRUE; // Packet was received. Soft watchdog timer should be clear

	COMMAND_T cmd = rx_buf_cmd();
	if (cmd != CMD_BURST_WRITE)
		return;

	uint32_t addr = rx_buf_addr();
	size_t len = rx_buf_data_size();

//	printf("%d\n", (int)len);
//	printf("%d %x %x %d\n", (int)size, (int)cmd, (int)addr, (int)len);

	if (size == len + 9) {
		uint32_t calc_crc = crc32(bytes, len + 5);
		uint32_t rx_crc = read_u32_rev(bytes, size, len + 5);

		if (calc_crc == rx_crc) {
			ad_writeBurst_irq(addr, len, bytes, sizeof(rx_buf.array));
			rx_buf_rdack(); // rx_but was read
		}
	}
}
