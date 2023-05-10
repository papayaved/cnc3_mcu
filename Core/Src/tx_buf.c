#include "usbd_cdc_if.h"
#include "tx_buf.h"
#include "aux_func.h"

uint8_t tx_buf[TX_BUF_SIZE];

uint8_t tx_wrack(uint32_t addr, uint8_t len) {
	write_u32_rev(tx_buf, sizeof(tx_buf), 0, CMD_WRITE<<28 | (addr & 0x0FFFFFFF));
	tx_buf[4] = len;

	uint32_t tx_crc = crc32(tx_buf, 5);
	write_u32_rev(tx_buf, sizeof(tx_buf), 5, tx_crc);

	return CDC_Transmit_FS(tx_buf, 9);
}

uint8_t tx_rdack(COMMAND_T cmd, uint32_t addr, uint8_t len) {
	write_u32_rev(tx_buf, sizeof(tx_buf), 0, cmd<<28 | (addr & 0x0FFFFFFF));
	tx_buf[4] = len;

	uint32_t tx_crc = crc32(tx_buf, 5 + len);
	write_u32_rev(tx_buf, sizeof(tx_buf), 5 + len, tx_crc);

	return CDC_Transmit_FS(tx_buf, len + 9);
}

uint8_t tx_readRegsAck(uint32_t addr, uint8_t len) {
	return tx_rdack(CMD_READ, addr, len);
}

uint8_t tx_readRegsAckBurst(uint32_t addr, uint8_t len) {
	return tx_rdack(CMD_BURST_READ, addr, len);
}

uint8_t tx_readFifoAck(uint32_t addr, uint8_t len) {
	return tx_rdack(CMD_READ_FIFO, addr, len);
}

void tx_error(COMMAND_T cmd, uint32_t addr) {
	write_u32_rev(tx_buf, sizeof(tx_buf), 0, CMD_ERROR<<28 | (addr & 0x0FFFFFFF));
	tx_buf[4] = cmd;

	uint32_t tx_crc = crc32(tx_buf, 5);
	write_u32_rev(tx_buf, sizeof(tx_buf), 5, tx_crc);

	CDC_Transmit_FS(tx_buf, 9);
}
