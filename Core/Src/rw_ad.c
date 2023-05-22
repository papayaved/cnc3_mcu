#include <stdio.h>

#include "stm32h7xx_hal.h"

#include "defines.h"
#include "rw_ad.h"
#include "aux_func.h"
#include "prog_array.h"
#include "cnc_task.h"
#include "fpga.h"
#include "fpga_gpo.h"
#include "tx_buf.h"
#include "imit_fifo.h"
#include "step_dir.h"
#include "debug_fifo.h"
#include "backup.h"
#include "context.h"
#include "feedback.h"
#include "encoder.h"
#include "pid.h"
#include "center.h"
#include "cnc_func.h"
#include "sem_led.h"
#include "my_wdt.h"
#include "enc_recalc_pos.h"
#include "acc.h"

void burst_read_reset();

static uint32_t test_reg = 0x12345678;
static center_t center = {0};

void reset() {
	cnc_reset();
	burst_read_reset();
	test_reg = 0x12345678;
	memset(&center, 0, sizeof(center_t));
	imit_fifo_clear();
	debug_fifo_clear();
}

void ad_writeRegs(const size_t addr, size_t len, const uint8_t buf[], const size_t N, const BOOL async) {
	uint32_t addr32;
	size_t len32;
	size_t pos = 5;
	static int bytes_cnt = 0;
//	static int cnt = 1;

	size_t len_max = N - 9;
	if (len > len_max) len = len_max;

//	printf("%x %d %x\n", addr, len, async);

	if ((addr & ADDR_MASK) == PA_BAR) {
		pa_writeBytes(addr & ~ADDR_MASK, len, buf, N, pos);

		if (!async)
			tx_wrack(addr, len);
		else {
			bytes_cnt += len;
		}

//		printf("%x %d %d %d\n", async, cnt++, bytes_cnt, len);
	} else if ((addr & ADDR_MASK) == MCU_BAR && (addr & 3) == 0 && (len & 3) == 0) {
		addr32 = (addr & ~ADDR_MASK) >> 2;
		len32 = len >> 2;

		for (int i = 0; i < len32; i++, addr32++, pos += sizeof(uint32_t)) {
			uint32_t wrdata = read_u32(buf, N, pos);

			const uint8_t* const bytes = (uint8_t*)&wrdata;
			const int32_t* const p32 = (int32_t*)&wrdata;
			const float* const pfloat = (float*)&wrdata;

			if (addr32 < 0x200)
				switch (addr32) {
					case 0:
//						printf("W: A0 %08x\n", wrdata);
						if (wrdata & 1)
							cnc_runReq();
						if (wrdata & 1<<8)
							cnc_revReq();
						if (wrdata & 1<<16)
							step_setImitEna(TRUE);

						break;
					case 1:
						if (wrdata & 1<<0)
							cnc_stopReq();
						if (wrdata & 1<<1)
							cnc_cancelReq();
						if (wrdata & 1<<3) {
							cnc_reset();
							burst_read_reset();
						}
						if (wrdata & 1<<4) {
							cnc_state_reset();
							step_writeTaskId(0); // move a string number to the begin
							burst_read_reset();
						}
						if (wrdata & 1<<16)
							step_setImitEna(FALSE);

						break;

					case 0x12: cnc_goto(wrdata); break;
					case 0x13: cnc_setParam(0, *p32); break; // x
					case 0x14: cnc_setParam(1, *p32); break; // y
					case 0x15: cnc_setParam(2, *p32); break; // u
					case 0x16: cnc_setParam(3, *p32); break; // v
					case 0x17: cnc_setParam(4, *p32); break; // enc_x
					case 0x18: cnc_setParam(5, *p32); // enc_y
						cnc_recovery();
						break;

					case 0x20: gpo_setControlsEnable((uint16_t)wrdata, (uint16_t)(wrdata>>16)); break;
					case 0x21: gpo_setDrumVel(wrdata); break;
					case 0x22: gpo_setVoltageLevel(wrdata); break;
					case 0x23: gpo_setCurrentIndex(wrdata); break;
					case 0x24: gpo_setPulseWidth(wrdata); break;
					case 0x25: gpo_setPulseRatio(wrdata); break;
					case 0x26: cnc_setSpeed(*pfloat); break;
					case 0x27: cnc_setStep(*pfloat); break;
					case 0x28: cnc_setScaleX(*pfloat); break;
					case 0x29: cnc_setScaleY(*pfloat); break;
					case 0x2A: cnc_setScaleU(*pfloat); break;
					case 0x2B: cnc_setScaleV(*pfloat); break;
					case 0x2C: cnc_setScaleEncX(*pfloat); break;
					case 0x2D: cnc_setScaleEncY(*pfloat); break;
					case 0x2E:
						 if (wrdata & 1<<16) // mask
							 enc_setEncMode(wrdata & 1); // data

						 if (wrdata & 1<<(16 + 8))
							 cnc_enableUV((wrdata & 1<<8) != 0);

						 if (wrdata & 1<<(16 + 9))
							 uv_enableRollerDia((wrdata & 1<<9) != 0);
						break;

					// Settings
					case 0x30:
						cnc_reset();
						fpga_setInputLevel((uint16_t)wrdata);
#ifdef PRINT
						printf("WR inLvl:%x\n", (int)wrdata);
#endif
						break;
					case 0x31:
						fpga_setMotorDir((uint16_t)wrdata);
						enc_setDir((uint16_t)(wrdata>>16));
						break;
					case 0x32:
						fpga_setSdOutEnable((wrdata & 1) != 0);
						fpga_setSdEnable((wrdata & 2) != 0);
						break;
					case 0x33:
						acc_enable((wrdata & 1) != 0);
						break;
					case 0x34:
						acc_setAcc(*pfloat);
						break;
					case 0x35:
						acc_setDec(*pfloat);
						break;

					case 0x38:
						fb_enable(wrdata & 1);
#ifdef PRINT
						printf("FB_ENA:%x\n", (int)fb_isEnabledForce());
#endif
						break;
					case 0x39:
						fb_setThld((uint16_t)wrdata, (uint16_t)(wrdata>>16)); // low, high
						break;
					case 0x3A:
						fb_setRollbackTimeout(wrdata);
						break;
					case 0x3B:
						cnc_setRollbackAttempts(wrdata);
						break;
					case 0x3C:
						cnc_setRollbackLength(*pfloat); // mm
						break;
					case 0x3D:
						fb_setRollbackSpeed(*pfloat); // mm/min
						break;
					case 0x3E:
						cnc_setFbAcc(*pfloat * (1.0 / VNOM)); // um/sec2/V
						break;
					case 0x3F:
						cnc_setFbDec(*pfloat * (1.0 / VNOM));
						break;

					case 0x40: cnc_setParam(0, *p32); break;
					case 0x41: cnc_setParam(1, *p32); break;
					case 0x42: cnc_setParam(2, *p32); break;
					case 0x43: cnc_setParam(3, *p32); break;
					case 0x44: cnc_setParam(4, *p32); break;
					case 0x45:
						cnc_setParam(5, *p32);

						if (cnc_isInit()) {
							cnc_reqG92();
						}
						break;

					case 0x50: cnc_setParam(0, *p32); break;
					case 0x51: cnc_setParam(1, *p32); break;
					case 0x52: cnc_setParam(2, *p32); break;
					case 0x53: cnc_setParam(3, *p32); break;
					case 0x54: cnc_setParam(4, *p32); break;
					case 0x55: cnc_setParam(5, *p32); break;
					case 0x56: cnc_setParam(6, *p32); break;
					case 0x57:
						cnc_setParam(7, *p32);

						if (cnc_isInit()) {
							cnc_reqG1();
						}
						break;

					// center function
					case 0x70:
						center.mode		= bytes[0];
						center.touches	= bytes[1];
						center.attempts	= bytes[2];
						center.drum_vel	= bytes[3];
						break;
					case 0x71:
						center.thld			= (uint16_t)(wrdata);
						center.fine_share	= bytes[2] * 0.01;
						break;
					case 0x72: center.R			= *pfloat; break;
					case 0x73: center.rollback	= *pfloat; break;
					case 0x74: center.F			= mmm_to_mmclock(*pfloat); break;
					case 0x75: center.F_fine	= mmm_to_mmclock(*pfloat); break;
					case 0x76: center.angle[0]	= *pfloat; break;
					case 0x77: center.angle[1]	= *pfloat; break;
					case 0x78:
						center.angle[2]	= *pfloat;
						cnc_centerReq(&center);
						break;

					case 0x80: LHTD
						break;

					case 0xFF: test_reg = wrdata; break;

//					case 0x100: if (!cnc_run()) pa_setBegin(wrdata); break;
					case 0x101: pa_setWraddr(wrdata); break;
//					case 0x102: if (!cnc_run()) pa_setRev(wrdata); break;

					case 0x106: if (wrdata & 1) pa_clear(); break;

					default: break;
				}
			else if (addr32 < 0x220)
				bkp_writeRegU32(addr32 & 0x1F, wrdata);
		}

		if (!gpo_getValid())
			gpo_apply();

		if (!async)
			tx_wrack(addr, len);
	}
	else if ((addr & ADDR_MASK) == FPGA_BAR && (addr & 1) == 0 && (len & 1) == 0) {
		uint32_t addr16 = (addr & ~ADDR_MASK) >> 1;
		size_t len16 = len >> 1;
		pos = 0;

		for (int i = 0; i < len16; i++, addr16++, pos += sizeof(uint16_t)) {
			uint16_t wrdata16 = read_u16(buf, N, pos);
			fpga_write_u16(addr16, wrdata16);
		}

		if (!async)
			tx_wrack(addr, len);
	}
}

/*
 * The function writes data into the program array
 * This function should be use inside an USB interrupt
 */
void ad_writeBurst_irq(const size_t addr, size_t len, const uint8_t buf[], const size_t N) {
	size_t len_max = N - 9;

	if (len > len_max)
		len = len_max;

//	printf("%x %d\n", addr, len);

	if ((addr & ADDR_MASK) == PA_BAR)
		pa_writeBytes(addr & ~ADDR_MASK, len, buf, N, 5);
}

const char desc[64] = DESC;
const uint32_t* const desc32 = (uint32_t*)desc;

const char __date__[16] = __DATE__; // Date of program compilation. Format: MMM DD YYYY. Size 12 bytes.
const char __time__[16] = __TIME__; // Time of compilation. Format: HH:MM:SS. Size 9 bytes.

const uint32_t* const __date32__ = (uint32_t*)__date__;
const uint32_t* const __time32__ = (uint32_t*)__time__;

/*
 * Function reads data and send them to USB
 * addr - first addr
 * len - length
 * burst - burst mode, sent a series of packets
 * return: USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
 */
uint8_t ad_readRegs(uint32_t addr, size_t len, BOOL burst) {
	static uint32_t rddata;
	static int32_t* const p32 = (int32_t*)&rddata;
	static float* const pfloat = (float*)&rddata;

	size_t pos = 5;

	if (len > sizeof(tx_buf) - 9) len = sizeof(tx_buf) - 9;

	if ((addr & ADDR_MASK) == PA_BAR) {
		pa_readBytes(addr & ~ADDR_MASK, len, tx_buf, sizeof(tx_buf), pos);
	}
	else if ((addr & ADDR_MASK) == MCU_BAR && (addr & 3) == 0 && (len & 3) == 0) {
		uint32_t addr32 = (addr & ~ADDR_MASK) >> 2;
		size_t len32 = len >> 2;

		for (int i = 0; i < len32; i++, addr32++, pos += sizeof(uint32_t)) {
			if (addr32 < 0x200)
				switch (addr32) {
					case 0: rddata = step_getImitEna()<<16 | cnc_isReverse()<<8 | cnc_error()<<3 | cnc_pause()<<2 | cnc_stop()<<1 | cnc_run(); break;

					case 0x10:
						cnc_ctx_getForce();
						rddata = cnc_ctx_get(0);
						break;
					case 0x11:
						rddata = cnc_ctx_get(1);
						break;
					case 0x12: // id
						rddata = cnc_ctx_get(2);
						break;
					case 0x13: // x
						rddata = cnc_ctx_get(3);
						break;
					case 0x14: // y
						rddata = cnc_ctx_get(4);
						break;
					case 0x15: // u
						rddata = cnc_ctx_get(5);
						break;
					case 0x16: // v
						rddata = cnc_ctx_get(6);
						break;
					case 0x17: // enc_x
						rddata = cnc_ctx_get(7);
						break;
					case 0x18: // enc_y
						rddata = cnc_ctx_get(8);
						break;
					case 0x19: // T
						rddata = cnc_ctx_get(9);
						break;
					case 0x1A: // T_cur
						rddata = cnc_ctx_get(10);
						break;
					case 0x1B: // step
						rddata = cnc_ctx_get(11);
						break;
					case 0x1C: // limsw, semaphore, feedback
						rddata = cnc_ctx_get(12);
						break;
					case 0x1D: // center
						rddata = cnc_ctx_get(13);
						break;

					// TODO: add all other bits
					case 0x20: rddata = (uint32_t)gpo_cncEnabled()<<7 | (uint32_t)sem_enabled()<<6; break;

					case 0x26: *pfloat = cnc_speed(); break;
					case 0x27: *pfloat = (float)cnc_step(); break;
					case 0x28: *pfloat = (float)cnc_scaleX(); break;
					case 0x29: *pfloat = (float)cnc_scaleY(); break;
					case 0x2A: *pfloat = (float)cnc_scaleU(); break;
					case 0x2B: *pfloat = (float)cnc_scaleV(); break;
					case 0x2C: *pfloat = (float)cnc_scaleEncX(); break;
					case 0x2D: *pfloat = (float)cnc_scaleEncY(); break;
					case 0x2E: rddata = (uint32_t)uv_dia_valid()<<9 | (uint32_t)cnc_uvEnabled()<<8 | (uint32_t)enc_isEncMode()<<0; break;

					// Settings
					case 0x30:
						rddata = fpga_getInputLevel();
#ifdef PRINT
						printf("RD inLvl:%x\n", (unsigned)rddata);
#endif
						break;
					case 0x31:
						rddata = (uint32_t)enc_getDir()<<16 | (uint32_t)fpga_getMotorDir();
						break;
					case 0x32:
						rddata = fpga_getSdEnable()<<1 | fpga_getSdOutEnable();
						break;
					case 0x33:
						rddata = acc_enabled();
						break;
					case 0x34:
						*pfloat = acc_getAcc();
						break;
					case 0x35:
						*pfloat = acc_getDec();
						break;

					case 0x38:
						rddata = (uint32_t)(fb_isEnabled() != 0);
						break;
					case 0x39:
						rddata = (uint32_t)fb_highThld()<<16 | (uint32_t)fb_lowThld(); // ADC bits
//						printf("RD THLD:%x %x\n", (unsigned)fpga_getHighThld(),.. (unsigned)fpga_getLowThld());
						break;
					case 0x3A:
						rddata = fb_getRollbackTimeout(); // ms
						break;
					case 0x3B:
						rddata = cnc_getRollbackAttempts();
						break;
					case 0x3C:
						*pfloat = cnc_getRollbackLength(); // mm
						break;
					case 0x3D:
						*pfloat = fb_getRollbackSpeed(); // mm/min
						break;
					case 0x3E:
						*pfloat = cnc_getFbAcc() * VNOM; // um/sec2/100V
						break;
					case 0x3F:
						*pfloat = cnc_getFbDec() * VNOM;
						break;

					case 0x40: *p32 = cnc_getParam(0); break;
					case 0x41: *p32 = cnc_getParam(1); break;
					case 0x42: *p32 = cnc_getParam(2); break;
					case 0x43: *p32 = cnc_getParam(3); break;
					case 0x44: *p32 = cnc_getParam(4); break;
					case 0x45: *p32 = cnc_getParam(5); break;

					case 0x50: *p32 = cnc_getParam(0); break;
					case 0x51: *p32 = cnc_getParam(1); break;
					case 0x52: *p32 = cnc_getParam(2); break;
					case 0x53: *p32 = cnc_getParam(3); break;
					case 0x54: *p32 = cnc_getParam(4); break;
					case 0x55: *p32 = cnc_getParam(5); break;
					case 0x56: *p32 = cnc_getParam(6); break;
					case 0x57: *p32 = cnc_getParam(7); break;

					case 0x60:
						fpga_adcSnapshot();
						rddata = (uint32_t)fpga_getADC(2)<<16 |  fpga_getADC(0); // back diff
						break;
					case 0x61:
						rddata = (uint32_t)fpga_getADC(5)<<16 | fpga_getADC(4); // wire- workpiece+
						break;
					case 0x62:
						rddata = (uint32_t)fpga_getADC(7)<<16 | fpga_getADC(6); // shunt hv+
						break;

					case 0x70: rddata = (uint32_t)center.drum_vel<<24 | (uint32_t)center.attempts<<16 | (uint32_t)center.touches<<8 | (uint32_t)(center.mode && 0xFF); break;
					case 0x71: {
						double fine_share = round(center.fine_share * 100.0);

						if (fine_share > 100.0)
							fine_share = 100;
						else if (fine_share < 0)
							fine_share = 0;

						rddata = ((uint32_t)fine_share)<<16 | (uint32_t)center.thld;
					}
						break;
					case 0x72: *pfloat = center.R; break;
					case 0x73: *pfloat = center.rollback; break;
					case 0x74: *pfloat = mmclock_to_mmm(center.F); break;
					case 0x75: *pfloat = mmclock_to_mmm(center.F_fine); break;
					case 0x76: *pfloat = center.angle[0]; break;
					case 0x77: *pfloat = center.angle[1]; break;
					case 0x78: *pfloat = center.angle[2]; break;

					case 0x7a: *pfloat = center_D(); break;

					case 0xE0: rddata = desc32[0]; break;
					case 0xE1: rddata = desc32[1]; break;
					case 0xE2: rddata = desc32[2]; break;
					case 0xE3: rddata = desc32[3]; break;
					case 0xE4: rddata = desc32[4]; break;
					case 0xE5: rddata = desc32[5]; break;
					case 0xE6: rddata = desc32[6]; break;
					case 0xE7: rddata = desc32[7]; break;
					case 0xE8: rddata = desc32[8]; break;
					case 0xE9: rddata = desc32[9]; break;
					case 0xEA: rddata = desc32[10]; break;
					case 0xEB: rddata = desc32[11]; break;
					case 0xEC: rddata = desc32[12]; break;
					case 0xED: rddata = desc32[13]; break;
					case 0xEE: rddata = desc32[14]; break;
					case 0xEF: rddata = desc32[15]; break;

					case 0xF0: rddata = __date32__[0]; break;
					case 0xF1: rddata = __date32__[1]; break;
					case 0xF2: rddata = __date32__[2]; break;
					case 0xF3: rddata = __date32__[3]; break;
					case 0xF4: rddata = __time32__[0]; break;
					case 0xF5: rddata = __time32__[1]; break;
					case 0xF6: rddata = __time32__[2]; break;
					case 0xF7: rddata = __time32__[3]; break;
					case 0xF8: rddata = SystemCoreClock; break;
					case 0xF9: rddata = VER_TYPE << 30 | FAC_VER << 24 | FAC_REV << 16 | VER << 8 | IS_STONE<<7 | REV; break;
					case 0xFA: rddata = (uint32_t)soft_wdt() << 1 | (uint32_t)hard_wdt(); break;

					case 0xFF: rddata = test_reg; break;

					case 0x100: rddata = pa_getPos(); break;
					case 0x101: rddata = pa_getWraddr(); break;
//					case 0x102: rddata = 1024; break; // usb rw test
					case 0x102: rddata = PA_SIZE; break;
					case 0x103: rddata = imit_fifo_count(); break;
					case 0x104: rddata = (uint32_t)cnc_getState(); break;
					case 0x105: *p32 = pa_getStrNum(); break;

					case 0x110:
					{
						motor_t* const m = imit_fifo_q();
						m->valid = !imit_fifo_empty();
						const uint32_t* const ptr32 = (uint32_t*)m;
						rddata = ptr32[0];
					}
						break;
					case 0x111:
					{
						const motor_t* const m = imit_fifo_q();
						const uint32_t* const ptr32 = (const uint32_t*)m;
						rddata = ptr32[1];
					}
						break;
					case 0x112:
					{
						const motor_t* const m = imit_fifo_q();
						const uint32_t* const ptr32 = (const uint32_t*)m;
						rddata = ptr32[2];
						imit_fifo_rdack();
					}
						break;

					default:
						rddata = 0;
						break;
				}
			else if (addr32 < 0x220) {
				rddata = bkp_readRegU32(addr32 & 0x1F);
#ifdef PRINT
				printf("Bkp %d %08x\n", (int)(addr32 & 0x1F), (int)rddata);
#endif
			}
			else
				rddata = 0;			   
			
			write_u32(tx_buf, sizeof(tx_buf), pos, rddata);
		}
	}
	else if ((addr & ADDR_MASK) == FPGA_BAR && (addr & 1) == 0 && (len & 1) == 0) {
		uint32_t addr16 = (addr & ~ADDR_MASK) >> 1;
		size_t len16 = len >> 1;

		for (int i = 0; i < len16; i++, addr16++, pos += sizeof(uint16_t)) {
			uint16_t rddata16 = fpga_read_u16(addr16);
			write_u16(tx_buf, sizeof(tx_buf), pos, rddata16);
		}
	}
	else
		memset(&tx_buf[pos], 0, len);

	return burst ? tx_readRegsAckBurst(addr, len) : tx_readRegsAck(addr, len);
}

/*
 * Reading data from one address
 */
void ad_readFifo(uint32_t addr, size_t len) {
	uint32_t rddata;
	if (len > sizeof(tx_buf) - 9) len = sizeof(tx_buf) - 9;

	size_t pos = 5;

	if ((addr & ADDR_MASK) == MCU_BAR && (addr & 3) == 0 && (len & 3) == 0) {
		uint32_t addr32 = (addr & ~ADDR_MASK) >> 2;
		size_t len32 = len >> 2;

		if (addr32 == 0x110) {
			size_t new_len32 = 3 * imit_fifo_count(); // one FIFO record 3 DWORDs

			if (new_len32 > len32)
				len32 = (len32 / 3) * 3; // aligned number
			else
				len32 = new_len32;

			size_t cnt = 0;

			for (int i = 0; i < len32; i++, pos += sizeof(uint32_t)) {
				motor_t* const m = imit_fifo_q();
				m->valid = !imit_fifo_empty();
				const uint32_t* const ptr32 = (uint32_t*)m;

				if (cnt > 2) cnt = 0;
				rddata = ptr32[cnt++];

				if (cnt == 3 && m->valid) imit_fifo_rdack();

				write_u32(tx_buf, sizeof(tx_buf), pos, rddata);
			}

			tx_readFifoAck(addr, len32<<2);
			return;
		}
	}

	memset(&tx_buf[pos], 0, len);
	tx_readFifoAck(addr, len);
}
