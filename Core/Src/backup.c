#include "backup.h"
#include <stdio.h>

#include "stm32h7xx_hal.h"
#include "rtc.h"
#include "stm32_hal_legacy.h"

#define BKP_REG32_NUM (32)
#define BKP_SIZE (4 * 1024) // 4KB
static volatile uint8_t* const bkpSram = (volatile uint8_t*)D3_BKPSRAM_BASE;
static volatile uint32_t* const bkpSram_u32 = (volatile uint32_t*)D3_BKPSRAM_BASE;

void bkp_enable() {
    /*DBP : Enable access to Backup domain */
    HAL_PWR_EnableBkUpAccess();
    /*PWREN : Enable backup domain access  */
//    __HAL_RCC_PWR_CLK_ENABLE();
    /*BRE : Enable backup regulator
      BRR : Wait for backup regulator to stabilize */
    HAL_PWREx_EnableBkUpReg();

//    __HAL_RCC_BKPSRAM_CLK_ENABLE();
   /*DBP : Disable access to Backup domain */
//    HAL_PWR_DisableBkUpAccess();
}

void writeBkpSram(size_t pos, uint8_t data) {
   /* Enable clock to BKPSRAM */
//  __HAL_RCC_BKPSRAM_CLK_ENABLE();
  /* Pointer write on specific location of backup SRAM */
  bkpSram[pos] = data;
 /* Disable clock to BKPSRAM */
// __HAL_RCC_BKPSRAM_CLK_DISABLE();
}

uint8_t readBkpSram(size_t index) {
   uint8_t res;

  /* Enable clock to BKPSRAM */
//  __HAL_RCC_BKPSRAM_CLK_ENABLE();
  /* Pointer write from specific location of backup SRAM */
  res =  bkpSram[index];
  /* Disable clock to BKPSRAM */
//  __HAL_RCC_BKPSRAM_CLK_DISABLE();
  return res;
}

void bkp_clearSram() {
//	__HAL_RCC_BKPSRAM_CLK_ENABLE();

	for (int i = 0; i < BKP_SIZE; i++)
		bkpSram[i] = 0;

//	__HAL_RCC_BKPSRAM_CLK_DISABLE();
}

void bkp_clearReg() {
	for (int i = 0; i < BKP_REG32_NUM; i++)
		HAL_RTCEx_BKUPWrite(&hrtc, i, 0);
}

void bkp_printSramString(size_t pos, size_t N) {
//	__HAL_RCC_BKPSRAM_CLK_ENABLE();

	size_t end = pos + N;

	if (end > BKP_SIZE)
		end = BKP_SIZE;

	for (int i = pos; i < end && bkpSram[i] != '\0'; i++)
		printf("%c", (char)bkpSram[i]);

	printf("\n");

//	__HAL_RCC_BKPSRAM_CLK_DISABLE();
}

void bkp_printRegString() {
	int stop = 0;

	for (int i = 0; i < BKP_REG32_NUM && !stop; i++) {
		uint32_t data = HAL_RTCEx_BKUPRead(&hrtc, i);
		const uint8_t* const data8 = (const uint8_t*)&data;

		for (int j = 0; j < sizeof(uint32_t); j++) {
			if (data8[j] == '\0') {
				stop = 1;
				break;
			}
			printf("%c", (char)data8[j]);
		}
	}

	printf("\n");
}

void bkp_printSram(size_t pos, size_t N) {
//	__HAL_RCC_BKPSRAM_CLK_ENABLE();

	size_t end = pos + N;

	if (end > BKP_SIZE)
		end = BKP_SIZE;

	for (int i = pos; ; ) {
		printf("%02x", (int)bkpSram[i++]);

		if (i >= end) {
			printf("\n");
			break;
		}
		else if ((i & 0xf) == 0)
			printf("\n");
		else
			printf(" ");
	}

//	__HAL_RCC_BKPSRAM_CLK_DISABLE();
}

void bkp_printReg() {
	for (int i = 0; i < BKP_REG32_NUM; i++) {
		uint32_t data = HAL_RTCEx_BKUPRead(&hrtc, i);
		printf("%08x", (int)data);
		if ((i & 0x3) == 3)
			printf("\n");
		else
			printf(" ");
	}
}

void bkp_writeStringSram(size_t pos, const char* str, size_t N) {
//	__HAL_RCC_BKPSRAM_CLK_ENABLE();

	size_t end = pos + N;

	if (end > BKP_SIZE)
		end = BKP_SIZE;

	for (int i = pos; i < end; i++)
		bkpSram[i] = *str++;

//	__HAL_RCC_BKPSRAM_CLK_DISABLE();
}

void bkp_writeRegString(const char* str) {
	int j = 0;
	uint32_t data = 0;
	uint8_t* const ptr = (uint8_t*)data;

	for (int i = 0; i < BKP_REG32_NUM && *str != '\0'; i++) {
		data = 0;

		for (j = 0; j < sizeof(int32_t) && *str != '\0'; j++, str++)
			ptr[j] = *str;

		HAL_RTCEx_BKUPWrite(&hrtc, i, data);
	}
}

void bkp_clearContextSram() {
	for (int i = 0; i < sizeof(cnc_context_t) / sizeof(uint32_t); i++)
		bkpSram_u32[i] = 0;
}


void bkp_clearContextReg() {
	for (int i = 0; i < BKP_REG32_NUM; i++)
		HAL_RTCEx_BKUPWrite(&hrtc, i, 0);
}

void bkp_saveContextSram(const cnc_context_t* const ctx) {
	for (int i = 0; i < CNC_CONTEX_SIZE32; i++)
		bkpSram_u32[i] = ctx->data[i];
}

void bkp_saveContextReg(cnc_context_t* const ctx) {
//	ctx->field.backup_valid = 1;
	ctx->bytes[52] |= 1;

#ifdef PRINT
	printf("Backup\n");
#endif

	for (int i = 0; i < BKP_REG32_NUM; i++) {
		if (i < sizeof(cnc_context_t) / sizeof(uint32_t)) {
			HAL_RTCEx_BKUPWrite(&hrtc, i, ctx->data[i]);
#ifdef PRINT
			printf("%d %08x\n", i , (int)ctx->data[i]);
#endif
		}
		else {
			HAL_RTCEx_BKUPWrite(&hrtc, i, 0);
#ifdef PRINT
			printf("%d 0\n", i);
#endif
		}
	}
}

BOOL bkp_readContextSram(cnc_context_t* const ctx) {
	for (int i = 0; i < CNC_CONTEX_SIZE32; i++)
		ctx->data[i] = bkpSram_u32[i];

	return ctx->field.backup_valid;
}

BOOL bkp_readContextReg(cnc_context_t* const ctx) {
	for (int i = 0; i < CNC_CONTEX_SIZE32 && i < BKP_REG32_NUM; i++)
		ctx->data[i] = HAL_RTCEx_BKUPRead(&hrtc, i);

	return ctx->field.backup_valid;
}

uint32_t bkp_readSramU32(size_t index) {
	if (index >= BKP_SIZE / sizeof(uint32_t))
		index = BKP_SIZE / sizeof(uint32_t) - 1;

	return bkpSram_u32[index];
}

uint32_t bkp_readRegU32(size_t index) {
	if (index >= BKP_REG32_NUM)
		return 0;

	return HAL_RTCEx_BKUPRead(&hrtc, index);
}

void bkp_writeRegU32(size_t index, uint32_t data) {
	if (index >= BKP_REG32_NUM)
		return;

	HAL_RTCEx_BKUPWrite(&hrtc, index, data);
}

void bkp_print() {
	static cnc_context_t bkp_ctx;

	if ( bkp_readContextReg(&bkp_ctx) )
	  print_cnc_context(&bkp_ctx);
	else
	  printf("No backup\n");
}

void bkp_test() {
	//static const char __date__[16] = __DATE__;
//	static const char __time__[16] = __TIME__;
//	size_t base = 0, N = 64;

	printf("Backup Test\n");

	bkp_enable();

	HAL_PWREx_EnableMonitoring();
	uint32_t temp = HAL_PWREx_GetTemperatureLevel();
	uint32_t vbat = HAL_PWREx_GetVBATLevel();
	printf("Temp:%x V:%x\n", (int)temp, (int)vbat);

	printf("Read Backup Regs\n");
	bkp_printReg();

	printf("Write Backup Regs\n");
	for (int i = 0; i < 32; i++)
		HAL_RTCEx_BKUPWrite(&hrtc, i, i);

	printf("Read Backup Regs\n");
	bkp_printReg();

//	printf("Write\n");
//	bkp_write_string(base, __time__, sizeof(__time__));
//	printf("Read\n");
//	bkp_print_string(base, N);
//	bkp_print_bytes(base, N);

	HAL_PWREx_DisableMonitoring();
	HAL_PWREx_DisableBkUpReg();
	HAL_PWR_DisableBkUpAccess();
}
