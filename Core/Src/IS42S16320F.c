//#include "IS42S16320F.h"
//
//FMC_SDRAM_CommandTypeDef command;
//HAL_StatusTypeDef hal_stat;
//
//void IS42S16320F_init(SDRAM_HandleTypeDef* hsdram) {
//	__IO uint32_t tmpmrd = 0;
//	command.CommandMode = FMC_SDRAM_CMD_CLK_ENABLE;
//	command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
//	command.AutoRefreshNumber = 1;
//	command.ModeRegisterDefinition = 0;
//	hal_stat = HAL_SDRAM_SendCommand(hsdram, &command, SDRAM_TIMEOUT);
//	HAL_Delay(1);
//
//	command.CommandMode = FMC_SDRAM_CMD_PALL;
//
//	command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
//
//	command.AutoRefreshNumber = 1;
//
//	command.ModeRegisterDefinition = 0;
//
//	hal_stat = HAL_SDRAM_SendCommand(hsdram, &command, SDRAM_TIMEOUT);
//
//	HAL_Delay(1);
//
//	command.CommandMode = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
//
//	command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
//
//	command.AutoRefreshNumber = 8;
//
//	command.ModeRegisterDefinition = 0;
//
//	hal_stat = HAL_SDRAM_SendCommand(hsdram, &command, SDRAM_TIMEOUT);
//
//	HAL_Delay(1);
//
//	tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_1          |
//
//						SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |
//
//						SDRAM_MODEREG_CAS_LATENCY_2           |
//
//						SDRAM_MODEREG_OPERATING_MODE_STANDARD |
//
//						SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;
//
//	 command.CommandMode = FMC_SDRAM_CMD_LOAD_MODE;
//
//	 command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
//
//	 command.AutoRefreshNumber = 1;
//
//	 command.ModeRegisterDefinition = tmpmrd;
//
//	 hal_stat = HAL_SDRAM_SendCommand(hsdram, &command, SDRAM_TIMEOUT);
//
//	 /* Step 8: Set the refresh rate counter */
//
//	/* (15.62 us x Freq) – 20 */
//
//	/* Set the device refresh counter */
//
//	hsdram->Instance->SDRTR |= ((uint32_t)((1292)<< 1));
//}
