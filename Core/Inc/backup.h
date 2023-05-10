#ifndef INC_BACKUP_H_
#define INC_BACKUP_H_

#include "my_types.h"

void bkp_enable();
void bkp_print();

void bkp_clearContextReg();
void bkp_saveContextReg(cnc_context_t* const ctx);

uint32_t bkp_readRegU32(size_t index);
void bkp_writeRegU32(size_t index, uint32_t data);

void bkp_test();

#endif /* INC_BACKUP_H_ */
