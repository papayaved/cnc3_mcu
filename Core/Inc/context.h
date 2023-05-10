#ifndef INC_CONTEXT_H_
#define INC_CONTEXT_H_

#include "my_types.h"

cnc_context_t* cnc_ctx_getForce();
uint32_t cnc_ctx_get(size_t index);
void cnc_ctx_clear();

#endif /* INC_CONTEXT_H_ */
