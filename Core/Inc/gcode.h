#ifndef INC_GCODE_H_
#define INC_GCODE_H_

#include "gcmd.h"

BOOL gcode_parse(const char** str, gcmd_t* const cmd, int* const G);

#endif /* INC_GCODE_H_ */
