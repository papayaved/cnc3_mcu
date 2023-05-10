#include "controls.h"

struct {
	uint32_t start:1;
} buttons;

void ctrl_startButtonClick() { buttons.start = 1; }

BOOL ctrl_startButtonClicked() {
	BOOL res = buttons.start;
	buttons.start = 0;
	return res;
}

BOOL alt_bp_btn_clicked() {
	return FALSE;
}
