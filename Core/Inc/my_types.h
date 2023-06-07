#ifndef SRC_MY_TYPES_H_
#define SRC_MY_TYPES_H_

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "defines.h"

#define DWORDS(BYTES) (BYTES > (BYTES / 4) * 4 ? BYTES / 4 + 1 : BYTES / 4)

typedef enum {FALSE, TRUE} BOOL;

// CNC
typedef enum {AXIS_X, AXIS_Y, AXIS_U, AXIS_V, MOTORS} AXIS_T;
typedef enum { DIR_PLUS = 0, DIR_MINUS = 1 } DIR_T;

typedef struct { int32_t x, y; } point_t;

typedef struct { double x, y; } fpoint_t;
typedef struct { double x, y, x_inv, y_inv; } scale_t;

typedef struct { uint32_t x, y; } path_t;

typedef enum { BACKWARD = -1, STAY = 0, FORWARD = 1 } DIRECTION_T;

typedef enum {PLANE_UNKNOWN, PLANE_XY, PLANE_XYUV} PLANE_T;

typedef struct {
	DIRECTION_T dir;
	AXIS_T axis;
} main_dir_t;

typedef struct {
	DIRECTION_T x;
	DIRECTION_T y;
} dir_t;

typedef struct {
	int32_t str_num;
	point_t pt; // current point XY
	point_t uv_pt; // UV
} context_t;

//typedef union {
//    uint32_t data[3];
//
//    struct {
//        uint32_t pump_ena:1;
//        uint32_t drum_state:2;
//        uint32_t wire_ena:1;
//        uint32_t voltage_ena:1;
//        uint32_t hold_ena:1;
//        uint32_t uv_ena:1;
//        uint32_t :1;
//        uint32_t drum_vel:8;
//        uint32_t voltage_level:8;
//        uint32_t current_index:8;
//
//        uint32_t pulse_width:8;
//        uint32_t pulse_ratio:8;
//        uint32_t :16;
//
//        float speed;
//    } field;
//} cnc_controls_t;

#define CNC_CONTEX_SIZE32 (14)

typedef union {
    uint32_t data[CNC_CONTEX_SIZE32];
	uint8_t bytes[CNC_CONTEX_SIZE32 * sizeof(uint32_t)];														

    struct {
    	// 0
        uint32_t pump_ena:1;
        uint32_t drum_state:2;
        uint32_t wire_ena:1;

        uint32_t hv_ena:1;
        uint32_t hold_ena:1;
        uint32_t center_ena:1;
        uint32_t is_init:1;

        uint32_t drum_vel:7;
        uint32_t d_ena:1;

        uint32_t uv_ena:1;
        uint32_t enc_ena:1;
        uint32_t rev:1;
        uint32_t rollback:1;

        uint32_t attempt:3;
        uint32_t acc_ena:1;

        uint32_t state:8;

        // 1
        uint32_t pulse_width:8;
        uint32_t pulse_ratio:8;

        uint32_t low_hv_ena:1;
        uint32_t :7;

        uint32_t current_index:8;

        // 2, 3, 4, 5, 6
        int32_t id, x, y, u, v;

        // 7, 8
        int32_t enc_x, enc_y;

        // 9, 10
        float T, T_cur; // clocks/mm

        // 11
        float step; // mm

        // 12
        uint32_t limsw_fwd:1;
        uint32_t limsw_rev:1;
        uint32_t limsw_alm:1;
        uint32_t wire_break:1;

        uint32_t pwr:1;
        uint32_t fb_stop:1;
        uint32_t fb_to:1;
        uint32_t hv_enabled:1;

        uint32_t sem_ena:1;
        uint32_t sem:3;

        uint32_t fb_ena:1;
        uint32_t attempts:3;

        uint32_t center_state:3;
        uint32_t touch_state:3;
        uint32_t center_mode:2;

        uint32_t center_attempt:8;

        // 13
        uint32_t backup_valid:1; // must be last reg
        uint32_t read_valid:1;
        uint32_t :6;

        uint32_t center_attempts:8;
        uint32_t touch:8;
        uint32_t touches:8;
    } field;
} cnc_context_t;

typedef struct {
	point_t coord;
	path_t path;
} pos_t;

void point_clear(point_t* const pt);
void fpoint_clear(fpoint_t* const pt);
void path_clear(path_t* const path);
void dir_clear(dir_t* const dir);
void context_clear(context_t* const ctx);
void pos_clear(pos_t* const pos);

void point_print(const point_t* const pt);
void fpoint_print(const fpoint_t* const pt);

void print_cnc_context(const cnc_context_t* const ctx);

#endif /* SRC_MY_TYPES_H_ */
