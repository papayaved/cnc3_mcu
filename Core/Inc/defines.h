#ifndef INC_DEFINES_H_
#define INC_DEFINES_H_

//#define RESEASE
#define STONE

#define NAME ("CNC")
#define MODEL ("3.1")

#if defined(RESEASE)
	#define VER_TYPE (1) // 0 - stable, 1 - alpha, 2 - beta
	#define WDT_ENA // a watch dog timer for MCU program error
	#define SOFT_WDT // for lose USB connection
#else
	#define VER_TYPE (2)
	#define PRINT
//	#define SOFT_WDT
#endif

#ifdef STONE
	#define IS_STONE (1) // stone version
#else
	#define IS_STONE (0)
#endif

/* Versions
 * Version 1.6
 * - added access to scale
 * - added comment
 * - changed some names
 * - cnc_enableUV - replaced to new address
 * Verion 1.7
 * - add using line encoders
 * 1.10 - fast load of G-code
 * 1.11 - correct line encoder
 * 1.12 - PID debug
 * 1.13 - added function cnc_state_reset for a light reseting CNC before the run
 * 1.14 - added LED control for WDTs and saving of context at hard WDT
 * 1.15 - added pa_clear() after cutting
 * 1.16 - from cnc2
 * 1.17 - soft pult abort at alarm
 * 1.18 - locked drum in FPGA (todo as in cnc2), move state machine alarm_sm to a separated file and rewrite logic, changed M commands, added low and high voltage thresholds
 * 1.19 - adding acceleration and deceleration by path
 */
#define FAC_VER (3)
#define FAC_REV (1) // CNC3.1
#define VER (1)
#define REV (19)

#ifndef MAX
	#define MAX(X, Y) ((X) > (Y) ? (X) : (Y))
#endif

#ifndef MIN
	#define MIN(X, Y) ((X) < (Y) ? (X) : (Y))
#endif

// Parameters
#define FPGA_CLOCK (72e6) // Hz
#define DEFAULT_PAUSE_CLOCK (FPGA_CLOCK / 7200) // 10,000 Hz

#define MS_TO_TICK(_MS) ((1.0e-3 * FPGA_CLOCK) * _MS) // convert milliseconds to FPGA ticks

#define ROLLBACK_DEFAULT	(0.3) // mm. This is the rollback distance after a short circuit
#define ROLLBACK_ATTEMPTS	(3) // After each rollback, the rollback is repeated again if the short circuit continues

#define F_MIN_MMM ( 0.01 ) // mm/min. Minimum speed
#define F_MAX_MMM ( 18 ) // mm/min. Maximum speed

#define MMM_TO_UMS(_MMM) ((_MMM) * (1000.0 / 60.0))
#define F_MIN_UMS ( MMM_TO_UMS(F_MIN_MMM) ) // um/sec. Minimum speed
#define F_MAX_UMS ( MMM_TO_UMS(F_MAX_MMM) ) // um/sec. Maximum speed

#define F_DEFAULT_UMS				(F_MAX_UMS) // um/s. Default cutting speed
#define F_ROLLBACK_DEFAULT_UMS		(F_MAX_UMS / 10) // um/s. Default rollback speed
#define T_DEFAULT_TICKS				( FPGA_CLOCK / (F_DEFAULT_UMS * 1e-3) ) // 240,000,000 clocks/mm
#define T_ROLLBACK_DEFAULT_TICKS	( FPGA_CLOCK / (F_ROLLBACK_DEFAULT_UMS * 1e-3) ) // 2,400,000,000 clocks/mm

#define COE_UMSEC2_TO_MMTICK2 ( 1e-3 / (FPGA_CLOCK * FPGA_CLOCK) ) // coefficient for convert from um/sec^2 to mm/clock^2
#define COE_UMS_TO_MMTICK ( 1e-3 / FPGA_CLOCK ) // coefficient for convert from um/sec to mm/tick
#define UMS_TO_MMTICKS(UMS)	( UMS * COE_UMS_TO_MMTICK ) // macros for conversion from um/sec to mm/clock

#define UMS_TO_TICKSMM(UMS)	( FPGA_CLOCK / (UMS * 1e-3) ) // macros for conversion from um/sec to clock/mm

#define T_MIN_TICK			UMS_TO_TICKSMM(F_MAX_UMS) // 240,000,000 clocks/mm. Minimum pulse period
#define T_MAX_TICK			UMS_TO_TICKSMM(F_MIN_UMS) // 72,000,000,000 clocks/mm. Maximum pulse period

#define F_MIN (1.0 / T_MAX_TICK) // mm/clock. Speed
#define F_MAX (1.0 / T_MIN_TICK) // mm/clock

// Scale motors and encoders
#define SCALE_MIN		(1) // steps/mm
#define SCALE_MAX		(1000000) // steps/mm
#define SCALE			(1000.0) // steps/mm
#define SCALE_UV		(1000.0) // steps/mm
#define SCALE_ENC		(1000.0 / 5) // steps/mm

#define STEP		(0.001) // mm
#define STEP_MIN	(1.0 / SCALE_MAX) // mm
#define STEP_MAX	(1.0 / SCALE_MIN) // mm

#define DRUM_VEL_MAX (7)

//#define SUM_IMIT_STEPS

#endif /* INC_DEFINES_H_ */
