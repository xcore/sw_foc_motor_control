/*
 * The copyrights, all other intellectual and industrial 
 * property rights are retained by XMOS and/or its licensors. 
 * Terms and conditions covering the use of this code can
 * be found in the Xmos End User License Agreement.
 *
 * Copyright XMOS Ltd 2013
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the 
 * copyright notice above.
 */                                   

/*****************************************************************************\
	This code is designed to work on a Motor with a Max speed of 4000 RPM,
	and a 1024 counts per revolution.

	The QEI data is read in via a 4-bit port. With assignments as follows:-
	WARNING: This in NON-standard. Traditionally Phase_A is the MS-bit.

	 bit_3   bit_2   bit_1    bit_0
	-------  -----  -------  -------
  N_Error  Index  Phase_B  Phase_A

	N_Error = 1 for No Errors


	In normal operation the B and A bits change as a grey-code,
	with the following convention.

	The positive spin direction is the one where Phase_A leads Phase_B.
	E.g. Phase_A goes high one bit-change earlier than Phase_B goes high.
	This definition is based on time, and is NOT dependent on spatial orientation of the motor!-)
 

			  <-----------  Negative Spin
	BA:  00 01 11 10 00
			  ----------->  Positive Spin

	During one revolution, BA will change 1024 times,
	Index will take the value of zero 1023 times, and the value one once only,
  at the position origin. 
	NB When the motor starts, it is NOT normally at the origin

	The timer is read every time the phase bits change. I.E. 1024 times per revolution

	The angular postion is incremented/decremented (with the spin value) if the 
	motor is NOT at the origin. 
	If the motor is at the origin, the angular position is reset to zero.

\*****************************************************************************/

#ifndef _QEI_SERVER_H_
#define _QEI_SERVER_H_

#include <stdlib.h>

#include <xs1.h>
#include <assert.h>
#include <print.h>
#include <safestring.h>
#include <syscall.h>

#include "qei_common.h"

#ifndef QEI_FILTER
	#error Define. QEI_FILTER in app_global.h
#endif

#ifndef MAX_SPEC_RPM 
	#error Define. MAX_SPEC_RPM in app_global.h
#endif // MAX_SPEC_RPM

#define QEI_DBG 0 // Set flag for printout of debug info.

#define HALF_QEI_CNT (QEI_PER_REV >> 1) // 180 degrees of mechanical rotation

#define MIN_TICKS_PER_QEI (TICKS_PER_MIN_PER_QEI / MAX_SPEC_RPM) // Min. expected Ticks/QEI // 12 bits
#define THR_TICKS_PER_QEI (MIN_TICKS_PER_QEI >> 1) // Threshold value used to trap annomalies // 11 bits

/* NB Confidence-level changes by 4, if a change of direction is detected. 
 * e.g. For MAX_CONFID = 11  3 consecutive estimates in the reverse direction are required to change spin 
 * 11 -> 7 -> 3 -> -1
 */
#define MAX_CONFID 11 // Maximum confidence value NB Choose odd positive number

// WARNING: The noisier the input data the higher MAX_QEI_STATE_ERR has to be.
#define MAX_QEI_STATE_ERR 128000 //MB~ 128 // 128 for 1 in 8 bit errors. Max. consecutive errors allowed.

#define QEI_CNT_LIMIT (QEI_PER_REV + HALF_QEI_CNT) // 540 degrees of rotation

#define START_UP_CHANGES 3 // Must see this number of pin changes before calculating velocity

#define QEI_SCALE_BITS 16 // Used to generate 2^n scaling factor
#define QEI_HALF_SCALE (1 << (QEI_SCALE_BITS - 1)) // Half Scaling factor (used in rounding)

#define QEI_COEF_BITS 8 // Used to generate filter coef divisor. coef_div = 1/2^n
#define QEI_COEF_DIV (1 << QEI_COEF_BITS) // Coef divisor
#define QEI_HALF_COEF (QEI_COEF_DIV >> 1) // Half of Coef divisor

#define MAX_QEI_STATUS_ERR 3 // 3 Maximum number of consecutive QEI status errors allowed

#define MIN_RPM 50 // In order to estimate the angular position, a minimum expected RPM has to be specified
// Now we can calculate the maximum expected time difference (in ticks) between QEI phase changes
#define MAX_TIME_DIFF (((MIN_RPM * TICKS_PER_SEC_PER_QEI) + (SECS_PER_MIN - 1)) / SECS_PER_MIN) // Round-up maximum expected time-diff (17-bits)

// WARNING: Values used in test harness need to be less severe than these. e.g. 807 and 1300
#define LO_QEI_SCALE 969 // 700 Scaling factor Used for Acceleration (ACC_SCALE >> SCALE_PRECISION) 
#define HI_QEI_SCALE 1114 // 1500 Scaling factor Used for Deceleration (DEC_SCALE >> SCALE_PRECISION) 
#define THIRD 341 // Scaling factor Used for calculating one third

/** Define No. of Bits for Scaling Factor Divisor */
#define SCALE_QEI_BITS 10 // No. of Bits for Scaling Factor Divisor
#define HALF_QEI_SCALE (1 << (SCALE_QEI_BITS - 1)) // Half Scaling factor Used for Rounding

#define INT16_BITS (sizeof(short) * BITS_IN_BYTE) // No. of bits in 16-bit integer
#define INT32_BITS (sizeof(int) * BITS_IN_BYTE) // No. of bits in 32-bit integer
#define INT64_BITS (sizeof(S64_T) * BITS_IN_BYTE) // No. of bits in signed 64-bit type!

#define QEI_BUF_BITS 5 // Use power-of-2 size to get all 1's mask
#define QEI_BUF_SIZ (1 << QEI_BUF_BITS) 
#define QEI_BUF_MASK (QEI_BUF_SIZ - 1)

#define DBG_SIZ 384

// WARNING. Positive and Negative states must have same magnitude
/** Different QEI decode states */
typedef enum QEI_STATE_ETAG
{
  QEI_HI_NEGA = -2,	// High probability Negative-spin Phase change
  QEI_LO_NEGA = -1,	// Low probability Negative-spin Phase change
  QEI_BIT_ERR = 0,		// Detected one or more bit errors 
  QEI_LO_POSI = 1, // Low probability Positive-spin Phase change
  QEI_HI_POSI = 2 // High probability Positive-spin Phase change
} QEI_STATE_ETYP;

typedef signed char ANG_INC_TYP; // Angular Increment type

typedef struct QEI_BUF_TAG // 
{
	QEI_RAW_TYP inp_pins; // Set of raw data values on input port pins
	int id; // Motor Id
	unsigned time; // Time when port-pins read
} QEI_BUF_TYP;

#if (QEI_DBG)
typedef struct DBG_SMP_TAG // MB~ Dbg
{
	char dbg_str[3]; // String representing BA values as charaters (e.g. "10" )
	int diff_time; // Difference between 2 adjacent time-stamps.
	ANG_INC_TYP phase_inc; // angular increment value
	ANG_INC_TYP hi_inc; // Higher bound for angular increment value
	ANG_INC_TYP lo_inc; // Lower bound for angular increment value
	ANG_INC_TYP ang_inc; // new angular increment value
	QEI_STATE_ETYP curr_state; // Curremt QEI state
	int confid; // Spin-direction confidence. (+ve: confident Positive spin, -ve: confident negative spin)
	int veloc; // measured angular velocity
} DBG_SMP_TYP;

typedef struct ALL_DBG_TAG // MB~ Dbg
{
	DBG_SMP_TYP ss[DBG_SIZ]; // Array of all Debug data
	int cnt; // Counts No of dbg array entries
} ALL_DBG_TYP;
#endif // QEI_DBG

/** Structure containing QEI parameters for one motor */
typedef struct QEI_DATA_TAG // 
{
#if (QEI_DBG)
	ALL_DBG_TYP dd; // All Debug data
#endif // QEI_DBG
	QEI_PARAM_TYP params; // QEI Parameter data (sent to QEI Client)
	QEI_PHASE_TYP inv_phase;	// Structure containing all inverse QEI phase values;
	unsigned prev_phases; // Previous phase values
	unsigned curr_time; // Time when port-pins read
	unsigned prev_time; // Previous port time-stamp
	unsigned t_dif_old; // oldest difference between 2 adjacent time-stamps. NB Must be unsigned due to clock-wrap 
	unsigned t_dif_cur; // current difference between 2 adjacent time-stamps. NB Must be unsigned due to clock-wrap 
	int t_dif_new; // newest difference between 2 adjacent time-stamps (down-scaled). NB Must be unsigned due to clock-wrap 
	int diff_time; // Difference between 2 adjacent time-stamps.
	int prev_diff; // Previous Difference between 2 adjacent time-stamps.
	int phase_index; // Converts [BA] phase value into circular index [0, 1, 2, 3]
	int prev_index; // previous circular phase index
	ANG_INC_TYP phase_inc; // Raw No. of angular increments from phase values
	ANG_INC_TYP hi_inc; // Higher bound for angular increment value
	ANG_INC_TYP lo_inc; // Lower bound for angular increment value
	ANG_INC_TYP ang_inc; // angular increment value
	unsigned prev_inc; // previous absolute angular increment value
	QEI_STATE_ETYP curr_state; // Current QEI state
	QEI_STATE_ETYP prev_state; // Previous QEI state
	int state_errs; // counter for invalid QEI state transistions
	int status_errs; // counter for invalid QEI status errors
	int pin_changes; // Counts pin changes during start-up phase
	int orig_cnt; // Counts number of origin detections (revolutions) of motor
	int ang_cnt; // Counts angular position of motor (from origin)
	int ang_speed; // Angular speed of motor measured in Ticks/angle_position
	int confid; // Spin-direction confidence. (+ve: confident Positive spin, -ve: confident Negative spin)
	int prev_orig; // Previous origin flag
	int half_qei; // Half QEI points per revolution (used for rounding)
	int id; // Unique motor identifier
	char dbg_str[3]; // String representing BA values as charaters (e.g. "10" )

	int filt_val; // filtered value
	int coef_err; // Coefficient diffusion error
	int scale_err; // Scaling diffusion error 
	int speed_err; // Speed diffusion error 

	int dbg; // Debug
} QEI_DATA_TYP;

/*****************************************************************************/
/** \brief Get QEI Sensor data from port (motor) and send to client
 * \param p4_qei // Array of QEI data ports for each motor
 * \param qei_clk // clock for generating accurate QEI timing
 */
void foc_qei_config(  // Configure all QEI ports
  buffered port:4 in pb4_QEI[NUMBER_OF_MOTORS], // Array of buffered 4-bit input ports (carries raw QEI motor data)
	clock qei_clk // clock for generating accurate QEI timing
	);
/*****************************************************************************/
/** \brief Get QEI Sensor data from port (motor) and send to client
 * \param c_qei // Array of channels connecting server & client
 * \param p4_qei // Array of QEI data ports for each motor
 */
void foc_qei_do_multiple( // Get QEI Sensor data from port (motor) and send to client
	streaming chanend c_qei[], // Array of channels connecting server & client
	buffered port:4 in pb4_qei[NUMBER_OF_MOTORS] // Array of buffered QEI data ports for each motor
);
/*****************************************************************************/
/** \brief Get QEI Sensor data from port (motor) and send to client
 * \param c_qei // channel connecting server & client
 * \param p4_qei // QEI data port for this motor
 */
void foc_qei_do_single( // Get QEI Sensor data from port (motor) and send to client
  int motor_id, // Unique Motor identifier	
	streaming chanend c_qei, // Array of channels connecting server & client
	buffered port:4 in pb4_qei // Array of buffered QEI data ports for each motor
);
/*****************************************************************************/

#endif // _QEI_SERVER_H_
