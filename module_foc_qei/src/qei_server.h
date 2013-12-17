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

#define QEI_SCALE_BITS 16 // Used to generate 2^n scaling factor
#define QEI_HALF_SCALE (1 << (QEI_SCALE_BITS - 1)) // Half Scaling factor (used in rounding)

#define QEI_BUF_BITS 5 // Use power-of-2 size to get all 1's mask
#define QEI_BUF_SIZ (1 << QEI_BUF_BITS) 
#define QEI_BUF_MASK (QEI_BUF_SIZ - 1)

#define MAX_QEI_STATUS_ERR 3 // 3 Maximum number of consecutive QEI status errors allowed

/* HALF_PERIOD determines the clock frequency for port sampling. 
 * The sampling period must allow enough time (inbetween samples) for processing
 * Currently this is about 660..680 cycles per 32-bit buffer. 
 * Therefore ~85 cycles/sample. There are a maximum of 2 motors to service.
 * Therefore, 170 cycles/sample/motor. With safety margin lets make it 192 cycles.
 */
#define HALF_PERIOD 90 // (Min 87) // Use Slow clock for Regular-Sampling mode 
#define TICKS_PER_SAMP (HALF_PERIOD << 1) // NB Max 510

#define SAMP_LOOP_BITS 3
#define SAMPS_PER_LOOP (1 << SAMP_LOOP_BITS) // 8
#define TICKS_PER_LOOP (TICKS_PER_SAMP << SAMP_LOOP_BITS) // 4080
#define STAG_TICKS ((TICKS_PER_LOOP + (NUMBER_OF_MOTORS >> 1)) / NUMBER_OF_MOTORS) // 2040 NB Used to stagger servicing of port buffers

// Require filter to decay from 1 to 1/2 after 5 samples. This is equivalent to filter coef of 0.1295 (~ 1/8)
#define QEI_VELOC_BITS 3 // NB Equivalent to a filter coefficient of 1/8)
#define QEI_HALF_VELOC (1 << (QEI_VELOC_BITS - 1)) 

#define MAX_TIME_ERR 1 // Max. No of consecutive timing errors allowed 

#define DBG_SIZ 384
				
/** Different QEI phases */
typedef enum QEI_PHASE_ETAG
{
  QEI_PHASE_A = 0,  // Phase_A identifier
  QEI_PHASE_B,  // Phase_A identifier
	NUM_QEI_PHASES // Number of different QEI Phase signals
} QEI_PHASE_ETYP;

typedef signed char ANG_INC_TYP; // Angular Increment type

typedef struct QEI_BUF_TAG // 
{
	QEI_RAW_TYP inp_pins; // Set of raw data values on input port pins
	int id; // Motor Id
	unsigned time32; // 32-bit Time-stamp for when port-pins read
	PORT_TIME_TYP time16; // 16-bit Time-stamp for when port-pins read
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

/** Type containing 2-D array for look-up table */
typedef struct QEI_LUT_TAG
{
	int incs[QEI_PERIOD_LEN][QEI_PERIOD_LEN];	// 2-D Look-up table
} QEI_LUT_TYP;

/** Structure containing all data for one QEI phase */
typedef struct QEI_PHASE_TAG // 
{
	int up_filt; // Up-scaled Low-pass filtered Phase signal
	unsigned prev; // Previous value of filtered phase signal
	int scale_err; // Error diffusion value for scaling
	int filt_err; // Error diffusion value for filtering
	QEI_PHASE_ETYP phase_id; // Unique QEI Phase identifier
	int motor_id; // Unique motor identifier
} QEI_PHASE_TYP;

/** Structure containing QEI parameters for one motor */
typedef struct QEI_DATA_TAG // 
{
#if (QEI_DBG)
	ALL_DBG_TYP dd; // All Debug data
#endif // QEI_DBG

	QEI_PARAM_TYP params; // QEI Parameter data (sent to QEI Client)
	QEI_LUT_TYP ang_lut;	// Look-up table for converting phase changes to angle increments
	QEI_PHASE_TYP phase_data[NUM_QEI_PHASES];	// Structure containing all data for one QEI phase

	unsigned prev_phases; // Previous phase values

	unsigned prev_time; // Previous port time-stamp
	unsigned change_time; // Time-stamp when valid phase change detected
	unsigned prev_change; // Previous valid phase change Time-stamp
	int tot_ang; // Counts total angular position of motor from time=0
	ANG_INC_TYP ang_inc; // angular increment value
	int orig_cnt; // Counts number of origin detections (revolutions) of motor
	int prev_orig; // Previous origin flag

	int status_errs; // counter for invalid QEI status errors
	int pin_changes; // Counts pin changes during start-up phase
	int id; // Unique motor identifier

	char dbg_str[3]; // String representing BA values as charaters (e.g. "10" )
	int dbg; // Debug

	int tmp_raw; // Debug
	int tmp_i[4]; // Debug
} QEI_DATA_TYP;

// Assign correct port type for sampling mode
#if (1 == QEI_RS_MODE)
#define QEI_PORT port:32 // Use 32-bit buffering for Regular-Sampling mode 
#else // if (1 == QEI_RS_MODE)
#define QEI_PORT port:4  // Use 4-bit buffering for Edge-Trigger mode 
#endif // else !(1 == QEI_RS_MODE)

/*****************************************************************************/
/** \brief Get QEI Sensor data from port (motor) and send to client
 * \param p4_qei // Array of QEI data ports for each motor
 * \param qei_clk // clock for generating accurate QEI timing
 */
void foc_qei_config(  // Configure all QEI ports
  buffered QEI_PORT in pb4_QEI[NUMBER_OF_MOTORS], // Array of buffered 4-bit input ports (carries raw QEI motor data)
	clock qei_clks[NUMBER_OF_MOTORS] // Array of clocks for generating accurate QEI timing (one per input port)
	);
/*****************************************************************************/
/** \brief Get QEI Sensor data from port (motor) and send to client
 * \param c_qei // Array of channels connecting server & client
 * \param p4_qei // Array of QEI data ports for each motor
 */
void foc_qei_do_multiple( // Get QEI Sensor data from port (motor) and send to client
	streaming chanend c_qei[], // Array of channels connecting server & client
	buffered QEI_PORT in pb4_inp[NUMBER_OF_MOTORS] // Array of 32-bit buffered 4-bit input ports on which to receive test data
);
/*****************************************************************************/

#endif // _QEI_SERVER_H_
