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

	The timer is read every time a full buffer of samples is returned to the S/W.
	(Currently 8 4-bit samples) NB these may all be the same value.

	The samples are inspected in chronological order, and when a valid phase change is detected,
	the angular postion is incremented/decremented (with the spin value).

	If the origin flag is detected, the angular position is corrected (to the nearest multiple of 1024).

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
#define QEI_SCALE_DIV (1 << QEI_SCALE_BITS) // Scaling factor
#define QEI_SCALE_HALF (QEI_SCALE_DIV >>1) // Half Scaling factor (used in rounding)

#define MAX_QEI_STATUS_ERR 3 // 3 Maximum number of consecutive QEI status errors allowed

/* HALF_PERIOD determines the clock frequency for port sampling. 
 * The sampling period must allow enough time (inbetween samples) for processing
 * Currently this is about 660..680 cycles per 32-bit buffer. 
 * Therefore ~85 cycles/sample. There are a maximum of 2 motors to service.
 * Therefore, 170 cycles/sample/motor. With safety margin lets make it 192 cycles.
 */
#define HALF_PERIOD 98 // 94 (Min 87) // ~100 Half of Max. allowed No. of ticks-per-sample
#define TICKS_PER_SAMP (HALF_PERIOD << 1) // ~200 Max. allowed No. of ticks-per-sample

#define SAMP_LOOP_BITS 3 // Used to define No. of samples in 32-bit port buffer
#define SAMPS_PER_LOOP (1 << SAMP_LOOP_BITS) // 8  No. of samples in 32-bit port buffer
#define TICKS_PER_LOOP (TICKS_PER_SAMP << SAMP_LOOP_BITS) // ~1600 Time taken to service a buffer of samples
#define STAG_TICKS ((TICKS_PER_LOOP + (NUMBER_OF_MOTORS >> 1)) / NUMBER_OF_MOTORS) // ~800 NB Used to stagger servicing of port buffers

// Require filter to decay from 1 to 1/2 after 5 samples. This is equivalent to filter coef of 0.1295 (~ 1/8)
#define QEI_VELOC_BITS 3 // bit resolution of QEI filter coefficient
#define QEI_VELOC_DIV (1 << QEI_VELOC_BITS) // 8 Divisor for filter coefficient
#define QEI_VELOC_HALF (QEI_VELOC_DIV >> 1) // 4 Half of filt-coef divisor (used for rounding) 

#define MAX_TIME_ERR 1 // Max. No of consecutive timing errors allowed 

#define PERIOD_DELTA_LIM 5 // Allow a change in QEI PERIOD of upto 5 phases

typedef signed char ANG_INC_TYP; // Angular Increment type

/** Different QEI phases */
typedef enum QEI_PHASE_ETAG
{
  QEI_PHASE_A = 0,  // Phase_A identifier
  QEI_PHASE_B,  // Phase_B identifier
	NUM_QEI_PHASES // Number of different QEI Phase signals
} QEI_PHASE_ETYP;

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
	QEI_PARAM_TYP params; // QEI Parameter data (sent to QEI Client)
	QEI_LUT_TYP ang_lut;	// Look-up table for converting phase changes to angle increments
	QEI_PHASE_TYP phase_data[NUM_QEI_PHASES];	// Structure containing all data for one QEI phase

	unsigned prev_phases; // Previous phase values

	unsigned prev_time; // Previous port time-stamp
	unsigned change_time; // Time-stamp when valid phase change detected
	unsigned prev_change; // Previous valid phase change Time-stamp
	int ang_tot; // Counts total angular position of motor from time=0
	int prev_ang;	// Angular position when previous origin detected (possibly false)
	ANG_INC_TYP ang_inc; // angular increment value
	unsigned rev_period; // number of QEI phases changes per revolution
	int prev_orig; // Previous origin flag

	int status_errs; // counter for invalid QEI status errors
	int pins_idle; // Flag set until first pin change detected
	int id; // Unique motor identifier

	char dbg_str[3]; // String representing BA values as charaters (e.g. "10" )
	int dbg; // Debug
	int dbg_ang; // Debug

	int tmp_raw; // Debug
	int tmp_i[4]; // Debug
} QEI_DATA_TYP;

#define QEI_PORT port:32 // Use 32-bit buffering for Regular-Sampling mode 

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
