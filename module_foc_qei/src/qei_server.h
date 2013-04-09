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

	 bit_3   bit_2   bit_1    bit_0
	-------  -----  -------  -------
  Un-used  Index  Phase_B  Phase_A

	In normal operation the B and A bits change as a grey-code,
	with the following convention

			  ----------->  Counter-Clockwise
	BA:  00 01 11 10 00
			  <-----------  Clockwise

	During one revolution, BA will change 1024 times,
	Index will take the value of zero 1023 times, and the value one once only,
  at the position origin. 
	NB When the motor starts, it is NOT normally at the origin

	A look-up table is used to decode the 2 phase bits, into a spin direction
	with the following meanings: 
		 1: Anit-Clocwise, 
		 0: Unknown    (The motor has either stopped, or jumped one or more phases)
		-1: Clocwise, 

	The timer is read every time the phase bits change. I.E. 1024 times per revolution

	The angular postion is incremented/decremented (with the spin value) if the 
	motor is NOT at the origin. 
	If the motor is at the origin, the angular position is reset to zero.

\*****************************************************************************/

#ifndef _QEI_SERVER_H_
#define _QEI_SERVER_H_

#include <xs1.h>
#include <assert.h>
#include <print.h>

#include "app_global.h"
#include "qei_common.h"

/* This is a bit of a cludge, we are using a non-standard configuration
 * where the timer on the tile for inner_loop() is running at 250 MHz,
 * but other timers are running at the default of 100 MHz.
 * Currently this flexibility to define timer frequencies for each tile does not exist.
 * Therefore, we set up the timer frequency here.
 */
#ifndef PLATFORM_REFERENCE_MHZ
#define PLATFORM_REFERENCE_MHZ 250
#define PLATFORM_REFERENCE_KHZ 250000
#define PLATFORM_REFERENCE_HZ 250000000 // NB Uses 28-bits
#endif

// Calculate speed definitions, preserving precision and preventing overflow !-)
#define TICKS_PER_SEC_PER_QEI (PLATFORM_REFERENCE_HZ / QEI_PER_REV) // Ticks/sec/angular_increment // 18-bits
#define TICKS_PER_MIN_PER_QEI (60 * TICKS_PER_SEC_PER_QEI) // Ticks/min/angular_increment // 24 bits
#define MIN_TICKS_PER_QEI (TICKS_PER_MIN_PER_QEI / MAX_SPEC_RPM) // Min. expected Ticks/QEI // 12 bits
#define THR_TICKS_PER_QEI (MIN_TICKS_PER_QEI >> 1) // Threshold value used to trap annomalies // 11 bits

#define MAX_CONFID 2 // Maximum confidence value
#define MAX_QEI_ERR 8 // Maximum number of consecutive QEI errors allowed

#define QEI_CNT_LIMIT (QEI_PER_REV + HALF_QEI_CNT) // 540 degrees of rotation

#define QEI_PHASES 4	// 4 combinatations of Phases_B & Phases_A  E.g. [ 00 01 11 10 ]

#define START_UP_CHANGES 3 // Must see this number of pin changes before calculating velocity

#define QEI_SCALE_BITS 16 // Used to generate 2^n scaling factor
#define QEI_HALF_SCALE (1 << (QEI_SCALE_BITS - 1)) // Half Scaling factor (used in rounding)

#define QEI_COEF_BITS 8 // Used to generate filter coef divisor. coef_div = 1/2^n
#define QEI_COEF_DIV (1 << QEI_COEF_BITS) // Coef divisor
#define QEI_HALF_COEF (QEI_COEF_DIV >> 1) // Half of Coef divisor

/** Different Motor Phases */
typedef enum QEI_ENUM_TAG
{
  QEI_CLOCK = -1, // Clockwise Phase change
  QEI_STALL = 0,  // Same Phase
  QEI_ANTI = 1,		// Anti-Clockwise Phase change
  QEI_JUMP = 2,		// Jumped 2 Phases
} QEI_ENUM_TYP;

/** Structure containing QEI parameters for one motor */
typedef struct QEI_DATA_TAG // 
{
	unsigned inp_pins; // Raw data values on input port pins
	unsigned prev_phases; // Previous phase values
	unsigned prev_time; // Previous angular position time stamp
	unsigned diff_time; // Difference between 2 adjacent time stamps. NB Must be unsigned due to clock-wrap 
	QEI_ENUM_TYP prev_state; // Previous QEI state
	int err_cnt; // counter for invalid QEI states
	int orig_cnt; // Increment every time motor passes origin (index)
	int ang_cnt; // Counts angular position of motor (from origin)
	int theta; // angular position returned to client
	int spin_sign; // Sign of spin direction
	int prev_orig; // Previous origin flag
	int confid; // Confidence in current qei-state
	int id; // Unique motor identifier
	int dbg; // Debug

	int filt_val; // filtered value
	int coef_err; // Coefficient diffusion error
	int scale_err; // Scaling diffusion error 
} QEI_DATA_S;

/*****************************************************************************/
void foc_qei_do_multiple( // Get QEI Sensor data from port (motor) and send to client
	streaming chanend c_qei[], // Array of channels connecting server & client
	port in p4_qei[] // Array of QEI data ports for each motor
);
/*****************************************************************************/

#endif // _QEI_SERVER_H_
