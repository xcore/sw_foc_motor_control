/**
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
 **/                                   

#ifndef _GENERATE_QEI_TESTS_H_
#define _GENERATE_QEI_TESTS_H_

#include <stdlib.h>

#include <xs1.h>
#include <assert.h>
#include <print.h>
#include <safestring.h>

#include "app_global.h"
#include "use_locks.h"
#include "qei_common.h"
#include "qei_server.h"
#include "test_qei_common.h"

#define MAX_TESTS 31 // No. of tests used for Max. speed check
#define MIN_TESTS 3  // No. of tests used for Min. speed check
#define ACC_TESTS 18 // No. of tests used for Acceleration check
#define DEC_TESTS 18 // No. of tests used for Deceleration check

#define NUM_PORT_TIMES (1 << 16) // No. of port timer values (16-bit)

#define SCALE_PRECISION 10 // No. of Bits for Scaling Factor Divisor
#define HALF_SCALE (1 << (SCALE_PRECISION - 1)) // Half Scaling factor Used for Rounding

#define ACC_SCALE 807 // Scaling factor Used for Acceleration (ACC_SCALE >> SCALE_PRECISION) 
#define DEC_SCALE 1300 // Scaling factor Used for Deceleration (DEC_SCALE >> SCALE_PRECISION) 

typedef struct QEI_PHASE_TAG // Structure containing Array of QEI Phase values
{
	int vals[NUM_QEI_PHASES];	// Array of QEI Phase values
} QEI_PHASE_TYP;

typedef struct TEST_QEI_TAG // Structure containing QEI test data
{
	char names[NUM_VECT_COMPS][STR_LEN]; // Array of names for each component of test vector
	TEST_VECT_TYP vector; // Structure of containing QEI test vector (QEI conditions to be tested)
	QEI_PHASE_TYP clk_wise;	// Structure of QEI phase values for clockwise rotation
	QEI_PHASE_TYP anti_clk;	// Structure of QEI phase values for anti-clockwise rotation
	QEI_PHASE_TYP phase;	// Structure of currently used QEI phase values
	int id;			// Current motor identifier
	int hi_ticks;			// No. of ticks/QEI at high speed
	int lo_ticks;			// No. of ticks/QEI at low speed
	int off;			// offset into QEI Phase cycle
	int cnt; // QEI position counter
	int orig; // QEI origin flag (Bit_2 is 1 at origin)
	int nerr; // QEI error flag (Bit_3 is 1 for NO errors)
	int scale; // velocity scaling factor (used for acceleration and deceleration)
	PORT_TIME_TYP time; // port timer value
	unsigned period; // period (in ticks) between tests
	int prev_qei;  // Previous QEI value
	int print;  // Print flag
	int dbg;  // Debug flag
} TEST_QEI_TYP;

/*****************************************************************************/
/** Generate QEI test data for all motors
 * \param c_tst // Channel for sending test vecotrs to test checker
 * \param p4_tst[]  // Array of ports on which to transmit test data
 */
void gen_all_qei_test_data( // Generate QEI Test data for all motors
	streaming chanend c_tst, // Channel for sending test vecotrs to test checker
	port out p4_tst[]  // Array of ports on which to transmit test data
);
/*****************************************************************************/
#endif /* _GENERATE_QEI_TESTS_H_ */
