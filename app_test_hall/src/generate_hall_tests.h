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

#ifndef _GENERATE_HALL_TESTS_H_
#define _GENERATE_HALL_TESTS_H_

#include <stdlib.h>

#include <xs1.h>
#include <assert.h>
#include <print.h>

#include "app_global.h"
#include "use_locks.h"
#include "hall_common.h"
#include "test_hall_common.h"

#define HIGH_SPEED 4000
#define LOW_SPEED  50

#define MAX_TESTS 13 // No. of tests used for Max. speed check
#define MIN_TESTS 5  // No. of tests used for Min. speed check
#define ACC_TESTS 18 // No. of tests used for Acceleration check
#define DEC_TESTS 18 // No. of tests used for Deceleration check

#define STR_LEN 256

#define SCALE_PRECISION 10 // No. of Bits for Scaling Factor Divisor
#define HALF_SCALE (1 << (SCALE_PRECISION - 1)) // Half Scaling factor Used for Rounding

#define ACC_SCALE 807 // Scaling factor Used for Acceleration (ACC_SCALE >> SCALE_PRECISION) 
#define DEC_SCALE 1300 // Scaling factor Used for Deceleration (DEC_SCALE >> SCALE_PRECISION) 

typedef struct HALL_PHASE_TAG // Structure containing Array of HALL Phase values
{
	int vals[NUM_HALL_PHASES];	// Array of HALL Phase values
} HALL_PHASE_TYP;

typedef struct TEST_HALL_TAG // Structure containing HALL test data
{
	HALL_PHASE_TYP clk_wise;	// Structure of HALL phase values for clockwise rotation
	HALL_PHASE_TYP anti_clk;	// Structure of HALL phase values for anti-clockwise rotation
	HALL_PHASE_TYP phase;	// Structure of currently used HALL phase values
	int hi_ticks;			// No. of ticks/HALL at high speed
	int lo_ticks;			// No. of ticks/HALL at low speed
	int off;			// offset into HALL Phase cycle
	int cnt; // HALL position counter
	int nerr; // HALL error flag (Bit_3 is 1 for NO errors)
	unsigned time; // previous timer value
	unsigned period; // period (in ticks) between tests
} TEST_HALL_TYP;

/*****************************************************************************/
/** Generate HALL test data for all motors
 * \param p4_tst[]  // Array of ports on which to transmit test data
 */
void gen_all_hall_test_data( // Generate HALL Test data for all motors
	port out p4_tst[]  // Array of ports on which to transmit test data
);
/*****************************************************************************/
#endif /* _GENERATE_HALL_TESTS_H_ */
