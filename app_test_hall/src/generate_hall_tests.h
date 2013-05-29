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
#include <safestring.h>

#include "app_global.h"
#include "use_locks.h"
#include "hall_common.h"
#include "hall_server.h"
#include "test_hall_common.h"

/** Define No. of tests used for Max. speed check */
#define MAX_TESTS 31 // No. of tests used for Max. speed check

/** Define No. of tests used for Min. speed check */
#define MIN_TESTS 3  // No. of tests used for Min. speed check

/** Define No. of tests used for Acceleration check */
#define ACC_TESTS 18 // No. of tests used for Acceleration check

/** Define No. of tests used for Deceleration check */
#define DEC_TESTS 18 // No. of tests used for Deceleration check

/** Define No. of port timer values (16-bit) */
#define NUM_PORT_TIMES (1 << 16) // No. of port timer values (16-bit)

/** Define Scaling factor Used for Acceleration */
#define ACC_SCALE 807 // Scaling factor Used for Acceleration (ACC_SCALE >> SCALE_PRECISION) 

/** Define Scaling factor Used for Deceleration */
#define DEC_SCALE 1300 // Scaling factor Used for Deceleration (DEC_SCALE >> SCALE_PRECISION) 

/** Define No. of Bits for Scaling Factor Divisor */
#define SCALE_PRECISION 10 // No. of Bits for Scaling Factor Divisor
#define HALF_SCALE (1 << (SCALE_PRECISION - 1)) // Half Scaling factor Used for Rounding

/** Type for Port timer values */
typedef unsigned short PORT_TIME_TYP;

/** Type containing array of Hall Phase values */
typedef struct HALL_PHASE_TAG // Structure containing Array of Hall Phase values
{
	int vals[NUM_HALL_PHASES];	// Array of Hall Phase values (NB Increment for clockwise rotation)
} HALL_PHASE_TYP;

/** Type containing all Hall test generation data */
typedef struct GENERATE_HALL_TAG // Structure containing Hall test generation data
{
	COMMON_HALL_TYP common; // Structure of Hall data common to Generator and Checker
	TEST_VECT_TYP vector; // Structure of containing Hall test vector (HALL conditions to be tested)
	HALL_PHASE_TYP phases;	// Structure containing all possible Hall phase values;
	int inc;			// Hall rotation increment (+1 for clock-wise, -1 for anti-clockwise)
	int id;			// Current motor identifier
	int hi_ticks;			// No. of ticks/HALL at high speed
	int lo_ticks;			// No. of ticks/HALL at low speed
	unsigned cnt;			// Hall position counter
	unsigned off;			// offset into Hall Phase cycle
	int orig; // Hall origin flag (Bit_2 is 1 at origin)
	int nerr; // Hall error flag (Bit_3 is 1 for NO errors)
	int scale; // velocity scaling factor (used for acceleration and deceleration)
	PORT_TIME_TYP time; // port timer value
	unsigned period; // period (in ticks) between tests
	int prev_hall;  // Previous Hall value
	int print;  // Print flag
	int dbg;  // Debug flag
} GENERATE_HALL_TYP;

/*****************************************************************************/
/** Generate Hall test data for all motors
 * \param c_tst // Channel for sending test vecotrs to test checker
 * \param p4_tst[]  // Array of ports on which to transmit test data
 */
void gen_all_hall_test_data( // Generate Hall Test data for all motors
	streaming chanend c_tst, // Channel for sending test vecotrs to test checker
	port out p4_tst[]  // Array of ports on which to transmit test data
);
/*****************************************************************************/
#endif /* _GENERATE_HALL_TESTS_H_ */
