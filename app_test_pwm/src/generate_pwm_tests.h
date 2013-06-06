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

#ifndef _GENERATE_PWM_TESTS_H_
#define _GENERATE_PWM_TESTS_H_

#include <stdlib.h>

#include <xs1.h>
#include <assert.h>
#include <print.h>
#include <safestring.h>

#include "app_global.h"
#include "use_locks.h"
#include "pwm_common.h"
#include "pwm_client.h"
#include "test_pwm_common.h"

/** Define PWM Width for 'Maximum test' */
#define MAX_PWM 3000 // PWM Width for 'Maximum test'

/** Define PWM Width for 'Minimum test' */
#define MIN_PWM 500 // PWM Width for 'Minimum test'

/** Define time period between generations of PWM Client data */
#define PWM_PERIOD (40 * MICRO_SEC) // Time period between generations of PWM Client data

/** Define No. of tests used for Max. speed check */
#define MAX_TESTS 4 // No. of tests used for Max. speed check

/** Define No. of tests used for Min. speed check */
#define MIN_TESTS 4 // No. of tests used for Min. speed check

/** Define No. of port timer values (16-bit) */
#define NUM_PORT_TIMES (1 << 16) // No. of port timer values (16-bit)

/** Define No. of Bits for Scaling Factor Divisor */
#define SCALE_PRECISION 10 // No. of Bits for Scaling Factor Divisor
#define HALF_SCALE (1 << (SCALE_PRECISION - 1)) // Half Scaling factor Used for Rounding

/** Type containing all PWM test generation data */
typedef struct GENERATE_PWM_TAG // Structure containing PWM test generation data
{
	COMMON_PWM_TYP common; // Structure of PWM data common to Generator and Checker
	TEST_VECT_TYP vector; // Structure of containing PWM test vector (PWM conditions to be tested)
	PWM_COMMS_TYP pwm_comms;	// Structure containing current PWM communication data (sent to Server)
	int scale; // velocity scaling factor (used for acceleration and deceleration)
	unsigned time; // timer value
	unsigned period; // period (in ticks) between tests
	unsigned width; // PWM width
	int print;  // Print flag
	int dbg;  // Debug flag
} GENERATE_PWM_TYP;

/*****************************************************************************/
/** Generate PWM test data for all motors
 * \param c_tst // Channel for sending test vecotrs to test checker
 * \param c_pwm // Channel between Client and Server
 */
void gen_all_pwm_test_data( // Generate PWM Test data for all motors
	streaming chanend c_tst, // Channel for sending test vecotrs to test checker
	chanend c_pwm 				// Channel between Client and Server
);
/*****************************************************************************/
#endif /* _GENERATE_PWM_TESTS_H_ */
