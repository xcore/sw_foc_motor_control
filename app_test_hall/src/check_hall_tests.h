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

#ifndef _CHECK_HALL_TESTS_H_
#define _CHECK_HALL_TESTS_H_

#include <stdlib.h>

#include <xs1.h>
#include <assert.h>
#include <print.h>
#include <safestring.h>

#include "app_global.h"
#include "use_locks.h"
#include "hall_common.h"
#include "hall_client.h"
#include "test_hall_common.h"

/** Define allowed error-status delay */
#define ERR_TIMEOUT 1 // Allowed error-status delay

/** Define time period between Hall Client requests for data */
#define HALL_PERIOD (40 * MICRO_SEC) // Time period between Hall Client requests for data

/** Type containing all check data */
typedef struct CHECK_HALL_TAG // Structure containing Hall check data
{
	COMMON_TST_TYP common; // Structure of Hall data common to Generator and Checker
	char padstr1[STR_LEN]; // Padding string used to format display output
	char padstr2[STR_LEN]; // Padding string used to format display output
	TEST_VECT_TYP curr_vect; // Structure of containing current Hall test vector (HALL conditions to be tested)
	TEST_VECT_TYP prev_vect; // Structure of containing previous Hall test vector
	HALL_PARAM_TYP curr_params;	// Structure containing current Hall parameters (received from Client)
	HALL_PARAM_TYP prev_params;	// Structure containing previouis Hall parameters (received from Client)
	int motor_errs[NUM_VECT_COMPS]; // Array of error counters for one motor
	int motor_tsts[NUM_VECT_COMPS]; // Array of test counters for one motor
	char disp_str[HALL_BITS]; // display string for Hall value
	int fail_cnt;	// Counter of failed tests
	int err_chk;	// error check value
	int err_cnt;	// Counter used in error test
	unsigned off_diff; // Difference in phase offset values (NB always +ve)
	unsigned time; // time value when new Hall parameters received
	int print;  // Print flag
	int dbg;  // Debug flag
} CHECK_TST_TYP;

/*****************************************************************************/
/** Display Hall results for all motors
 * \param c_tst // Channel for sending test vecotrs to test checker
 * \param c_hall[]	// Array of channels connecting Hall client & server
 */
void check_all_hall_client_data( // Display Hall results for all motors
	streaming chanend c_tst, // Channel for sending test vecotrs to test checker
	streaming chanend c_hall[] // Array of Hall channels between Client and Server
);
/*****************************************************************************/
#endif /* _CHECK_HALL_TESTS_H_ */
