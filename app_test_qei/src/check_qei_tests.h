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

#ifndef _CHECK_QEI_TESTS_H_
#define _CHECK_QEI_TESTS_H_

#include <stdlib.h>

#include <xs1.h>
#include <assert.h>
#include <print.h>
#include <safestring.h>

#include "app_global.h"
#include "use_locks.h"
#include "qei_common.h"
#include "qei_client.h"
#include "test_qei_common.h"

/** Define allowed error-status delay */
#define ERR_TIMEOUT 1 // Allowed error-status delay

/** Define allowed QEI origin-test delay */
#define ORIG_TIMEOUT 1 // Allowed QEI origin-test delay

/** Define time period between QEI Client requests for data */
#define QEI_PERIOD (40 * MICRO_SEC) // Time period between QEI Client requests for data

/** Type containing all check data */
typedef struct CHECK_QEI_TAG // Structure containing QEI check data
{
	COMMON_QEI_TYP common; // Structure of QEI data common to Generator and Checker
	char padstr1[STR_LEN]; // Padding string used to format display output
	char padstr2[STR_LEN]; // Padding string used to format display output
	TEST_VECT_TYP curr_vect; // Structure of containing current QEI test vector (QEI conditions to be tested)
	TEST_VECT_TYP prev_vect; // Structure of containing previous QEI test vector
	QEI_PARAM_TYP curr_params;	// Structure containing current QEI parameters (received from Client)
	QEI_PARAM_TYP prev_params;	// Structure containing previouis QEI parameters (received from Client)
	int motor_errs[NUM_VECT_COMPS]; // Array of error counters for one motor
	int motor_tsts[NUM_VECT_COMPS]; // Array of test counters for one motor
	int fail_cnt;	// Counter of failed tests
	int err_chk;	// error check value
	int err_cnt;	// Counter used in error test
	int orig_chk;	// origin check value
	int orig_cnt;	// Counter used in origin test
	int speed_sum; // Accumulator for speed tests
	int speed_num; // No of accumulations for speed tests
	int hi_bound; // error bound for high speed test
	int lo_bound; // error bound for low speed test
	unsigned time; // time value when new QEI parameters received
	int print;  // Print flag
	int dbg;  // Debug flag
} CHECK_QEI_TYP;

/*****************************************************************************/
/** Display QEI results for all motors
 * \param c_tst // Channel for sending test vecotrs to test checker
 * \param c_qei[]	// Array of channels connecting QEI client & server
 */
void check_all_qei_client_data( // Display QEI results for all motors
	streaming chanend c_tst, // Channel for sending test vecotrs to test checker
	streaming chanend c_qei[] // Array of QEI channels between Client and Server
);
/*****************************************************************************/
#endif /* _CHECK_QEI_TESTS_H_ */
