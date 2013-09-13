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
#include "master_print_scheduler.h"

/** Define allowed error-status delay */
#define ERR_TIMEOUT 1 // Allowed error-status delay

/** Define allowed QEI origin-test delay */
#define ORIG_TIMEOUT 1 // Allowed QEI origin-test delay

/** Define time period between QEI Client requests for data */
#define QEI_PERIOD (40 * MICRO_SEC) // Time period between QEI Client requests for data

#define HALF_QEI_POS (QEI_PER_REV >> 1) // Half No. of QEI positions

#define VECT_BUF_BITS 2 // No. of bits used to represent vector buffer size
#define VECT_BUF_SIZ (1 << VECT_BUF_BITS) // vector buffer size
#define VECT_BUF_MASK (VECT_BUF_SIZ  - 1) // Used to wrap buffer offsets

// No. of bits used to represent down-scaling factor used to generate speed-err bound
// #define SPEED_ERR_BITS 4 // NB 4 gives 6.25% error
#define SPEED_ERR_BITS 6 // NB 6 gives 1.56% error

/** Type containing all check data */
typedef struct CHECK_QEI_TAG // Structure containing QEI check data
{
	TEST_OPTS_TYP options; // Structure of test_option data
	TEST_VECT_TYP curr_vect; // Structure of containing current QEI test vector (QEI conditions to be tested)
	TEST_VECT_TYP prev_vect; // Structure of containing previous QEI test vector
	QEI_PARAM_TYP curr_params;	// Structure containing current QEI parameters (received from Client)
	QEI_PARAM_TYP prev_params;	// Structure containing previouis QEI parameters (received from Client)
	STRING_TYP prefix; // prefix string for each display output
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
	int ang_bound;	// angular_position margin of error
	unsigned time; // time value when new QEI parameters received
	int print_on;  // Print flag
	int dbg;  // Debug flag
} CHECK_TST_TYP;

/*****************************************************************************/
/** Display QEI results for all motors
 * \param c_tst // Channel for sending test vecotrs to test checker
 * \param c_qei[]	// Array of channels connecting QEI client & server
 * \param c_disp // Channel for sending display data to print scheduler core
 */
void check_all_qei_client_data( // Display QEI results for all motors
	const COMMON_TST_TYP &comm_data_s, // Structure containing common test data
	streaming chanend c_tst, // Channel for sending test vecotrs to test checker
	streaming chanend c_qei[], // Array of QEI channels between Client and Server
	streaming chanend c_disp // Channel for sending display data to print scheduler core
);
/*****************************************************************************/
#endif /* _CHECK_QEI_TESTS_H_ */
