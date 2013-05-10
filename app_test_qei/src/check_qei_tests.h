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
#include "test_qei_common.h"
#include "qei_client.h"

typedef struct CHECK_QEI_TAG // Structure containing QEI check data
{
	char names[NUM_VECT_COMPS][STR_LEN]; // Array of names for each component of test vector
	TEST_VECT_TYP vector; // Structure of containing QEI test vector (QEI conditions to be tested)
	QEI_PARAM_TYP params;	// Structure containing QEI parameters (received from Client)
	int motor_errs[NUM_VECT_COMPS]; // Array of error counters for one motor
	int motor_tsts[NUM_VECT_COMPS]; // Array of test counters for one motor
	int all_errs[NUMBER_OF_MOTORS]; // Array of Error accumulators for each motor
	int all_tsts[NUMBER_OF_MOTORS]; // Array of Test accumulators for each motor
 	int id; // Unique motor identifier
 	int prev_ang; // Previous QEI angular position
	int fail_cnt;	// Counter of failed tests
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
