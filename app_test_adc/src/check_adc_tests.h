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

#ifndef _CHECK_ADC_TESTS_H_
#define _CHECK_ADC_TESTS_H_

#include <stdlib.h>

#include <xs1.h>
#include <assert.h>
#include <print.h>
#include <safestring.h>

#include "app_global.h"
#include "use_locks.h"
#include "adc_common.h"
#include "adc_client.h"
#include "test_adc_common.h"

/** Type containing all check data */
typedef struct CHECK_ADC_TAG // Structure containing ADC check data
{
	COMMON_ADC_TYP common; // Structure of ADC data common to Generator and Checker
	char padstr1[STR_LEN]; // Padding string used to format display output
	char padstr2[STR_LEN]; // Padding string used to format display output
	TEST_VECT_TYP curr_vect; // Structure of containing current ADC test vector (ADC conditions to be tested)
	TEST_VECT_TYP prev_vect; // Structure of containing previous ADC test vector
	ADC_PARAM_TYP curr_params;	// Structure containing current ADC parameters (received from Client)
	ADC_PARAM_TYP prev_params;	// Structure containing previouis ADC parameters (received from Client)
	int motor_errs[NUM_VECT_COMPS]; // Array of error counters for one motor
	int motor_tsts[NUM_VECT_COMPS]; // Array of test counters for one motor
	unsigned curr_time; // time value when current ADC parameters received
	unsigned prev_time; // time value when previous ADC parameters received 
	int all_errs; // Error accumulator
	int all_tsts; // Test accumulator
	int fail_cnt;	// Counter of failed tests
	int speed_sum; // Accumulator for speed tests
	int speed_num; // No of accumulations for speed tests
	int hi_bound; // error bound for high speed test
	int lo_bound; // error bound for low speed test
	int more; // Flag set if more data required for test
	int print;  // Print flag
	int dbg;  // Debug flag
} CHECK_ADC_TYP;

/*****************************************************************************/
/** Display ADC results for all motors
 * \param c_adc[],	// Array of channels for communication with ADC_Server
 * \param c_sin, // Channel for communication with Sine_Generator cores
 * \param c_tst // Channel for communication with Test_Generator
 */
void check_all_adc_client_data( // Display ADC results for all motors
	streaming chanend c_adc[], // Array of channel for communication with ADC_Server
	streaming chanend c_sin, // Channel for communication with Sine_Generator cores
	streaming chanend c_tst // Channel for communication with Test_Generator
);
/*****************************************************************************/
#endif /* _CHECK_ADC_TESTS_H_ */
