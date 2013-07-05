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

#ifndef _GENERATE_ADC_TESTS_H_
#define _GENERATE_ADC_TESTS_H_

#include <xs1.h>
#include <xccompat.h>
#include <assert.h>
#include <print.h>
#include <safestring.h>
#include <syscall.h>

#include "app_global.h"
#include "use_locks.h"
#include "adc_common.h"
#include "test_adc_common.h"

/** Define No. of tests used for Max. speed check */
#define MAX_TESTS 31 // No. of tests used for Max. speed check

/** Define No. of tests used for Min. speed check */
#define MIN_TESTS 3  // No. of tests used for Min. speed check

/** Define No. of port timer values (16-bit) */
#define NUM_PORT_TIMES (1 << 16) // No. of port timer values (16-bit)

#define FILE_SIZE (STR_LEN * NUM_TEST_OPTS) // Size of ADC control file (in Bytes)

/** Type containing all ADC test generation data */
typedef struct GENERATE_TST_TAG // Structure containing ADC test generation data
{
	COMMON_ADC_TYP common; // Structure of ADC data common to Generator and Checker
	TEST_VECT_TYP curr_vect; // Structure of containing current ADC test vector (ADC conditions to be tested)
	TEST_VECT_TYP prev_vect; // Structure of containing previous ADC test vector (ADC conditions to be tested)
	unsigned period; // Period between generated sets of ADC stimuli
	int print;  // Print flag
	int dbg;  // Debug flag
} GENERATE_TST_TYP;

/*****************************************************************************/
/** Generate ADC test data for all motors
 * \param c_chk, // Channel for communication with Checker cores
 * \param c_adc, // Channel for communication with ADC_Interface 
 * \param c_sin // Channel for communication with Sine_Generator cores
 */
void gen_all_adc_test_data( // Generate ADC Test data for all motors
	streaming chanend c_chk, // Channel for communication with Checker cores
	streaming chanend c_sin // Channel for communication with Sine_Generator cores
);
/*****************************************************************************/
#endif /* _GENERATE_ADC_TESTS_H_ */
