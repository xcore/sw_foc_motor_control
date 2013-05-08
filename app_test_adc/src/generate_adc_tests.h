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

#include <stdlib.h>

#include <xs1.h>
#include <assert.h>
#include <print.h>

#include "app_global.h"
#include "use_locks.h"
#include "maths_functions.h"
#include "adc_common.h"
#include "test_adc_common.h"

#define STR_LEN 256

#define MAX_TESTS 32

#define WAVE_RESOLUTION 5 // 10 No. of bits used to define sample index
#define NUM_WAVE_SAMPS (1 << WAVE_RESOLUTION) // No. Of Samples used in one wave period
#define MAX_WAVE_ID (NUM_WAVE_SAMPS - 1) // Max. Samples index

typedef struct ADC_WAVE_TAG // Structure containing data for one wave period
{
	int samps[NUM_WAVE_SAMPS];	// Array of samples for one wave period 
} ADC_WAVE_TYP;

typedef struct TEST_WAVE_TAG // Structure containing all wave data
{
	ADC_WAVE_TYP sine; // sine-wave samples
} TEST_WAVE_TYP;

typedef struct TEST_ADC_TAG // Structure containing ADC test data
{
	int vals[NUM_ADC_PHASES];	// Array of ADC values for each phase
	int offsets[NUM_ADC_PHASES];	// Array of ADC Phase offsets
	int cnt; // test counter
	int id; // Motors Identifier
	unsigned time; // previous timer value
	unsigned period; // period (in ticks) between tests
} TEST_ADC_TYP;

/*****************************************************************************/
/** Generate ADC test data for all motors
 * \param pb32_tst // Array of 32-bit buffered ports outputing test ADC values 
 * \param c_pwm2adc_trig // Array of channels outputting ADC trigger 
 */
void gen_all_adc_test_data( // Generate ADC Test data for all motors
	buffered out port:32 pb32_tst[][NUM_ADC_PHASES], // Array of 32-bit buffered ports outputing test ADC values 
	chanend c_pwm2adc_trig[] // Array of channels outputting ADC trigger 
);
/*****************************************************************************/
#endif /* _GENERATE_ADC_TESTS_H_ */
