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

//MB~ #define PERIOD_BITS 2 // No. of bits used to represent No. of period times to capture (in ADC Sine-Wave)
#define PERIOD_BITS 2 //2 No. of bits used to represent No. of period times to capture (in ADC Sine-Wave)
#define NUM_PERIODS (1 << PERIOD_BITS) // No. of period times to capture (in ADC Sine-Wave)
#define HALF_PERIODS (NUM_PERIODS >> 1) // Used for rounding

#define LO_SKIP_CHANGES 2 //3 Low No. of skipped state-changes while Sine-wave settles (used for large gain)
#define HI_SKIP_CHANGES 2 //4 High No. of skipped state-changes while Sine-wave settles (used for small gain)

/** Noise threshold used to control ADC state-change. NB Currently only noise is due to diffusion error */
#define NOISE_THRESH 1 // Noise Threshold

/** Enumeration of ADC states */
typedef enum ADC_STATE_ETAG
{
  NEGATIVE = 0,	// Negative half of ADC cycle
  POSITIVE,			// Positive half of ADC cycle
  NUM_ADC_STATES	// Handy Value!-)
} ADC_STATE_ENUM;

/** Type containing ADC statistics */
typedef struct STATS_ADC_TAG // Structure containing ADC check data
{
	unsigned sum_periods; // Accumulator for period-times
	unsigned change_time; // time-stamp for ADC state change
	unsigned half_period; // Half of period-time (between ADC-state changes)
	ADC_STATE_ENUM state; // current ADC state for this phase
	int num_changes;	// Number of recorded ADC state changes
	int done; // Flag set when enough data collected for this phase
} STATS_ADC_TYP;

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
	STATS_ADC_TYP stats[NUM_ADC_PHASES]; // Array of structures containing statistics for each ADC phase
	int motor_errs[NUM_VECT_COMPS]; // Array of error counters for one motor
	int motor_tsts[NUM_VECT_COMPS]; // Array of test counters for one motor
	ADC_PHASE_ENUM prev_phase; // Previous phase that had a state-change
	unsigned curr_time; // time value when current ADC parameters received
	unsigned prev_time; // time value when previous ADC parameters received 
	unsigned chk_period; // Check value for period duration
	unsigned speed; // Magnitude of angular velocity
	int sign; // Sign of angular velocity
	int veloc; // Angular velocity
	int skips; // Sign of angular velocity
	int all_errs; // Error accumulator
	int all_tsts; // Test accumulator
	int done;		// Flag set when test data collected for all phases
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
