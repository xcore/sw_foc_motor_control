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

#define PERIOD_BITS 2 //2 No. of bits used to represent No. of period times to capture (in ADC Sine-Wave)
#define NUM_PERIODS (1 << PERIOD_BITS) // No. of period times to capture (in ADC Sine-Wave)
#define HALF_PERIODS (NUM_PERIODS >> 1) // Used for rounding

#define LO_SKIP_CHANGES 4 // 4 Low No. of skipped state-changes while Sine-wave settles (used for large gain)
#define HI_SKIP_CHANGES 4 // 4 High No. of skipped state-changes while Sine-wave settles (used for small gain)

/** Noise threshold used to control ADC state-change. NB Currently only noise is due to diffusion error */
#define NOISE_THRESH 1 // Noise Threshold

/** Minimum possible ADC value */
#define MIN_ADC_VAL (-(1 << (ADC_ACTIVE_BITS - 1))) // Currently -2048

/** Maximum possible ADC value */
#define MAX_ADC_VAL (-1 -MIN_ADC_VAL) // Currently -2048

#define LARGE_BOUND 600000 // Zero-mean error-bound for Large Gain
#define SMALL_BOUND 1600000 // Zero-mean error-bound for Small Gain

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
	unsigned half_period; // Half of period-time (between ADC-state changes)
	int sum_adcs; 		// Accumulator for ADC values
	int half_adc; 		// Sum of ADC values over half a period (between ADC-state changes)
	unsigned change_time; // time-stamp for ADC state change
	ADC_STATE_ENUM state; // current ADC state for this phase
	ADC_TYP max; // Maximum measured ADC value
	ADC_TYP min; // Minimum measured ADC value
	int num_changes;	// Number of recorded ADC state changes
	int done; // Flag set when enough data collected for this phase
} STATS_ADC_TYP;

/** Type containing all check data */
typedef struct CHECK_TST_TAG // Structure containing ADC check data
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
	int chk_ampli; // Theoretical maximum peak-to-peak amplitude of ADC wave-train
	unsigned period_bound; // Error bound for period-duration check
	unsigned mean_bound; // Error bound for zero-mean check
	int ampli_bound; // Error bound for ADC gain check
	unsigned speed; // Magnitude of angular velocity
	int sign; // Sign of angular velocity
	int veloc; // Angular velocity
	int skips; // Sign of angular velocity
	int all_errs; // Error accumulator
	int all_tsts; // Test accumulator
	int done;		// Flag set when test data collected for all phases
	int print_on;  // Print flag
	int print_cnt; // Print counter
	int dbg;  // Debug flag
} CHECK_TST_TYP;

/*****************************************************************************/
/** Display ADC results for all motors
 * \param c_adc[],	// Array of channels for communication with ADC_Client cores
 * \param c_tst // Channel for communication with Test_Generator core
 */
void check_all_adc_client_data( // Display ADC results for all motors
	streaming chanend c_adc[], // Array of channel for communication with ADC_Client cores
	streaming chanend c_gen // Channel for communication with Test_Generator core
);
/*****************************************************************************/
#endif /* _CHECK_ADC_TESTS_H_ */
