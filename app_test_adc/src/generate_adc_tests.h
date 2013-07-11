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
#include "sine_common.h"
#include "test_adc_common.h"

#define SIN_RES_BITS 12 // Controls Resolution of Sine table
#define NUM_SIN_VALS (1 << SIN_RES_BITS) // Number of Sine table Entries
#define MAX_SIN_INDEX (NUM_SIN_VALS - 1) // Max. table index value

#define SIN_NAME "sine_table.bin" // name of file containing binary tabulated sine values

#define SIN_BUF_SIZE (NUM_SIN_VALS * sizeof(SIN_TYP)) // Total size of Sine-table (in bytes)

#define NUM_SINx2_VALS (NUM_SIN_VALS + NUM_SIN_VALS) // Twice No of tabulated values
#define NUM_SINx3_VALS (NUM_SINx2_VALS + NUM_SIN_VALS) // Thrice No of tabulated values
#define NUM_SINx4_VALS (NUM_SINx3_VALS + NUM_SIN_VALS) // Four times No of tabulated values

#define MAX_SINx2_ID (NUM_SINx2_VALS - 1) // Max. index for 2nd Quadrant
#define MAX_SINx3_ID (NUM_SINx3_VALS - 1) // Max. index for 3rd Quadrant
#define MAX_SINx4_ID (NUM_SINx4_VALS - 1) // Max. index for 4th Quadrant

#define DIV_BITS 28 // bit shift using for integer division

#define FORTY_SECS (40UL * SECOND) // 40 seconds in clock_ticks (NB requires 32-bit unsigned)

/** Define No. of tests used for Max. speed check */
#define MAX_TESTS 31 // No. of tests used for Max. speed check

/** Define No. of tests used for Min. speed check */
#define MIN_TESTS 3  // No. of tests used for Min. speed check

/** Define No. of port timer values (16-bit) */
#define NUM_PORT_TIMES (1 << 16) // No. of port timer values (16-bit)

#define FILE_SIZE (STR_LEN * NUM_TEST_OPTS) // Size of ADC control file (in Bytes)

#if (XCC_VERSION_MAJOR >= 1200)
/** 64-bit types */
typedef signed long long S64_T;
typedef unsigned long long U64_T;
#endif // (XCC_VERSION_MAJOR >= 1200)

typedef signed long ADC_GEN_TYP; // Standard type for output of ADC Generator. NB Requires scaling to ADC_TYP

/** Type containing all ADC test generation data */
typedef struct GENERATE_TST_TAG // Structure containing ADC test generation data
{
	COMMON_ADC_TYP common; // Structure of ADC data common to Generator and Checker
	SIN_TYP table[NUM_SIN_VALS]; // Tabulated sine values
	TEST_VECT_TYP curr_vect; // Structure of containing current ADC test vector (ADC conditions to be tested)
	TEST_VECT_TYP prev_vect; // Structure of containing previous ADC test vector (ADC conditions to be tested)
	unsigned real_time; // real time-stamp (from local timer)
	unsigned curr_mtim; // Current (32-bit) mock time value
	unsigned prev_mtim; // Previous (32-bit) mock time value
	int gain; // Amplitude requested
	unsigned sum_time; // (32-bit) time value
	int sign;		// Sign of angular velocity
	unsigned speed;	// Magnitude of angular velocity
	int veloc; // Angular Velocity requested
	ADC_GEN_TYP adc_a; // Generatored ADC value for phase_A
	ADC_GEN_TYP adc_b; // Generatored ADC value for phase_B
	int scale; // Scaling factor used in bit-shift division
	int err; // rounding error
	unsigned period; // Period between generated sets of ADC stimuli
	int first; // Flag set until 1st data received
	int print;  // Print flag
	int dbg;  // Debug flag
} GENERATE_TST_TYP;

/*****************************************************************************/
/** Generate ADC test data for all motors
 * \param c_chk, // Channel for communication with Checker cores
 * \param c_adc, // Channel for communication with ADC_Interface 
 */
void gen_all_adc_test_data( // Generate ADC Test data for all motors
	streaming chanend c_chk, // Channel for communication with Checker core
	streaming chanend c_adc // Channel for communication with ADC_Interface core
);
/*****************************************************************************/
#endif /* _GENERATE_ADC_TESTS_H_ */
