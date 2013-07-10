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

#ifndef _SINE_GENERATORS_H_
#define _SINE_GENERATORS_H_

#include <xs1.h>
#include <xccompat.h>
#include <assert.h>
#include <print.h>
#include <syscall.h>

#include "app_global.h"
#include "adc_common.h"
#include "test_adc_common.h"
#include "sine_common.h"
#include "use_locks.h"

#define CLOCK_WRAP (1 << 25) // Wrap timer after this many ticks

#define NUM_SINx2_VALS (NUM_SIN_VALS + NUM_SIN_VALS) // Twice No of tabulated values
#define NUM_SINx3_VALS (NUM_SINx2_VALS + NUM_SIN_VALS) // Thrice No of tabulated values
#define NUM_SINx4_VALS (NUM_SINx3_VALS + NUM_SIN_VALS) // Four times No of tabulated values

#define MAX_SINx2_ID (NUM_SINx2_VALS - 1) // Max. index for 2nd Quadrant
#define MAX_SINx3_ID (NUM_SINx3_VALS - 1) // Max. index for 3rd Quadrant
#define MAX_SINx4_ID (NUM_SINx4_VALS - 1) // Max. index for 4th Quadrant

#define DIV_BITS 28 // bit shift using for integer division

#define FORTY_SECS (40UL * SECOND) // 40 seconds in clock_ticks (NB requires 32-bit unsigned)

#if (XCC_VERSION_MAJOR >= 1200)
/** 64-bit types */
typedef signed long long S64_T;
typedef unsigned long long U64_T;
#endif // (XCC_VERSION_MAJOR >= 1200)

typedef signed long ADC_GEN_TYP; // Standard type for output of ADC Generator. NB Requires scaling to ADC_TYP
 
/** Type containing all Sine generation data */
typedef struct SINE_TST_TAG // Structure containing ADC test generation data
{
	SIN_TYP table[NUM_SIN_VALS]; // Tabulated sine values
	TEST_VECT_TYP curr_vect; // Structure of containing current ADC test vector (ADC conditions to be tested)
	TEST_VECT_TYP prev_vect; // Structure of containing previous ADC test vector (ADC conditions to be tested)
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
	int first; // Flag set until 1st data received
} SINE_TST_TYP;

/*****************************************************************************/
/** Transmits a Sine value having received a velocity and time value
 * \param c_tst, // Channel for communication with Test_Generator
 * \param c_chk // Channel for communicating with Checker core
 * \param c_adc // Channel for communicating with ADC_Interface_core
 */
void get_sine_data( // Transmits a Sine value having received a velocity and time value
	streaming chanend c_tst, // Channel for communication with Test_Generator
	streaming chanend c_chk, // Channel for communication with Checker core
	streaming chanend c_adc // Channel for communicating with ADC_Interface core
);
/*****************************************************************************/
#endif /* _SINE_GENERATORS_H_ */
