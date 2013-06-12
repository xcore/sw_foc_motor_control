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

#ifndef _TEST_PWM_COMMON_H_
#define _TEST_PWM_COMMON_H_

#include <xs1.h>
#include <assert.h>
#include <print.h>
#include <safestring.h>

#include "use_locks.h"
#include "app_global.h"
#include "pwm_common.h"

/** Define string size */
#define STR_LEN 256

/** Define PWM-Phase under test */
#define TEST_PHASE PWM_PHASE_A

/** Define PWM-Phase under test */
#define TEST_LEG PWM_HI_LEG

/** Define value for Low Speed test */
#define MIN_PWM (PWM_MAX_VALUE >> 3) 

/** Define value for High Speed test */
#define MAX_PWM (PWM_MAX_VALUE - MIN_PWM)

/** Enumeration of PWM Test Vector Components */
typedef enum VECT_COMP_ETAG
{
  CNTRL = 0,	// Special Case: Control/Comunications state
  WIDTH,			// PWM Width-state
  PHASE,			// PWM-Phase
  LEG,				// PWM-Leg
  NUM_VECT_COMPS	// Handy Value!-)
} VECT_COMP_ENUM;

/** Enumeration of PWM Width-states */
typedef enum WIDTH_PWM_ETAG
{
  LARGE = 0,	// Large PWM width (for Fast Speed)
  SMALL,			// Smaell PWM width (for Slow Speed)
  NUM_PWM_WIDTHS	// Handy Value!-)
} WIDTH_PWM_ENUM;

/** Enumeration of PWM Control-states */
typedef enum CNTRL_PWM_ETAG
{
	QUIT = 0,	// Quit testing (for current motor)
  VALID,		// Valid test
  SKIP,			// Skip this test (test set-up)
  NUM_PWM_CNTRLS	// Handy Value!-)
} CNTRL_PWM_ENUM;

// NB Enumeration of PWM Phase-states in sc_pwm/module_foc_pwm/src/pwm_common.h

/** Define maximum number of states for any test vector component (used to size arrays) */
#define MAX_COMP_STATES NUM_PWM_CNTRLS	// Edit this line

/** Type containing string */
typedef struct STRING_TAG // Structure containing string array
{
	char str[STR_LEN]; // String array (NB Structure allows easy string copy)
} STRING_TYP;

/** Type containing Test Vector */
typedef struct TEST_VECT_TAG // Structure containing test vector (PWM conditions to be tested)
{
	int comp_state[NUM_VECT_COMPS]; // array containing current states for each test vector component 
} TEST_VECT_TYP;

/** Type containing Meta-information for one Test Vector */
typedef struct VECT_COMP_TAG // Structure containing common PWM test data for one test vector component
{
	STRING_TYP state_names[MAX_COMP_STATES]; // Array of names for each state of this test vector component 
	STRING_TYP comp_name; // name for this test vector component
	int num_states; // number of states for this test vector component
} VECT_COMP_TYP;

/** Type containing all Test Vector Meta-information */
typedef struct COMMON_PWM_TAG // Structure containing all common PWM test data
{
	VECT_COMP_TYP comp_data[NUM_VECT_COMPS]; // Array of data for each component of test vector
} COMMON_PWM_TYP;

/*****************************************************************************/
/** Initialise common PWM Test data
 * \param comm_pwm_s, // Reference to structure of common PWM data
 */
void init_common_data( // Initialise common PWM Test data
	COMMON_PWM_TYP &comm_pwm_s // Reference to structure of common PWM data
);
/*****************************************************************************/
/** Print test vector details
 * \param comm_pwm_s, // Reference to structure of common PWM data
 * \param inp_vect, // Structure containing current PWM test vector to be printed
 * \param prefix_str[] // Prefix string
 */
void print_test_vector( // Print test vector details
	COMMON_PWM_TYP &comm_pwm_s, // Reference to structure of common PWM data
	TEST_VECT_TYP inp_vect, // Structure containing current PWM test vector to be printed
	const char prefix_str[] // prefix string
);
/*****************************************************************************/
#endif /* _TEST_PWM_COMMON_H_ */
