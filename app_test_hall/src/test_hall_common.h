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

#ifndef _TEST_HALL_COMMON_H_
#define _TEST_HALL_COMMON_H_

#include <xs1.h>
#include <assert.h>
#include <print.h>
#include <safestring.h>

#include "use_locks.h"
#include "hall_server.h"

#define STR_LEN 256

#define HIGH_SPEED 4000
#define LOW_SPEED  50

/** Different Hall Test Vector Components */
typedef enum VECT_COMP_ETAG
{
  ERROR = 0,	// Error-state
  ORIGIN,			// Origin-state
  SPIN,				// Spin-state
  SPEED,			// Speed-state
  CNTRL,			// Control/Comunications state
  NUM_VECT_COMPS	// Handy Value!-)
} VECT_COMP_ENUM;

// NB Error States (ERROR_HALL_ENUM) defined in module_foc_hall/src/hall_common.h

/** Different Hall Origin states */
typedef enum ORIG_HALL_ETAG
{
  ORIG_OFF = 0,	// Not Origin
  ORIG_ON,			// Origin 
  NUM_HALL_ORIGS	// Handy Value!-)
} ORIG_HALL_ENUM;

/** Different Hall Spin states */
typedef enum SPIN_HALL_ETAG
{
  ANTI = 0,				// Anti-clockwise
  CLOCK = 1,				// Clock-wise
  NUM_HALL_SPINS = 2	// Handy Value!-)
} SPIN_HALL_ENUM;

/** Different Hall Speed-states */
typedef enum SPEED_HALL_ETAG
{
  ACCEL = 0,	// Accelerate
  FAST,				// No Acceleration, Fast speed
  DECEL,			// Decelerate
  SLOW,				// No Acceleration, Slow speed
  NUM_HALL_SPEEDS	// Handy Value!-)
} SPEED_HALL_ENUM;

/** Different Hall Control-states */
typedef enum CNTRL_HALL_ETAG
{
	QUIT = 0,	// Quit testing (for current motor)
  VALID,		// Valid test
  SKIP,			// Skip this test (test set-up)
  NUM_HALL_CNTRLS	// Handy Value!-)
} CNTRL_HALL_ENUM;

#define MAX_COMP_STATES NUM_HALL_SPEEDS	// Edit this line

typedef struct STRING_TAG // Structure containing Hall test data
{
	char str[STR_LEN]; // String array (NB Structure allows easy string copy)
} STRING_TYP;

typedef struct TEST_VECT_TAG // Structure containing test vector (HALL conditions to be tested)
{
	int comp_state[NUM_VECT_COMPS]; // array containing current states for each test vector component 
} TEST_VECT_TYP;

typedef struct VECT_COMP_TAG // Structure containing common Hall test data for one test vector component
{
	STRING_TYP state_names[MAX_COMP_STATES]; // Array of names for each state of test vector component 
	STRING_TYP comp_name; // names for test vector component
	int num_states; // number of states for this test vector component
} VECT_COMP_TYP;

typedef struct COMMON_HALL_TAG // Structure containing all common Hall test data
{
	VECT_COMP_TYP comp_data[NUM_VECT_COMPS]; // Array of data for each component of test vector
} COMMON_HALL_TYP;

/*****************************************************************************/
void init_common_data( // Initialise Hall Test data
	COMMON_HALL_TYP &comm_hall_s // Reference to structure of common Hall data
);
/*****************************************************************************/
void print_test_vector( // print test vector details
	COMMON_HALL_TYP &comm_hall_s, // Reference to structure of common Hall data
	TEST_VECT_TYP inp_vect, // Structure containing current Hall test vector to be printed
	const char prefix_str[] // prefix string
);
/*****************************************************************************/
#endif /* _TEST_HALL_COMMON_H_ */
