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

#ifndef _TEST_QEI_COMMON_H_
#define _TEST_QEI_COMMON_H_

#include <xs1.h>
#include <assert.h>
#include <print.h>
#include <safestring.h>

#include "use_locks.h"
#include "qei_server.h"

/** Define string size */
#define STR_LEN 256

/** Define value for High Speed test */
#define HIGH_SPEED 4000

/** Define value for Low Speed test */
#define LOW_SPEED  50

/** Enumeration of QEI Test Vector Components */
typedef enum VECT_COMP_ETAG
{
  ERROR = 0,	// Error-state
  ORIGIN,			// Origin-state
  SPIN,				// Spin-state
  SPEED,			// Speed-state
  CNTRL,			// Control/Comunications state
  NUM_VECT_COMPS	// Handy Value!-)
} VECT_COMP_ENUM;

// NB Error States (ERROR_QEI_ENUM) enumerated in module_foc_qei/src/qei_common.h

/** Enumeration of QEI Origin states */
typedef enum ORIG_QEI_ETAG
{
  ORIG_OFF = 0,	// Not Origin
  ORIG_ON,			// Origin 
  NUM_QEI_ORIGS	// Handy Value!-)
} ORIG_QEI_ENUM;

/** Enumeration of QEI Spin states */
typedef enum SPIN_QEI_ETAG
{
  ANTI = 0,				// Anti-clockwise
  CLOCK = 1,				// Clock-wise
  NUM_QEI_SPINS = 2	// Handy Value!-)
} SPIN_QEI_ENUM;

/** Enumeration of QEI Speed-states */
typedef enum SPEED_QEI_ETAG
{
  ACCEL = 0,	// Accelerate
  FAST,				// No Acceleration, Fast speed
  DECEL,			// Decelerate
  SLOW,				// No Acceleration, Slow speed
  NUM_QEI_SPEEDS	// Handy Value!-)
} SPEED_QEI_ENUM;

/** Enumeration of QEI Control-states */
typedef enum CNTRL_QEI_ETAG
{
	QUIT = 0,	// Quit testing (for current motor)
  VALID,		// Valid test
  SKIP,			// Skip this test (test set-up)
  NUM_QEI_CNTRLS	// Handy Value!-)
} CNTRL_QEI_ENUM;

/** Define maximum number of states for any test vector component (used to size arrays) */
#define MAX_COMP_STATES NUM_QEI_SPEEDS	// Edit this line

/** Type containing string */
typedef struct STRING_TAG // Structure containing string array
{
	char str[STR_LEN]; // String array (NB Structure allows easy string copy)
} STRING_TYP;

/** Type containing Test Vector */
typedef struct TEST_VECT_TAG // Structure containing test vector (QEI conditions to be tested)
{
	int comp_state[NUM_VECT_COMPS]; // array containing current states for each test vector component 
} TEST_VECT_TYP;

/** Type containing Meta-information for one Test Vector */
typedef struct VECT_COMP_TAG // Structure containing common QEI test data for one test vector component
{
	STRING_TYP state_names[MAX_COMP_STATES]; // Array of names for each state of this test vector component 
	STRING_TYP comp_name; // name for this test vector component
	int num_states; // number of states for this test vector component
} VECT_COMP_TYP;

/** Type containing all Test Vector Meta-information */
typedef struct COMMON_QEI_TAG // Structure containing all common QEI test data
{
	VECT_COMP_TYP comp_data[NUM_VECT_COMPS]; // Array of data for each component of test vector
} COMMON_QEI_TYP;

/*****************************************************************************/
/** Initialise common QEI Test data
 * \param comm_qei_s, // Reference to structure of common QEI data
 */
void init_common_data( // Initialise common QEI Test data
	COMMON_QEI_TYP &comm_qei_s // Reference to structure of common QEI data
);
/*****************************************************************************/
/** Print test vector details
 * \param comm_qei_s, // Reference to structure of common QEI data
 * \param inp_vect, // Structure containing current QEI test vector to be printed
 * \param prefix_str[] // Prefix string
 */
void print_test_vector( // Print test vector details
	COMMON_QEI_TYP &comm_qei_s, // Reference to structure of common QEI data
	TEST_VECT_TYP inp_vect, // Structure containing current QEI test vector to be printed
	const char prefix_str[] // prefix string
);
/*****************************************************************************/
#endif /* _TEST_QEI_COMMON_H_ */
