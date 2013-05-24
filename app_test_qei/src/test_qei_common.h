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

#define STR_LEN 256

#define HIGH_SPEED 4000
#define LOW_SPEED  50

#define MAX_TIME 100000000 // Used in print formatting

#define PRINT 1 // Flag set for verbose printing

/** Different QEI Test Vector Components */
typedef enum VECT_COMP_ETAG
{
  ERROR = 0,	// Error-state
  ORIGIN,			// Origin-state
  SPIN,				// Spin-state
  SPEED,			// Speed-state
  NUM_VECT_COMPS	// Handy Value!-)
} VECT_COMP_ENUM;

/** Different QEI Error states */
typedef enum ERROR_QEI_ETAG
{
  ERR_OFF = 0,	// No Error
  ERR_ON,			// Error
  NUM_QEI_ERRS	// Handy Value!-)
} ERROR_QEI_ENUM;

/** Different QEI Origin states */
typedef enum ORIG_QEI_ETAG
{
  ORIG_OFF = 0,	// Not Origin
  ORIG_ON,			// Origin 
  NUM_QEI_ORIGS	// Handy Value!-)
} ORIG_QEI_ENUM;

/** Different QEI Spin states */
typedef enum SPIN_QEI_ETAG
{
  ANTI = -1,				// Anti-clockwise
  CLOCK = 1,				// Clock-wise
  NUM_QEI_SPINS = 2	// Handy Value!-)
} SPIN_QEI_ENUM;

/** Different QEI Speed-states */
typedef enum SPEED_QEI_ETAG
{
  ACCEL = 0,	// Accelerate
  FAST,				// No Acceleration, Fast speed
  DECEL,			// Decelerate
  SLOW,				// No Acceleration, Slow speed
  NUM_QEI_SPEEDS	// Handy Value!-)
} SPEED_QEI_ENUM;

/** Different QEI Control-states */
typedef enum CNTRL_QEI_ETAG
{
	QUIT = 0,	// Quit testing (for current motor)
  VALID,		// Valid test
  SKIP,			// Skip this test (test set-up)
  NUM_QEI_CNTRLS	// Handy Value!-)
} CNTRL_QEI_ENUM;

typedef struct TEST_VECT_TAG // Structure containing test vector (QEI conditions to be tested)
{
	ERROR_QEI_ENUM err;		// Error-state to be tested
	ORIG_QEI_ENUM orig;		// Origin-state to be tested
	SPIN_QEI_ENUM spin;		// Spin-state to be tested
	SPEED_QEI_ENUM speed;	// Speed-state to be tested
	CNTRL_QEI_ENUM cntrl;	// Test Command & Control value
} TEST_VECT_TYP;

#endif /* _TEST_QEI_COMMON_H_ */
