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

#ifndef _GENERATE_QEI_TESTS_H_
#define _GENERATE_QEI_TESTS_H_

#include <stdlib.h>

#include <xs1.h>
#include <assert.h>
#include <print.h>

#include "app_global.h"
#include "use_locks.h"
#include "qei_common.h"
#include "test_qei_common.h"

#define QEI_PHASE_NUM (QEI_PHASE_MASK + 1) // Number of QEI Phases

#define HIGH_SPEED 4000
#define LOW_SPEED  50

#define MAX_TESTS 30 // No. of tests used for Max. speed check
#define MIN_TESTS 3  // No. of tests used for Min. speed check
#define ACC_TESTS 18 // No. of tests used for Acceleration check
#define DEC_TESTS 18 // No. of tests used for Deceleration check

#define STR_LEN 256

#define SCALE_PRECISION 10 // No. of Bits for Scaling Factor Divisor
#define HALF_SCALE (1 << (SCALE_PRECISION - 1)) // Half Scaling factor Used for Rounding

#define ACC_SCALE 807 // Scaling factor Used for Acceleration (ACC_SCALE >> SCALE_PRECISION) 
#define DEC_SCALE 1300 // Scaling factor Used for Deceleration (DEC_SCALE >> SCALE_PRECISION) 

typedef struct QEI_PHASE_TAG // Structure containing Array of QEI Phase values
{
	int vals[QEI_PHASE_NUM];	// Array of QEI Phase values
} QEI_PHASE_TYP;

typedef struct TEST_QEI_TAG // Structure containing QEI test data
{
	QEI_PHASE_TYP clk_wise;	// Structure of QEI phase values for clockwise rotation
	QEI_PHASE_TYP anti_clk;	// Structure of QEI phase values for anti-clockwise rotation
	QEI_PHASE_TYP phase;	// Structure of currently used QEI phase values
	int hi_ticks;			// No. of ticks/QEI at high speed
	int lo_ticks;			// No. of ticks/QEI at low speed
	int off;			// offset into QEI Phase cycle
	int cnt; // QEI position counter
	unsigned time; // previous timer value
	unsigned period; // period (in ticks) between tests
} TEST_QEI_TYP;

/*****************************************************************************/
/** Generate QEI test data for all motors
 * \param p4_tst[]  // Array of ports on which to transmit test data
 */
void gen_all_qei_test_data( // Generate QEI Test data for all motors
	port out p4_tst[]  // Array of ports on which to transmit test data
);
/*****************************************************************************/
#endif /* _GENERATE_QEI_TESTS_H_ */
