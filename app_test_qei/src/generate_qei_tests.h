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

#include <xs1.h>
#include <assert.h>
#include <print.h>
#include <safestring.h>
#include <syscall.h>

#include "app_global.h"
#include "use_locks.h"
#include "qei_common.h"
#include "qei_server.h"
#include "test_qei_common.h"
#include "master_print_scheduler.h"

/** Define No. of tests used for Max. speed check */
#define MAX_TESTS 31 // No. of tests used for Max. speed check

/** Define No. of tests used for Min. speed check */
#define MIN_TESTS 3  // No. of tests used for Min. speed check

/** Define No. of tests used for Acceleration check */
#define ACC_TESTS 18 // No. of tests used for Acceleration check

/** Define No. of tests used for Deceleration check */
#define DEC_TESTS 18 // No. of tests used for Deceleration check

/** Define Scaling factor Used for Acceleration */
#define ACC_SCALE 935 // 807 Scaling factor Used for Acceleration (ACC_SCALE >> SCALE_PRECISION) 

/** Define Scaling factor Used for Deceleration */
#define DEC_SCALE 1104 // 1300 Scaling factor Used for Deceleration (DEC_SCALE >> SCALE_PRECISION) 

#define FILE_SIZE (STR_LEN * NUM_TEST_OPTS) // Size of PWM control file (in Bytes)

#define MIN_WAIT_TIME 440 // Minimum wait time in Port-Timer ticks

/** Define No. of port timer values (16-bit) */
#define NUM_PORT_TIMES (1 << INT16_BITS) // No. of port timer values (16-bit)
#define HALF_PORT_TIMES (NUM_PORT_TIMES >> 1) // Half No. of port timer values

/** Define No. of Bits for Scaling Factor Divisor */
#define SCALE_PRECISION 10 // No. of Bits for Scaling Factor Divisor
#define HALF_SCALE (1 << (SCALE_PRECISION - 1)) // Half Scaling factor Used for Rounding

/** Type for Port timer values */
typedef unsigned short PORT_TIME_TYP;

/** Type containing all QEI test generation data */
typedef struct GENERATE_QEI_TAG // Structure containing QEI test generation data
{
	TEST_OPTS_TYP options; // Structure of test_option data
	TEST_VECT_TYP curr_vect; // Structure of containing current QEI test vector (QEI conditions to be tested)
	TEST_VECT_TYP prev_vect; // Structure of containing previous QEI test vector (QEI conditions to be tested)
	QEI_PHASE_TYP phases;	// Structure containing all possible QEI phase values;
	int inc;			// QEI rotation increment (+1 for clock-wise, -1 for anti-clockwise)
	int hi_ticks;			// No. of ticks/QEI at high speed
	int lo_ticks;			// No. of ticks/QEI at low speed
	unsigned cnt;			// QEI position counter
	unsigned off;			// offset into QEI Phase cycle
	unsigned rnd;		// Random No. from CRC
	int orig; // QEI origin flag (Bit_2 is 1 at origin)
	int nerr; // QEI error flag (Bit_3 is 1 for NO errors)
	int scale; // velocity scaling factor (used for acceleration and deceleration)
	PORT_TIME_TYP time_p; // port timer transmit time for QEI data
	unsigned period; // period (in ticks) between tests
	unsigned tim; // time (in ticks) of test
	unsigned tim2; // time (in ticks) of test
	QEI_RAW_TYP prev_qei;  // Previous QEI value
	int print;  // Print flag
	int dbg;  // Debug flag
	unsigned tx_time; // Time of QEI data transmission from Big Timer
	timer Big_Timer; // Big Timer
} GENERATE_TST_TYP;

/*****************************************************************************/
/** Generate QEI test data for all motors
 * \param c_tst // Channel for sending test vecotrs to test checker
 * \param c_disp // Channel for sending display data to print scheduler core
 * \param p4_tst[]  // Array of ports on which to transmit test data
 */
void gen_all_qei_test_data( // Generate QEI Test data for all motors
	const COMMON_TST_TYP &comm_data_s, // Structure containing common test data
	streaming chanend c_tst, // Channel for sending test vecotrs to test checker
	streaming chanend c_disp, // Channel for sending display data to print scheduler core
	buffered port:4 out pb4_tst[]  // Array of buffered 4-bit output ports on which to transmit test data
);
/*****************************************************************************/
#endif /* _GENERATE_QEI_TESTS_H_ */
