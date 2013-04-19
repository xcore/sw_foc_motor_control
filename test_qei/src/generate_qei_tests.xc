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

#include "generate_qei_tests.h"

/*	Order is 00 -> 01 -> 11 -> 10  Clockwise direction
 *	Order is 00 -> 10 -> 11 -> 01  Anti-Clockwise direction
 */
/*****************************************************************************/
static void init_qei_data( // Initialise QEI Test data
	TEST_QEI_TYP &tst_data_s // Reference to structure of QEI test data
)
{
	QEI_CYC_TYP forward = {{ 0 ,1 ,3 ,2 }};	// Forward QEI Cycle
	QEI_CYC_TYP reverse = {{ 0 ,2 ,3 ,1 }};	// Reverse QEI Cycle

	tst_data_s.f_cyc = forward; // Assign forward rotation cycle
	tst_data_s.r_cyc = reverse; // Assign reverse rotation cycle
	tst_data_s.cyc_off = 0; // Set QEI cycle to start

} // init_qei_data
/*****************************************************************************/
static void gen_motor_qei_test_data( // Generate QEI Test data for one motor
	TEST_QEI_TYP tst_data_s, // Structure of QEI test data
	int motor_id, // Motors Identifier
	port out p4_tst  // current port on which to transmit test data
)
{
	int test_cnt; // test counter
	int qei_val;  // QEI value [I B A]
	timer chronometer; // XMOS timer
	unsigned time_val;   // time value
	unsigned time_inc;   // time increment


	chronometer :> time_val; // Get start time
	time_inc = (586 * MICRO_SEC); // Speed = 100 RPM

	for (test_cnt=0; test_cnt<NUM_TESTS; test_cnt++)
	{
		qei_val = tst_data_s.f_cyc.vals[tst_data_s.cyc_off];

		tst_data_s.cyc_off++;
		tst_data_s.cyc_off &= QEI_CYCLE_MSK; // wrap cycle count

// if (10 == test_cnt) qei_val += QEI_CYCLE_LEN; // Signal Origin 
		p4_tst <: qei_val;

		chronometer when timerafter(time_val + time_inc) :> time_val;
		time_inc -= (MICRO_SEC << 2); // Increase speed
	} // for test_cnt

} // gen_motor_qei_test_data
/*****************************************************************************/
void gen_all_qei_test_data( // Generate QEI Test data
	port out p4_tst[]  // Array of ports on which to transmit test data
)
{
	TEST_QEI_TYP tst_data_s; // Structure of QEI test data
	int motor_cnt; // counts Motors 


	init_qei_data( tst_data_s );

	// Loop through motors, so we can print results serially
	for (motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++)
	{
		gen_motor_qei_test_data( tst_data_s ,motor_cnt ,p4_tst[motor_cnt] );
	} // for motor_cnt

} // gen_all_qei_test_data
/*****************************************************************************/
