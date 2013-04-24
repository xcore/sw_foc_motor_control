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

/*	[BA] order is 00 -> 01 -> 11 -> 10  Clockwise direction
 *	[BA] order is 00 -> 10 -> 11 -> 01  Anti-Clockwise direction
 */
/*****************************************************************************/
static unsigned veloc_to_ticks( // Convert Velocity (in RPM) to ticks (in Reference Frequency Cycles)
	int veloc_val // input velocity
) // Returns time in ticks
{
	double scale = ((double)60 * ((double)PLATFORM_REFERENCE_HZ / (double)QEI_PER_REV)); // Scaling factor (Hz/QEI/Sec)

	return (unsigned)(scale/abs(veloc_val));
}	// veloc_to_ticks
/*****************************************************************************/
static void init_qei_data( // Initialise QEI Test data
	TEST_QEI_TYP &tst_data_s // Reference to structure of QEI test data
)
{
	QEI_PHASE_TYP clkwise = {{ 0 ,2 ,3 ,1 }};	// Array of Clockwise QEI Phase values [BA}
	QEI_PHASE_TYP anticlk = {{ 0 ,1 ,3 ,2 }};	// Array of Anti-Clockwise QEI Phase values [BA]

	tst_data_s.clk_wise = clkwise; // Assign clockwise rotation cycle
	tst_data_s.anti_clk = anticlk; // Assign anti-clockwise rotation cycle

	tst_data_s.hi_ticks = veloc_to_ticks( HIGH_SPEED ); // Convert HIGH_SPEED to ticks
	tst_data_s.lo_ticks = veloc_to_ticks( LOW_SPEED ); // Convert LOW_SPEED to ticks

} // init_qei_data
/*****************************************************************************/
static void do_qei_test( // Performs one QEI test
	TEST_QEI_TYP &tst_data_s, // Reference to structure of QEI test data
	int motor_id, // Motors Identifier
	port out p4_tst  // current port on which to transmit test data
)
{
	int qei_val;  // QEI value [E I B A]


	tst_data_s.cnt = (tst_data_s.cnt + 1) & QEI_REV_MASK; // Increment (and wrap) QEI position
	tst_data_s.off = (tst_data_s.cnt & QEI_PHASE_MASK); // get phase position for current QEI position

	// Build QEI value ...

	qei_val = tst_data_s.phase.vals[tst_data_s.off]; // Initialise QEI Value with phase bits
	if (!tst_data_s.cnt) qei_val |= QEI_ORIG_MASK; // If zero position, OR with Flag for Origin
	qei_val |= tst_data_s.nerr; // OR with error flag (Bit_3)

	p4_tst <: qei_val;

	acquire_lock(); // Acquire Display Mutex
	printint( motor_id ); printstr( ":QEI:" ); printintln( qei_val);
	release_lock(); // Release Display Mutex
} // do_qei_test
/*****************************************************************************/
static void steady_speed( // Holds steady speed for a number of tests
	TEST_QEI_TYP &tst_data_s, // Reference to structure of QEI test data
	int motor_id, // Motors Identifier
	port out p4_tst,  // current port on which to transmit test data
	int test_cnt, // count-down test counter
	const char str_val[] // String containing title for tests
)
{
	timer chronometer; // XMOS timer


	// Initial start-up, spin at full speed
	acquire_lock(); // Acquire Display Mutex
	printstrln( str_val ); // Print test title
	release_lock(); // Release Display Mutex

	// Loop until all tests done
	while(test_cnt)
	{
		// Wait till test period elapsed
		chronometer when timerafter(tst_data_s.time + tst_data_s.period) :> tst_data_s.time;

		do_qei_test( tst_data_s ,motor_id ,p4_tst ); // Performs one QEI test

		test_cnt--; // Decrement test counter
	} // while(test_cnt)

} // steady_speed
/*****************************************************************************/
static void change_speed( // Changes speed over a number of tests
	TEST_QEI_TYP &tst_data_s, // Reference to structure of QEI test data
	int motor_id, // Motors Identifier
	port out p4_tst,  // current port on which to transmit test data
	int test_cnt, // count-down test counter
	const char str_val[], // String containing title for tests
	int scale // (scale/1024) is scaling factor for time period
)
{
	timer chronometer; // XMOS timer


	// Decelerate
	acquire_lock(); // Acquire Display Mutex
	printstrln( str_val ); // Print test title
	release_lock(); // Release Display Mutex

	while(test_cnt)
	{
		// Wait till test period elapsed
		chronometer when timerafter(tst_data_s.time + tst_data_s.period) :> tst_data_s.time;

		do_qei_test( tst_data_s ,motor_id ,p4_tst ); // Performs one QEI test

		// Alter period to change speed
		tst_data_s.period = (scale * tst_data_s.period + HALF_SCALE) >> SCALE_PRECISION;

		test_cnt--; // Decrement test counter
	} // while(test_cnt)

} // change_speed
/*****************************************************************************/
static void gen_motor_qei_test_data( // Generate QEI Test data for one motor
	TEST_QEI_TYP &tst_data_s, // Reference to structure of QEI test data
	int motor_id, // Motors Identifier
	port out p4_tst  // current port on which to transmit test data
)
{
	timer chronometer; // XMOS timer


	chronometer :> tst_data_s.time; // Get start time

	// NB These tests assume QEI_FILTER = 0

	tst_data_s.nerr = QEI_NERR_MASK; // Initialise error flag to NO errors (Bit_3 = 1)
	tst_data_s.phase = tst_data_s.clk_wise; // Test Clock-Wise direction
	tst_data_s.period = tst_data_s.lo_ticks; // Set period for Low start speed
	change_speed( tst_data_s ,motor_id ,p4_tst ,ACC_TESTS ," Accelerate Clockwise " ,ACC_SCALE );

	tst_data_s.period = tst_data_s.hi_ticks; // Set period for High speed
	steady_speed( tst_data_s ,motor_id ,p4_tst ,MAX_TESTS ," Max_Speed Clockwise ");
	change_speed( tst_data_s ,motor_id ,p4_tst ,DEC_TESTS ," Decelerate Clockwise " ,DEC_SCALE );

	tst_data_s.phase = tst_data_s.anti_clk; // Test Anti-Clockwise direction
	change_speed( tst_data_s ,motor_id ,p4_tst ,ACC_TESTS ," Accelerate Anti-Clockwise " ,ACC_SCALE );

	tst_data_s.period = tst_data_s.hi_ticks; // Set period for High speed
	steady_speed( tst_data_s ,motor_id ,p4_tst ,MAX_TESTS ," Max_Speed Anti-Clockwise ");
	change_speed( tst_data_s ,motor_id ,p4_tst ,DEC_TESTS ," Decelerate Anti-Clockwise " ,DEC_SCALE );

	tst_data_s.period = tst_data_s.lo_ticks; // Low finish speed
	steady_speed( tst_data_s ,motor_id ,p4_tst ,1 ," Min_Speed Anti-Clockwise ");

	tst_data_s.nerr = 0; // Clear (Bit_3) to Signal error condition
	steady_speed( tst_data_s ,motor_id ,p4_tst ,(MIN_TESTS - 1) ," Generate Error Flags ");

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
		acquire_lock(); // Acquire Display Mutex
		printstr( " Start Test Gen. For Motor_"); printintln( motor_cnt );
		release_lock(); // Release Display Mutex

		tst_data_s.off = 0; // Initialise  QEI phase offset
		tst_data_s.cnt = (QEI_REV_MASK - 30); // Set QEI position counter to arbitary number (< QEI_REV_MASK)

		gen_motor_qei_test_data( tst_data_s ,motor_cnt ,p4_tst[motor_cnt] );
	} // for motor_cnt

	acquire_lock(); // Acquire Display Mutex
	printstrln( "Test Generation Ends " );
	release_lock(); // Release Display Mutex
} // gen_all_qei_test_data
/*****************************************************************************/
