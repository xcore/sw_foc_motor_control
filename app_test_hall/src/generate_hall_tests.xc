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

#include "generate_hall_tests.h"

/*	[CBA] order is 001 -> 011 -> 010 -> 110 -> 100 -> 101  Clockwise direction
 *	[CBA] order is 001 -> 101 -> 100 -> 110 -> 010 -> 011  Anti-Clockwise direction
 */
/*****************************************************************************/
static unsigned veloc_to_ticks( // Convert Velocity (in RPM) to ticks (in Reference Frequency Cycles)
	int veloc_val // input velocity
) // Returns time in ticks
{
	double scale = ((double)60 * ((double)PLATFORM_REFERENCE_HZ / (double)HALL_PER_REV)); // Scaling factor (Hz/HALL/Sec)

	return (int)(0.5 + scale/(double)abs(veloc_val) );
}	// veloc_to_ticks
/*****************************************************************************/
static void init_hall_data( // Initialise HALL Test data
	TEST_HALL_TYP &tst_data_s // Reference to structure of HALL test data
)
{
	HALL_PHASE_TYP clkwise = {{ 1 ,3 ,2 ,6 ,4 ,5 }};	// Array of Clockwise HALL Phase values [A}
	HALL_PHASE_TYP anticlk = {{ 1 ,5 ,4 ,6 ,2 ,3 }};	// Array of Anti-Clockwise HALL Phase values [BA]

	tst_data_s.clk_wise = clkwise; // Assign clockwise rotation cycle
	tst_data_s.anti_clk = anticlk; // Assign anti-clockwise rotation cycle

	tst_data_s.hi_ticks = veloc_to_ticks( HIGH_SPEED ); // Convert HIGH_SPEED to ticks
	tst_data_s.lo_ticks = veloc_to_ticks( LOW_SPEED ); // Convert LOW_SPEED to ticks

} // init_hall_data
/*****************************************************************************/
static int circular_increment( // Increments input value and wraps if necessary
	int inp_val, // Input value to increment
	int size		// No. of possible incremental values
) // Returns incremented value
{
	int out_val = inp_val + 1; // Preset Output value to incremented input value


	// Check output needs wrapping
	if (out_val >= size)
	{
		out_val = 0; // wrap
	} // if (out_val >= size)

	return out_val;
} // circular_increment
/*****************************************************************************/
static void do_hall_test( // Performs one HALL test
	TEST_HALL_TYP &tst_data_s, // Reference to structure of HALL test data
	int motor_id, // Motors Identifier
	port out p4_tst  // current port on which to transmit test data
)
{
	int hall_val;  // HALL value [E C B A]

	tst_data_s.cnt = circular_increment( tst_data_s.cnt ,HALL_PER_REV );
	tst_data_s.off = circular_increment( tst_data_s.off ,NUM_HALL_PHASES );

	// Build HALL value ...

	hall_val = tst_data_s.phase.vals[tst_data_s.off]; // Initialise HALL Value with phase bits
	hall_val |= tst_data_s.nerr; // OR with error flag (Bit_3)

	p4_tst <: hall_val;

	acquire_lock(); // Acquire Display Mutex
	printint( motor_id ); printstr( ":HALL:" ); printintln( hall_val);
	release_lock(); // Release Display Mutex
} // do_hall_test
/*****************************************************************************/
static void steady_speed( // Holds steady speed for a number of tests
	TEST_HALL_TYP &tst_data_s, // Reference to structure of HALL test data
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

		do_hall_test( tst_data_s ,motor_id ,p4_tst ); // Performs one HALL test

		test_cnt--; // Decrement test counter
	} // while(test_cnt)

} // steady_speed
/*****************************************************************************/
static void change_speed( // Changes speed over a number of tests
	TEST_HALL_TYP &tst_data_s, // Reference to structure of HALL test data
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
acquire_lock();	printstrln("CS_1"); release_lock();
		chronometer when timerafter(tst_data_s.time + tst_data_s.period) :> tst_data_s.time;
acquire_lock();	printstrln("CS_2"); release_lock();

		do_hall_test( tst_data_s ,motor_id ,p4_tst ); // Performs one HALL test

		// Alter period to change speed
		tst_data_s.period = (scale * tst_data_s.period + HALF_SCALE) >> SCALE_PRECISION;

		test_cnt--; // Decrement test counter
acquire_lock();	printstrln("CS_3"); release_lock();
	} // while(test_cnt)

} // change_speed
/*****************************************************************************/
static void gen_motor_hall_test_data( // Generate HALL Test data for one motor
	TEST_HALL_TYP &tst_data_s, // Reference to structure of HALL test data
	int motor_id, // Motors Identifier
	port out p4_tst  // current port on which to transmit test data
)
{
	timer chronometer; // XMOS timer


	chronometer :> tst_data_s.time; // Get start time

	// NB These tests assume HALL_FILTER = 0

	tst_data_s.nerr = HALL_NERR_MASK; // Initialise error flag to NO errors (Bit_3 = 1)
	tst_data_s.phase = tst_data_s.clk_wise; // Test Clock-Wise direction

	tst_data_s.period = tst_data_s.hi_ticks; // Set period for High speed
	steady_speed( tst_data_s ,motor_id ,p4_tst ,MAX_TESTS ," Max_Speed Clockwise ");

	tst_data_s.phase = tst_data_s.anti_clk; // Test Anti-Clockwise direction
	steady_speed( tst_data_s ,motor_id ,p4_tst ,MAX_TESTS ," Max_Speed Anti-Clockwise ");

	tst_data_s.nerr = 0; // Clear (Bit_3) to Signal error condition
	steady_speed( tst_data_s ,motor_id ,p4_tst ,(MIN_TESTS - 1) ," Generate Error Flags ");

} // gen_motor_hall_test_data
/*****************************************************************************/
void gen_all_hall_test_data( // Generate HALL Test data
	port out p4_tst[]  // Array of ports on which to transmit test data
)
{
	TEST_HALL_TYP tst_data_s; // Structure of HALL test data
	int motor_cnt; // counts Motors 


	init_hall_data( tst_data_s );

	// Loop through motors, so we can print results serially
	for (motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++)
	{
		acquire_lock(); // Acquire Display Mutex
		printstr( " Start Test Gen. For Motor_"); printintln( motor_cnt );
		release_lock(); // Release Display Mutex

		tst_data_s.off = 0; // Initialise  HALL phase offset
		tst_data_s.cnt = 0; // Initialise  HALL position counter

		gen_motor_hall_test_data( tst_data_s ,motor_cnt ,p4_tst[motor_cnt] );
	} // for motor_cnt

	acquire_lock(); // Acquire Display Mutex
	printstrln( "Test Generation Ends " );
	release_lock(); // Release Display Mutex
} // gen_all_hall_test_data
/*****************************************************************************/
