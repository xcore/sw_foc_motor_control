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
static void init_test_data( // Initialise QEI Test data
	TEST_QEI_TYP &tst_data_s // Reference to structure of QEI test data
)
{
	QEI_PHASE_TYP clkwise = {{ 0 ,2 ,3 ,1 }};	// Array of Clockwise QEI Phase values [BA}
	QEI_PHASE_TYP anticlk = {{ 0 ,1 ,3 ,2 }};	// Array of Anti-Clockwise QEI Phase values [BA]

	tst_data_s.clk_wise = clkwise; // Assign clockwise rotation cycle
	tst_data_s.anti_clk = anticlk; // Assign anti-clockwise rotation cycle

	tst_data_s.hi_ticks = veloc_to_ticks( HIGH_SPEED ); // Convert HIGH_SPEED to ticks
	tst_data_s.lo_ticks = veloc_to_ticks( LOW_SPEED ); // Convert LOW_SPEED to ticks

} // init_test_data
/*****************************************************************************/
static void init_motor_tests( // Initialisation for each set of motor tests
	TEST_QEI_TYP &tst_data_s, // Reference to structure of QEI test data
	int motor_id // Unique Motor Identifier
)
{
	tst_data_s.id = motor_id; // Assign motor identifier
	tst_data_s.period = tst_data_s.lo_ticks; // Set period for Low start speed
	tst_data_s.off = 0; // Initialise  QEI phase offset
	tst_data_s.cnt = (QEI_REV_MASK - 30); // Set QEI position counter to arbitary number (< QEI_REV_MASK)
	tst_data_s.vector.valid = 0; // Initialise to invalid test for start-up mode
} // init_motor_tests
/*****************************************************************************/
static void assign_test_vector_error( // Assign Error-state of test vector
	TEST_QEI_TYP &tst_data_s, // Reference to structure of QEI test data
	ERROR_QEI_ENUM inp_err // Input Error-state
)
{
	switch( inp_err )
	{
		case ERR_OFF: // No Errors
			tst_data_s.nerr = QEI_NERR_MASK; // Set error flag to NO errors (Bit_3 = 1)
			safestrcpy( tst_data_s.names[ERROR] ," No_Err" );
		break; // case ERR_OFF:

		case ERR_ON: // Force Error
			tst_data_s.nerr = 0; // Clear (Bit_3) to Signal error condition
			safestrcpy( tst_data_s.names[ERROR] ," Err_On" );
		break; // case ERR_OFF:

		default:
			acquire_lock(); // Acquire Display Mutex
			printstrln("ERROR: Unknown QEI Error-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( inp_err )

	tst_data_s.vector.err = inp_err; // Update Error-state of test vector
} // assign_test_vector_error
/*****************************************************************************/
static void assign_test_vector_origin( // Assign Origin-state of test vector
	TEST_QEI_TYP &tst_data_s, // Reference to structure of QEI test data
	ORIG_QEI_ENUM inp_orig // Input Origin-state
)
{
	switch( inp_orig )
	{
		case ORIG_OFF: // No Origin
			tst_data_s.orig = 0; // Clear origin flag (Bit_2 = 0)
			safestrcpy( tst_data_s.names[ORIGIN] ," No_Orig" );
		break; // case ORIG_OFF:

		case ORIG_ON: // Origin
			tst_data_s.orig = QEI_ORIG_MASK; // Set origin flag (Bit_2 = 1)
			safestrcpy( tst_data_s.names[ORIGIN] ," Orig_On" );
		break; // case ORIG_OFF:

		default:
			acquire_lock(); // Acquire Display Mutex
			printstrln("ERROR: Unknown QEI Origin-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( inp_orig )

	tst_data_s.vector.orig = inp_orig; // Update Error-state of test vector
} // assign_test_vector_origin
/*****************************************************************************/
static void assign_test_vector_spin( // Assign Spin-state of test vector
	TEST_QEI_TYP &tst_data_s, // Reference to structure of QEI test data
	SPIN_QEI_ENUM inp_spin // Input Spin-state
)
{
	switch( inp_spin )
	{
		case CLOCK: // Clock-wise
			tst_data_s.phase = tst_data_s.clk_wise; // Set Clock-Wise direction
			safestrcpy( tst_data_s.names[SPIN] ," Clock-Wise" );
		break; // case CLOCK:

		case ANTI: // Anti-clockwise
			tst_data_s.phase = tst_data_s.anti_clk; // Set Anti-Clockwise direction
			safestrcpy( tst_data_s.names[SPIN] ," Anti-Clock" );
		break; // case ANTI:

		default:
			acquire_lock(); // Acquire Display Mutex
			printstrln("ERROR: Unknown QEI Spin-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( inp_spin )

	tst_data_s.vector.spin = inp_spin; // Update Spin-state of test vector
} // assign_test_vector_spin
/*****************************************************************************/
static void assign_test_vector_speed( // Assign Speed-state of test vector
	TEST_QEI_TYP &tst_data_s, // Reference to structure of QEI test data
	SPEED_QEI_ENUM inp_speed // Input speed-state
)
{
	switch( inp_speed )
	{
		case ACCEL: // Accelerate
			tst_data_s.scale = ACC_SCALE; // Set speed scaling factor for acceleration
			safestrcpy( tst_data_s.names[SPEED] ," Accelerating" );
		break; // case ACCEL:

		case FAST: // Constant Fast Speed
			tst_data_s.period = tst_data_s.hi_ticks; // Set period for High speed
			safestrcpy( tst_data_s.names[SPEED] ," Fast-steady " );
		break; // case FAST:

		case DECEL: // Accelerate
			tst_data_s.scale = DEC_SCALE; // Set speed scaling factor for deceleration
			safestrcpy( tst_data_s.names[SPEED] ," Decelerating" );
		break; // case DECEL:

		case SLOW: // Constant Slow Speed
			tst_data_s.period = tst_data_s.lo_ticks; // Low finish speed
			safestrcpy( tst_data_s.names[SPEED] ," Slow-steady " );
		break; // case SLOW:

		default:
			acquire_lock(); // Acquire Display Mutex
			printstrln("ERROR: Unknown QEI Velocity-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( inp_speed )

	tst_data_s.vector.speed = inp_speed; // Update speed-state of test vector
} // assign_test_vector_speed
/*****************************************************************************/
static void do_qei_test( // Performs one QEI test
	TEST_QEI_TYP &tst_data_s, // Reference to structure of QEI test data
	port out p4_tst  // current port on which to transmit test data
)
{
	timer chronometer; // XMOS timer
	int qei_val;  // QEI value [E I B A]


	tst_data_s.cnt = (tst_data_s.cnt + 1) & QEI_REV_MASK; // Increment (and wrap) QEI position
	tst_data_s.off = (tst_data_s.cnt & QEI_PHASE_MASK); // get phase position for current QEI position

	// Build QEI value ...

	qei_val = tst_data_s.phase.vals[tst_data_s.off]; // Initialise QEI Value with phase bits
	qei_val |= tst_data_s.orig; // OR with origin flag (Bit_2)
	qei_val |= tst_data_s.nerr; // OR with error flag (Bit_3)

	// Wait till test period elapsed
	chronometer when timerafter(tst_data_s.time + tst_data_s.period) :> tst_data_s.time;
	p4_tst <: qei_val;

	acquire_lock(); // Acquire Display Mutex
	printint( tst_data_s.id ); printstr( ":QEI:" ); printintln( qei_val);
	release_lock(); // Release Display Mutex

} // do_qei_test
/*****************************************************************************/
static void do_qei_vector( // Do all tests for one QEI test vector
	TEST_QEI_TYP &tst_data_s, // Reference to structure of QEI test data
	streaming chanend c_tst, // Channel for sending test vecotrs to test checker
	port out p4_tst,  // current port on which to transmit test data
	int test_cnt // count-down test counter
)
{
	int comp_cnt; // Counter for Test Vector components


	c_tst <: tst_data_s.vector; // transmit test vector details to test checker

	// Decelerate
	acquire_lock(); // Acquire Display Mutex
	for (comp_cnt=0; comp_cnt<NUM_VECT_COMPS; comp_cnt++)
	{
		printstr( tst_data_s.names[comp_cnt] ); // Print component status
	} // for comp_cnt
	printcharln( ':' );
	release_lock(); // Release Display Mutex

	// Loop through tests for current test vector
	while(test_cnt)
	{
		do_qei_test( tst_data_s ,p4_tst ); // Performs one QEI test

		// Check which speed mode being tested
		switch( tst_data_s.vector.speed )
		{
			case ACCEL: // Accelerate
				tst_data_s.period = (tst_data_s.scale * tst_data_s.period + HALF_SCALE) >> SCALE_PRECISION; // Alter period to change speed
			break; // case ACCEL:
	
			case FAST: // Constant Fast Speed
			break; // case FAST:
	
			case DECEL: // Accelerate
				tst_data_s.period = (tst_data_s.scale * tst_data_s.period + HALF_SCALE) >> SCALE_PRECISION; // Alter period to change speed
			break; // case ACCEL:
	
			case SLOW: // Constant Slow Speed
			break; // case FAST:
	
			default:
				acquire_lock(); // Acquire Display Mutex
				printstrln("ERROR: Unknown QEI Velocity-state");
				release_lock(); // Release Display Mutex
				assert(0 == 1);
			break; // default:
		} // switch( tst_data_s.vector.speed )

		test_cnt--; // Decrement test counter
	} // while(test_cnt)

} // do_qei_vector
/*****************************************************************************/
static void gen_motor_qei_test_data( // Generate QEI Test data for one motor
	TEST_QEI_TYP &tst_data_s, // Reference to structure of QEI test data
	streaming chanend c_tst, // Channel for sending test vecotrs to test checker
	port out p4_tst  // current port on which to transmit test data
)
{
	timer chronometer; // XMOS timer


	acquire_lock(); // Acquire Display Mutex
	printstr( " Start Test Gen. For Motor_"); printintln( tst_data_s.id );
	printstr("Size="); printintln( sizeof(TEST_VECT_TYP) );
	release_lock(); // Release Display Mutex

	chronometer :> tst_data_s.time; // Get start time

	// NB These tests assume QEI_FILTER = 0

	assign_test_vector_error( tst_data_s ,ERR_OFF ); // Set test vector to NO errors
	assign_test_vector_origin( tst_data_s ,ORIG_OFF ); // Set test vector to No Origin
	assign_test_vector_spin( tst_data_s ,CLOCK ); // Set test vector to Clock-wise spin
	assign_test_vector_speed( tst_data_s ,ACCEL ); // Set test vector to Accelerate
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,2 ); // Start-up phase

	tst_data_s.vector.valid = 1; // Switch on testing
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,(ACC_TESTS - 2) );

	assign_test_vector_speed( tst_data_s ,FAST ); // Set test vector to constant Fast speed
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,4 );

	assign_test_vector_origin( tst_data_s ,ORIG_ON ); // Set test vector to Origin
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,1 );

	assign_test_vector_origin( tst_data_s ,ORIG_OFF ); // Set test vector to No Origin
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,(MAX_TESTS - 5) );

	assign_test_vector_speed( tst_data_s ,DECEL ); // Set test vector to Decelerate
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,DEC_TESTS );

	assign_test_vector_spin( tst_data_s ,ANTI ); // Set test vector to Anti-clockwise spin
	assign_test_vector_speed( tst_data_s ,ACCEL ); // Set test vector to Accelerate
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,ACC_TESTS );

	assign_test_vector_speed( tst_data_s ,FAST ); // Set test vector to constant Fast speed
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,MAX_TESTS );

	assign_test_vector_speed( tst_data_s ,DECEL ); // Set test vector to Decelerate
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,DEC_TESTS );

	assign_test_vector_speed( tst_data_s ,SLOW ); // Set test vector to constant Slow speed
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,1 );

	assign_test_vector_error( tst_data_s ,ERR_ON );
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,(MIN_TESTS - 1) );

} // gen_motor_qei_test_data
/*****************************************************************************/
void gen_all_qei_test_data( // Generate QEI Test data
	streaming chanend c_tst, // Channel for sending test vecotrs to test checker
	port out p4_tst[]  // Array of ports on which to transmit test data
)
{
	TEST_QEI_TYP tst_data_s; // Structure of QEI test data
	int motor_cnt; // counts Motors 


	init_test_data( tst_data_s );

	// Loop through motors, so we can print results serially
	for (motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++)
	{
		init_motor_tests( tst_data_s ,motor_cnt ); // Initialise set of motor tests

		gen_motor_qei_test_data( tst_data_s ,c_tst ,p4_tst[motor_cnt] );
	} // for motor_cnt

	acquire_lock(); // Acquire Display Mutex
	printstrln( "Test Generation Ends " );
	release_lock(); // Release Display Mutex
} // gen_all_qei_test_data
/*****************************************************************************/
