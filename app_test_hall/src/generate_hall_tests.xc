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
static unsigned speed_to_ticks( // Convert Velocity (in RPM) to ticks per Hall position (in Reference Frequency Cycles)
	int speed // input speed
) // Returns time in ticks
{
	unsigned ticks; // Output ticks per Hall position (in Reference Frequency Cycles)


	ticks = (TICKS_PER_MIN_PER_HALL + (speed >> 1)) / speed;

	return ticks;
}	// speed_to_ticks
/*****************************************************************************/
static void init_test_data( // Initialise Hall Test data
	GENERATE_HALL_TYP &tst_data_s // Reference to structure of Hall test data
)
{
	init_common_data( tst_data_s.common ); // Initialise data common to Generator and Checker

	// Convert Speed values to Ticks/HALL_position values
	tst_data_s.hi_ticks = speed_to_ticks( HIGH_SPEED ); // Convert HIGH_SPEED to ticks
	tst_data_s.lo_ticks = speed_to_ticks( LOW_SPEED ); // Convert LOW_SPEED to ticks

	tst_data_s.print = PRINT_TST_HALL; // Set print mode
	tst_data_s.dbg = 0; // Set debug mode

} // init_test_data
/*****************************************************************************/
static void init_motor_tests( // Initialisation for each set of motor tests
	GENERATE_HALL_TYP &tst_data_s, // Reference to structure of Hall test data
	int motor_id // Unique Motor Identifier
)
{
	tst_data_s.id = motor_id; // Assign motor identifier
	tst_data_s.period = tst_data_s.lo_ticks; // Set period for Low start speed
	tst_data_s.off = 0; // Set Hall phase offset to arbitary number (< HALL_PER_POLE)
	tst_data_s.vector.comp_state[CNTRL] = SKIP; // Initialise to skipped test for start-up mode
	tst_data_s.prev_hall = HALL_NERR_MASK; // Initialise to starting value
} // init_motor_tests
/*****************************************************************************/
static void assign_test_vector_error( // Assign Error-state of test vector
	GENERATE_HALL_TYP &tst_data_s, // Reference to structure of Hall test data
	ERROR_HALL_ENUM inp_err // Input Error-state
)
{
	switch( inp_err )
	{
		case HALL_ERR_OFF: // No Errors
			tst_data_s.nerr = HALL_NERR_MASK; // Set error flag to NO errors (Bit_3 = 1)
		break; // case HALL_ERR_OFF:

		case HALL_ERR_ON: // Force Error
			tst_data_s.nerr = 0; // Clear (Bit_3) to Signal error condition
		break; // case HALL_ERR_OFF:

		default:
			acquire_lock(); // Acquire Display Mutex
			printstrln("ERROR: Unknown Hall Error-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( inp_err )

	tst_data_s.vector.comp_state[ERROR] = inp_err; // Update Error-state of test vector
} // assign_test_vector_error
/*****************************************************************************/
static void assign_test_vector_spin( // Assign Spin-state of test vector
	GENERATE_HALL_TYP &tst_data_s, // Reference to structure of Hall test data
	SPIN_HALL_ENUM inp_spin // Input Spin-state
)
{
	switch( inp_spin )
	{
		case CLOCK: // Clock-wise
		break; // case CLOCK:

		case ANTI: // Anti-clockwise
		break; // case ANTI:

		default:
			acquire_lock(); // Acquire Display Mutex
			printstrln("ERROR: Unknown Hall Spin-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( inp_spin )

	tst_data_s.vector.comp_state[SPIN] = inp_spin; // Update Spin-state of test vector
} // assign_test_vector_spin
/*****************************************************************************/
static void assign_test_vector_speed( // Assign Speed-state of test vector
	GENERATE_HALL_TYP &tst_data_s, // Reference to structure of Hall test data
	SPEED_HALL_ENUM inp_speed // Input speed-state
)
{
	switch( inp_speed )
	{
		case ACCEL: // Accelerate
			tst_data_s.scale = ACC_SCALE; // Set speed scaling factor for acceleration
		break; // case ACCEL:

		case FAST: // Constant Fast Speed
			tst_data_s.period = tst_data_s.hi_ticks; // Set period for High speed
		break; // case FAST:

		case DECEL: // Accelerate
			tst_data_s.scale = DEC_SCALE; // Set speed scaling factor for deceleration
		break; // case DECEL:

		case SLOW: // Constant Slow Speed
			tst_data_s.period = tst_data_s.lo_ticks; // Low finish speed
		break; // case SLOW:

		default:
			acquire_lock(); // Acquire Display Mutex
			printstrln("ERROR: Unknown Hall Velocity-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( inp_speed )

	tst_data_s.vector.comp_state[SPEED] = inp_speed; // Update speed-state of test vector
} // assign_test_vector_speed
/*****************************************************************************/
static void do_hall_test( // Performs one Hall test
	GENERATE_HALL_TYP &tst_data_s, // Reference to structure of Hall test data
	port out p4_tst  // current port on which to transmit test data
)
{
	int hall_val;  // Hall value [E I B A]
	unsigned time_val; // Temporary time value


	// Increment/Decrement (and wrap) Hall position ...

	switch( tst_data_s.vector.comp_state[SPIN] )
	{
		case CLOCK: // Clock-wise
			if ((HALL_PER_POLE - 1) > tst_data_s.off)
			{
				tst_data_s.off++;	// Increment Hall position
			} // if ((HALL_PER_POLE - 1) > tst_data_s.off)
			else
			{
				tst_data_s.off = 0;	// Wrap Hall position
			} // else !((HALL_PER_POLE - 1) > tst_data_s.off)
		break; // case CLOCK:

		case ANTI: // Anti-clockwise
			if (0 < tst_data_s.off)
			{
				tst_data_s.off--;	// Decrement Hall position
			} // if (0 < tst_data_s.off)
			else
			{
				tst_data_s.off = (HALL_PER_POLE - 1);	// Wrap Hall position
			} // else !(0 < tst_data_s.off)
		break; // case ANTI:

		default:
			acquire_lock(); // Acquire Display Mutex
			printstrln("ERROR: Unknown Hall Spin-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( tst_data_s.vector.comp_state[SPIN] )

	// Build Hall value ...

	hall_val = tst_data_s.common.phases.vals[tst_data_s.off]; // Initialise Hall Value with phase bits
	hall_val |= tst_data_s.nerr; // OR with error flag (Bit_3)

	// Wait till test period elapsed
	time_val = tst_data_s.period;

	while (time_val > NUM_PORT_TIMES)
	{	// Wait for NUM_PORT_TIMES ticks
		p4_tst @ tst_data_s.time <: tst_data_s.prev_hall; // Resend old value
		time_val -= NUM_PORT_TIMES;
	} // while (time_val > (1 << 16))

	tst_data_s.time = (PORT_TIME_TYP)((unsigned)tst_data_s.time + time_val);
	p4_tst @ tst_data_s.time <: hall_val; // Now send new value
	tst_data_s.prev_hall = hall_val; // Store Hall value

	if (tst_data_s.print)
	{
		acquire_lock(); // Acquire Display Mutex
		printint( tst_data_s.id ); printstr( ":HALL:" ); printintln(hall_val); 
		release_lock(); // Release Display Mutex
	} // if (tst_data_s.print)

} // do_hall_test
/*****************************************************************************/
static void do_hall_vector( // Do all tests for one Hall test vector
	GENERATE_HALL_TYP &tst_data_s, // Reference to structure of Hall test data
	streaming chanend c_tst, // Channel for sending test vecotrs to test checker
	port out p4_tst,  // current port on which to transmit test data
	int test_cnt // count-down test counter
)
{
	c_tst <: tst_data_s.vector; // transmit test vector details to test checker

	// Decelerate
	if (tst_data_s.print)
	{
		print_test_vector( tst_data_s.common ,tst_data_s.vector ,"" );
	} // if (tst_data_s.print)

	// Loop through tests for current test vector
	while(test_cnt)
	{
		do_hall_test( tst_data_s ,p4_tst ); // Performs one Hall test

		// Check which speed mode being tested
		switch( tst_data_s.vector.comp_state[SPEED] )
		{
			case ACCEL: // Accelerate
				tst_data_s.period = (tst_data_s.scale * tst_data_s.period + HALF_SCALE) >> SCALE_PRECISION; // Alter period to change speed
			break; // case ACCEL:
	
			case FAST: // Constant Fast Speed
			break; // case FAST:
	
			case DECEL: // Decelerate
				tst_data_s.period = (tst_data_s.scale * tst_data_s.period + HALF_SCALE) >> SCALE_PRECISION; // Alter period to change speed
			break; // case ACCEL:
	
			case SLOW: // Constant Slow Speed
			break; // case FAST:
	
			default:
				acquire_lock(); // Acquire Display Mutex
				printstrln("ERROR: Unknown Hall Velocity-state");
				release_lock(); // Release Display Mutex
				assert(0 == 1);
			break; // default:
		} // switch( tst_data_s.vector.comp_state[SPEED] )

		test_cnt--; // Decrement test counter
	} // while(test_cnt)

} // do_hall_vector
/*****************************************************************************/
static void gen_motor_hall_test_data( // Generate Hall Test data for one motor
	GENERATE_HALL_TYP &tst_data_s, // Reference to structure of Hall test data
	streaming chanend c_tst, // Channel for sending test vecotrs to test checker
	port out p4_tst  // current port on which to transmit test data
)
{
	if (tst_data_s.print)
	{
		acquire_lock(); // Acquire Display Mutex
		printstr( " Start Test Gen. For Motor_"); printintln( tst_data_s.id );
		release_lock(); // Release Display Mutex
	} // if (tst_data_s.print)

	p4_tst <: 0 @ tst_data_s.time; // Get start time

	// NB These tests assume HALL_FILTER = 0

	assign_test_vector_error( tst_data_s ,HALL_ERR_OFF ); // Set test vector to NO errors
	assign_test_vector_spin( tst_data_s ,CLOCK ); // Set test vector to Clock-wise spin
	assign_test_vector_speed( tst_data_s ,FAST ); // Set test vector to constant Fast speed

	tst_data_s.vector.comp_state[CNTRL] = VALID; // Settling complete, Switch on testing
	do_hall_vector( tst_data_s ,c_tst ,p4_tst ,(MAX_TESTS >> 1) );

	assign_test_vector_error( tst_data_s ,HALL_ERR_ON ); // Switch on error-bit
	tst_data_s.vector.comp_state[CNTRL] = SKIP; // Switch off testing, while server counts required consecutive errors
	do_hall_vector( tst_data_s ,c_tst ,p4_tst ,MAX_HALL_STATUS_ERR );

	tst_data_s.vector.comp_state[CNTRL] = VALID; // Switch on testing, now error-status should be set
	do_hall_vector( tst_data_s ,c_tst ,p4_tst ,MAX_HALL_STATUS_ERR );

	assign_test_vector_error( tst_data_s ,HALL_ERR_OFF ); // Switch off error-bit
	tst_data_s.vector.comp_state[CNTRL] = SKIP; // Switch off testing, while server counts required consecutive non-errors
	do_hall_vector( tst_data_s ,c_tst ,p4_tst ,MAX_HALL_STATUS_ERR );

	tst_data_s.vector.comp_state[CNTRL] = VALID; // Switch on testing, now error-status should be cleared
	do_hall_vector( tst_data_s ,c_tst ,p4_tst ,MAX_HALL_STATUS_ERR );

	assign_test_vector_spin( tst_data_s ,ANTI ); // Set test vector to Anti-clockwise spin
	tst_data_s.vector.comp_state[CNTRL] = VALID; // Settling complete, Switch on testing
	do_hall_vector( tst_data_s ,c_tst ,p4_tst ,(MAX_TESTS >> 1) );

	tst_data_s.vector.comp_state[CNTRL] = QUIT; // Signal that testing has ended for current motor
	do_hall_vector( tst_data_s ,c_tst ,p4_tst ,1 );

} // gen_motor_hall_test_data
/*****************************************************************************/
void gen_all_hall_test_data( // Generate Hall Test data
	streaming chanend c_tst, // Channel for sending test vecotrs to test checker
	port out p4_tst[]  // Array of ports on which to transmit test data
)
{
	GENERATE_HALL_TYP tst_data_s; // Structure of Hall test data
	int motor_cnt; // counts Motors 


	init_test_data( tst_data_s );

	// Loop through motors, so we can print results serially
	for (motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++)
	{
		init_motor_tests( tst_data_s ,motor_cnt ); // Initialise set of motor tests

		gen_motor_hall_test_data( tst_data_s ,c_tst ,p4_tst[motor_cnt] );
	} // for motor_cnt

	if (tst_data_s.print)
	{
		acquire_lock(); // Acquire Display Mutex
		printstrln( "Test Generation Ends " );
		release_lock(); // Release Display Mutex
	} // if (tst_data_s.print)
} // gen_all_hall_test_data
/*****************************************************************************/
