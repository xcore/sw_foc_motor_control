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

/* This code adopts the Traditional convention that
 * the positive spin direction is the one where Phase_A leads Phase_B.
 * E.g. Phase_A goes high one bit-change earlier than Phase_B goes high.
 * This definition is based on time, and is NOT dependent on spatial orientation of the motor!-)
 *
 * WARNING: Traditionally Phase_A is the Most Significant Bit (MSB).
 * However, the QEI server S/W is designed to work with the XMOS motor board.
 * This board has Phase_A as the Least Significant Bit (LSB).
 * So the generated bit changes are as follows:-
 *
 *	[CBA] order is 001 -> 011 -> 010 -> 110 -> 100 -> 101  Positive-spin
 *	[CBA] order is 001 -> 101 -> 100 -> 110 -> 010 -> 011  Negative-spin
 */
/*****************************************************************************/
static void parse_control_file( // Parse Hall control file and set up test options
	GENERATE_TST_TYP &tst_data_s // Reference to structure of Hall test data
)
{
  unsigned char file_buf[FILE_SIZE];
  int char_cnt; // Character counter
  int line_cnt = 0; // line counter
  int test_cnt = 0; // test counter
  int new_line = 0; // flag if new-line found
  int tst_line = 0; // flag set if test-option found
  char curr_char; // Current character
  int file_id; // File identifier
  int status = 0; // Error status


	// Initialise file buffer
  for (char_cnt = 0; char_cnt < FILE_SIZE; ++char_cnt)
	{
    file_buf[char_cnt] = 0;
  } // for char_cnt

  file_id = _open( "hall_tests.txt" ,O_RDONLY ,0 ); // Open control file for HALL tests

  assert(-1 != file_id); // ERROR: Open file failed (_open)

  _read( file_id ,file_buf ,FILE_SIZE );  // Read file into buffer

  status = _close(file_id);	// Close file
  assert(0 == status);	// ERROR: Close file failed (_close)

	acquire_lock(); // Acquire Display Mutex
	printcharln(' ');
	printstrln("Read following Test Options ..." );

	// Parse the file buffer for test options
  for (char_cnt = 0; char_cnt < FILE_SIZE; ++char_cnt)
	{
    curr_char = file_buf[char_cnt]; // Get next character

    if (!curr_char) break; // Check for end of file info.


		switch (curr_char)
		{
    	case '0' : // Opt out of test
				tst_data_s.common.options.flags[test_cnt] = 0;
				tst_line = 1; // Flag test-option found
    	break; // case '0' :

    	case '1' : // Opt for this test
				tst_data_s.common.options.flags[test_cnt] = 1;
				tst_line = 1; // Flag test-option found
    	break; // case '1' :

    	case '#' : // Start of comment
				new_line = 1; // Set flag for new-line
    	break; // case '1' :

    	case '\n' : // End of line.
				new_line = 1; // Set flag for new-line
    	break; // case '\n' :

    	default : // Whitespace
				// Un-determined line
    	break; // case '\n' :
		} // switch (curr_char)

		// Check if we have a test-option line
		if (tst_line)
		{ // Process test-option line
			test_cnt++;
			printchar(curr_char);
			printchar(' ');

			while ('\n' != file_buf[char_cnt])
			{
				char_cnt++;
				assert(char_cnt < FILE_SIZE); // End-of-file found
				printchar( file_buf[char_cnt] );
			} // while ('\n' != file_buf[char_cnt])

			line_cnt++;
			tst_line = 0; // Clear test-option flag
			new_line = 0; // Clear new_line flag
		} // if (tst_line)
		else
		{ // Process other line
			// Check if we need to move to new-line
			if (new_line)
			{ // skip to next line
				while ('\n' != file_buf[char_cnt])
				{
					char_cnt++;
					assert(char_cnt < FILE_SIZE); // End-of-file found
				} // while ('\n' != file_buf[char_cnt])

				line_cnt++;
				new_line = 0; // Clear new_line flag
			} // if (new_line)
		} // else !(tst_line)
  } // for char_cnt

	printcharln(' ');
	release_lock(); // Release Display Mutex

	// Do some checks ...
	assert(test_cnt == NUM_TEST_OPTS); // Check read required number of test options found
	assert(NUM_TEST_OPTS <= line_cnt); // Check enough file lines read
	assert(test_cnt <= line_cnt); // Check no more than one test/line
	assert( tst_data_s.common.options.flags[TST_MOTOR] < NUMBER_OF_MOTORS ); // Check Motor Identifier in range

	return;
} // parse_control_file
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
	GENERATE_TST_TYP &tst_data_s, // Reference to structure of Hall test data
	streaming chanend c_tst // Channel for sending test vecotrs to test checker
)
{
	init_common_data( tst_data_s.common ); // Initialise data common to Generator and Checker

	// Convert Speed values to Ticks/HALL_position values
	tst_data_s.hi_ticks = speed_to_ticks( HIGH_SPEED ); // Convert HIGH_SPEED to ticks
	tst_data_s.lo_ticks = speed_to_ticks( LOW_SPEED ); // Convert LOW_SPEED to ticks

	tst_data_s.print = VERBOSE_PRINT; // Set print mode
	tst_data_s.dbg = 0; // Set debug mode
	tst_data_s.disp_str[HALL_BITS] = 0; // Add string terminator string

	parse_control_file( tst_data_s );

	c_tst <: tst_data_s.common.options; // Send test options to checker core
} // init_test_data
/*****************************************************************************/
static void init_motor_tests( // Initialisation for each set of motor tests
	GENERATE_TST_TYP &tst_data_s // Reference to structure of Hall test data
)
{
	tst_data_s.period = tst_data_s.lo_ticks; // Set period for Low start speed
	tst_data_s.off = 0; // Set Hall phase offset to arbitary number (< HALL_PER_POLE)
	tst_data_s.curr_vect.comp_state[CNTRL] = SKIP; // Initialise to skipped test for start-up mode
	tst_data_s.prev_vect.comp_state[CNTRL] = QUIT; // Initialise to something that will force an update
	tst_data_s.prev_hall = HALL_NERR_MASK; // Initialise to starting value
} // init_motor_tests
/*****************************************************************************/
static void assign_test_vector_error( // Assign Error-state of test vector
	GENERATE_TST_TYP &tst_data_s, // Reference to structure of Hall test data
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

	tst_data_s.curr_vect.comp_state[ERROR] = inp_err; // Update Error-state of test vector
} // assign_test_vector_error
/*****************************************************************************/
static void assign_test_vector_phase( // Assign phase-state of test vector
	GENERATE_TST_TYP &tst_data_s, // Reference to structure of Hall test data
	PHASE_HALL_ENUM inp_phase // Input Phase-state
)
{
	switch( inp_phase )
	{
		case CHANGE: // Phase-change
		break; // case CHANGE:

		default:
			acquire_lock(); // Acquire Display Mutex
			printstrln("ERROR: Unknown Hall Phase-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( inp_phase )

	tst_data_s.curr_vect.comp_state[PHASE] = inp_phase; // Update Spin-state of test vector
} // assign_test_vector_spin
/*****************************************************************************/
static void assign_test_vector_spin( // Assign Spin-state of test vector
	GENERATE_TST_TYP &tst_data_s, // Reference to structure of Hall test data
	SPIN_HALL_ENUM inp_spin // Input Spin-state
)
{
	switch( inp_spin )
	{
		case POSITIVE: // Positive-spin
		break; // case POSITIVE:

		case NEGATIVE: // Negative-spin
		break; // case NEGATIVE:

		default:
			acquire_lock(); // Acquire Display Mutex
			printstrln("ERROR: Unknown Hall Spin-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( inp_spin )

	tst_data_s.curr_vect.comp_state[SPIN] = inp_spin; // Update Spin-state of test vector
} // assign_test_vector_spin
/*****************************************************************************/
static void assign_test_vector_speed( // Assign Speed-state of test vector
	GENERATE_TST_TYP &tst_data_s, // Reference to structure of Hall test data
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

	tst_data_s.curr_vect.comp_state[SPEED] = inp_speed; // Update speed-state of test vector
} // assign_test_vector_speed
/*****************************************************************************/
static void do_hall_test( // Performs one Hall test
	GENERATE_TST_TYP &tst_data_s, // Reference to structure of Hall test data
	port out p4_tst  // current port on which to transmit test data
)
{
	int hall_val;  // Hall value [E I B A]
	unsigned time_val; // Temporary time value


	// Increment/Decrement (and wrap) Hall position ...

	switch( tst_data_s.curr_vect.comp_state[SPIN] )
	{
		case POSITIVE: // Positive-spin
			if ((HALL_PER_POLE - 1) > tst_data_s.off)
			{
				tst_data_s.off++;	// Increment Hall position
			} // if ((HALL_PER_POLE - 1) > tst_data_s.off)
			else
			{
				tst_data_s.off = 0;	// Wrap Hall position
			} // else !((HALL_PER_POLE - 1) > tst_data_s.off)
		break; // case POSITIVE:

		case NEGATIVE: // Negative-spin
			if (0 < tst_data_s.off)
			{
				tst_data_s.off--;	// Decrement Hall position
			} // if (0 < tst_data_s.off)
			else
			{
				tst_data_s.off = (HALL_PER_POLE - 1);	// Wrap Hall position
			} // else !(0 < tst_data_s.off)
		break; // case NEGATIVE:

		default:
			acquire_lock(); // Acquire Display Mutex
			printstrln("ERROR: Unknown Hall Spin-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( tst_data_s.curr_vect.comp_state[SPIN] )

	// Build Hall value ...

	hall_val = tst_data_s.common.phases[tst_data_s.off]; // Initialise Hall Value with phase bits
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
		convert_unsigned_to_binary_string( tst_data_s.disp_str ,hall_val ,HALL_BITS );

		acquire_lock(); // Acquire Display Mutex
		printstr( "HALL=" ); printstrln(tst_data_s.disp_str);
		release_lock(); // Release Display Mutex

	} // if (tst_data_s.print)

} // do_hall_test
/*****************************************************************************/
static int vector_compare( // Check if 2 sets of test vector are different
	TEST_VECT_TYP &vect_a, // Structure of containing 1st set of vectore components
	TEST_VECT_TYP &vect_b  // Structure of containing 2nd set of vectore components
) // return TRUE (1) if vectors are different, FALSE(0) if equal
{
	VECT_COMP_ENUM comp_cnt; // vector component counter


	for (comp_cnt=0; comp_cnt<NUM_VECT_COMPS; comp_cnt++)
	{
		if (vect_a.comp_state[comp_cnt] != vect_b.comp_state[comp_cnt]) return 1;
	} // for comp_cnt=0

	return 0; // No differences found
} // vector_compare
/*****************************************************************************/
static void do_hall_vector( // Do all tests for one Hall test vector
	GENERATE_TST_TYP &tst_data_s, // Reference to structure of Hall test data
	streaming chanend c_tst, // Channel for sending test vecotrs to test checker
	port out p4_tst,  // current port on which to transmit test data
	int test_cnt // count-down test counter
)
{
	int new_vect; // flag set if new test vector detected


	new_vect = vector_compare( tst_data_s.curr_vect ,tst_data_s.prev_vect );

	// Check for new test-vector
	if (new_vect)
	{
		c_tst <: tst_data_s.curr_vect; // transmit new test vector details to test checker

		// Check if verbose printing required
		if (tst_data_s.print)
		{
			print_test_vector( tst_data_s.common ,tst_data_s.curr_vect ,"" );
		} // if (tst_data_s.print)

		tst_data_s.prev_vect = tst_data_s.curr_vect; // update previous vector
	} // if (new_vect)

	// Loop through tests for current test vector
	while(test_cnt)
	{
		do_hall_test( tst_data_s ,p4_tst ); // Performs one Hall test

		// Check which speed mode being tested
		switch( tst_data_s.curr_vect.comp_state[SPEED] )
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
		} // switch( tst_data_s.curr_vect.comp_state[SPEED] )

		test_cnt--; // Decrement test counter
	} // while(test_cnt)

} // do_hall_vector
/*****************************************************************************/
static void gen_motor_hall_test_data( // Generate Hall Test data for one motor
	GENERATE_TST_TYP &tst_data_s, // Reference to structure of Hall test data
	streaming chanend c_tst, // Channel for sending test vecotrs to test checker
	port out p4_tst  // current port on which to transmit test data
)
{
	acquire_lock(); // Acquire Display Mutex
	printstr( " Start Test Generation For Motor_"); printintln( tst_data_s.common.options.flags[TST_MOTOR] );
	release_lock(); // Release Display Mutex

	p4_tst <: 0 @ tst_data_s.time; // Get start time

	// NB These tests assume HALL_FILTER = 0

	assign_test_vector_error( tst_data_s ,HALL_ERR_OFF ); // Set test vector to NO errors
	assign_test_vector_phase( tst_data_s ,CHANGE ); // Set test vector to Change Phase
	assign_test_vector_spin( tst_data_s ,POSITIVE ); // Set test vector to Positive-spin
	assign_test_vector_speed( tst_data_s ,FAST ); // Set test vector to constant Fast speed

	tst_data_s.curr_vect.comp_state[CNTRL] = VALID; // Settling complete, Switch on testing
	do_hall_vector( tst_data_s ,c_tst ,p4_tst ,(MAX_TESTS >> 1) );

	// Check if Error-status test activated
	if (tst_data_s.common.options.flags[TST_ERROR])
	{ // Do Error-Status test
		assign_test_vector_error( tst_data_s ,HALL_ERR_ON ); // Switch on error-bit
		tst_data_s.curr_vect.comp_state[CNTRL] = SKIP; // Switch off testing, while server counts required consecutive errors
		do_hall_vector( tst_data_s ,c_tst ,p4_tst ,MAX_HALL_STATUS_ERR );

		tst_data_s.curr_vect.comp_state[CNTRL] = VALID; // Switch on testing, now error-status should be set
		do_hall_vector( tst_data_s ,c_tst ,p4_tst ,MAX_HALL_STATUS_ERR );

		assign_test_vector_error( tst_data_s ,HALL_ERR_OFF ); // Switch off error-bit
		tst_data_s.curr_vect.comp_state[CNTRL] = SKIP; // Switch off testing, while server counts required consecutive non-errors
		do_hall_vector( tst_data_s ,c_tst ,p4_tst ,MAX_HALL_STATUS_ERR );

		tst_data_s.curr_vect.comp_state[CNTRL] = VALID; // Switch on testing, now error-status should be cleared
		do_hall_vector( tst_data_s ,c_tst ,p4_tst ,MAX_HALL_STATUS_ERR );
	} // if (tst_data_s.common.options.flags[TST_ERROR])
	else
	{ // Skip Error-Status test
		do_hall_vector( tst_data_s ,c_tst ,p4_tst ,(MAX_TESTS >> 1) );
	} // else !(tst_data_s.common.options.flags[TST_ERROR])

	// Check if Negative-spin direction test activated
	if (tst_data_s.common.options.flags[TST_NEGA])
	{ // Do Negative-spin direction test
		assign_test_vector_spin( tst_data_s ,NEGATIVE ); // Set test vector to Negative-spin
		tst_data_s.curr_vect.comp_state[CNTRL] = SKIP; // Switch off testing, while changing direction
		do_hall_vector( tst_data_s ,c_tst ,p4_tst ,2 );

		tst_data_s.curr_vect.comp_state[CNTRL] = VALID; // Direction changed, Switch on testing
		do_hall_vector( tst_data_s ,c_tst ,p4_tst ,(MAX_TESTS >> 1) );
	} // if (tst_data_s.common.options.flags[TST_NEGA])

	tst_data_s.curr_vect.comp_state[CNTRL] = QUIT; // Signal that testing has ended for current motor
	do_hall_vector( tst_data_s ,c_tst ,p4_tst ,0 );

} // gen_motor_hall_test_data
/*****************************************************************************/
void gen_all_hall_test_data( // Generate Hall Test data
	streaming chanend c_tst, // Channel for sending test vecotrs to test checker
	port out p4_tst[]  // Array of ports on which to transmit test data
)
{
	GENERATE_TST_TYP tst_data_s; // Structure of Hall test data


	init_test_data( tst_data_s ,c_tst );

	init_motor_tests( tst_data_s ); // Initialise set of motor tests

	gen_motor_hall_test_data( tst_data_s ,c_tst ,p4_tst[tst_data_s.common.options.flags[TST_MOTOR]] );

	acquire_lock(); // Acquire Display Mutex
	printstrln("");
	printstrln( "Test Generation Ends " );
	release_lock(); // Release Display Mutex

} // gen_all_hall_test_data
/*****************************************************************************/
