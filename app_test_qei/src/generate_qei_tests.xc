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
static unsigned crc_rand(
	GENERATE_TST_TYP &tst_data_s // Reference to structure of QEI test data
) // Returns random bit value
{
  crc32( tst_data_s.rnd ,~0 ,0xEDB88320 );

//MB~ acquire_lock(); printstr("R="); printintln( out_bit ); release_lock();
return (tst_data_s.rnd < (1 << 29)); // NB 12% 1's
}
/*****************************************************************************/
 static void parse_control_file( // Parse QEI control file and set up test options
	GENERATE_TST_TYP &tst_data_s // Reference to structure of QEI test data
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

  file_id = _open( "qei_tests.txt" ,O_RDONLY ,0 ); // Open control file for QEI tests

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
				tst_data_s.options.flags[test_cnt] = 0;
				tst_line = 1; // Flag test-option found
    	break; // case '0' :

    	case '1' : // Opt for this test
				tst_data_s.options.flags[test_cnt] = 1;
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
	assert( tst_data_s.options.flags[TST_MOTOR] < NUMBER_OF_MOTORS ); // Check Motor Identifier in range
 
	return;
} // parse_control_file
/*****************************************************************************/
static unsigned speed_to_ticks( // Convert Velocity (in RPM) to ticks per QEI position (in Reference Frequency Cycles)
	int speed // input speed
) // Returns time in ticks
{
	unsigned ticks; // Output ticks per QEI position (in Reference Frequency Cycles)


	ticks = (TICKS_PER_MIN_PER_QEI + (speed >> 1)) / speed;

	return ticks;
}	// speed_to_ticks
/*****************************************************************************/
static void init_test_data( // Initialise QEI Test data
	GENERATE_TST_TYP &tst_data_s, // Reference to structure of QEI test data	
	streaming chanend c_tst // Channel for sending test vecotrs to test checker
)
{
	QEI_PHASE_TYP clkwise = {{ 0 ,1 ,3 ,2 }};	// Array of QEI Phase values [BA} (NB Increment for clock-wise rotation)


	tst_data_s.phases = clkwise; // Assign QEI Phase values (NB Increment for clock-wise rotation)

	// Convert Speed values to Ticks/QEI_position values
	tst_data_s.hi_ticks = speed_to_ticks( HIGH_SPEED ); // Convert HIGH_SPEED to ticks
	tst_data_s.lo_ticks = speed_to_ticks( LOW_SPEED ); // Convert LOW_SPEED to ticks

	tst_data_s.rnd = 0xffffffff; // Initialise random number
	tst_data_s.print = VERBOSE_PRINT; // Set print mode
	tst_data_s.dbg = 0; // Set debug mode

	parse_control_file( tst_data_s ); 

	c_tst <: tst_data_s.options; // Send test options to checker core
} // init_test_data
/*****************************************************************************/
static void init_motor_tests( // Initialisation for each set of motor tests
	GENERATE_TST_TYP &tst_data_s // Reference to structure of QEI test data
)
{
	tst_data_s.period = tst_data_s.lo_ticks; // Set period for Low start speed
	tst_data_s.off = 0; // Initialise  QEI phase offset
	tst_data_s.curr_vect.comp_state[CNTRL] = SKIP; // Initialise to skipped test for set-up mode
	tst_data_s.prev_vect.comp_state[CNTRL] = QUIT; // Initialise to something that will force an update
	tst_data_s.prev_qei = QEI_NERR_MASK; // Initialise to starting value

	// Start QEI counter to a sensibly short time before origin fires.
	tst_data_s.cnt = (-20) & QEI_REV_MASK;
} // init_motor_tests
/*****************************************************************************/
static void assign_test_vector_error( // Assign Error-state of test vector
	GENERATE_TST_TYP &tst_data_s, // Reference to structure of QEI test data
	ERROR_QEI_ENUM inp_err // Input Error-state
)
{
	switch( inp_err )
	{
		case QEI_ERR_OFF: // No Errors
			tst_data_s.nerr = QEI_NERR_MASK; // Set error flag to NO errors (Bit_3 = 1)
		break; // case QEI_ERR_OFF:

		case QEI_ERR_ON: // Force Error
			tst_data_s.nerr = 0; // Clear (Bit_3) to Signal error condition
		break; // case QEI_ERR_OFF:

		default:
			acquire_lock(); // Acquire Display Mutex
			printstrln("ERROR: Unknown QEI Error-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( inp_err )

	tst_data_s.curr_vect.comp_state[ERROR] = inp_err; // Update Error-state of test vector
} // assign_test_vector_error
/*****************************************************************************/
static void assign_test_vector_origin( // Assign Origin-state of test vector
	GENERATE_TST_TYP &tst_data_s, // Reference to structure of QEI test data
	ORIG_QEI_ENUM inp_orig // Input Origin-state
)
{
	tst_data_s.curr_vect.comp_state[ORIGIN] = inp_orig; // Update Error-state of test vector
} // assign_test_vector_origin
/*****************************************************************************/
static void assign_test_vector_spin( // Assign Spin-state of test vector
	GENERATE_TST_TYP &tst_data_s, // Reference to structure of QEI test data
	SPIN_QEI_ENUM inp_spin // Input Spin-state
)
{
	switch( inp_spin )
	{
		case CLOCK: // Clock-wise
			tst_data_s.inc = 1; // Increment for Clock-Wise direction
		break; // case CLOCK:

		case ANTI: // Anti-clockwise
			tst_data_s.inc = -1; // Decrement for Anti-Clockwise direction
		break; // case ANTI:

		default:
			acquire_lock(); // Acquire Display Mutex
			printstrln("ERROR: Unknown QEI Spin-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( inp_spin )

	tst_data_s.curr_vect.comp_state[SPIN] = inp_spin; // Update Spin-state of test vector
} // assign_test_vector_spin
/*****************************************************************************/
static void assign_test_vector_speed( // Assign Speed-state of test vector
	GENERATE_TST_TYP &tst_data_s, // Reference to structure of QEI test data
	SPEED_QEI_ENUM inp_speed // Input speed-state
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
			printstrln("ERROR: Unknown QEI Velocity-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( inp_speed )

	tst_data_s.curr_vect.comp_state[SPEED] = inp_speed; // Update speed-state of test vector
} // assign_test_vector_speed
/*****************************************************************************/
static void do_qei_test( // Performs one QEI test
	GENERATE_TST_TYP &tst_data_s, // Reference to structure of QEI test data
	streaming chanend c_disp, // Channel for sending display data to print scheduler core
	buffered port:4 out pb4_tst  // current port on which to transmit test data
)
{
	int time_cnt; // Time counter
	int grace_time; // Available time between QEI value transmissions 
	QEI_RAW_TYP qei_val;  // QEI value [E I B A]
	PORT_TIME_TYP port_time; // value of port timer
	PORT_TIME_TYP elapsed_time; // elapsed time since last QEI transmission
	unsigned big_time; // Big timer value


	tst_data_s.cnt = (tst_data_s.cnt + tst_data_s.inc) & QEI_REV_MASK; // Increment/Decrement (and wrap) QEI position
	tst_data_s.off = (tst_data_s.cnt & QEI_PHASE_MASK); // get phase position for current QEI position

	// Build QEI value ...

	// Check if at origin
	if (0 == tst_data_s.cnt)
	{
		tst_data_s.orig = QEI_ORIG_MASK; // Set origin flag (Bit_2 = 1)
	} // if (0 == tst_data_s.cnt)
	else
	{
		tst_data_s.orig = 0; // Clear origin flag (Bit_2 = 0)
	} // if (0 == tst_data_s.cnt)

	qei_val = tst_data_s.phases.vals[tst_data_s.off]; // Initialise QEI Value with phase bits
	qei_val |= tst_data_s.orig; // OR with origin flag (Bit_2)
	qei_val |= tst_data_s.nerr; // OR with error flag (Bit_3)

	if (crc_rand(tst_data_s)) qei_val ^= 1; // Flip bit_0 with low probability
	if (crc_rand(tst_data_s)) qei_val ^= 2; // Flip bit-1 with low probability 

	// Wait till test period elapsed ...

	tst_data_s.Big_Timer :> big_time; // Get current time from Big timer
	assert( big_time < (tst_data_s.tx_time + (unsigned)NUM_PORT_TIMES)); // ERROR: port-time wrapped. Logic broken

	pb4_tst <: tst_data_s.prev_qei @ port_time;	// Re-send old data to get current port time

	elapsed_time = (PORT_TIME_TYP)(port_time - tst_data_s.time_p); // Calculate time since last QEI transmission (NB Automaticallly wraps)
	grace_time = (int)tst_data_s.period - (int)MIN_WAIT_TIME;	// Calculate available time between QEI value transmissions 

	// Check if enough time to set up timed port-write
	if ((int)elapsed_time > grace_time)
	{ // Time slot missed, execute immediately
		pb4_tst <: qei_val; // send new value
		tst_data_s.Big_Timer :> tst_data_s.tx_time; // Store transmission time

		tst_data_s.time_p += (PORT_TIME_TYP)tst_data_s.period; // Calculate missed QEI transmission time
	}	// if (grace_time < MIN_WAIT_TIME)
	else
	{
		time_cnt = (int)tst_data_s.period; // Initialise count-down timer
		tst_data_s.time_p += (PORT_TIME_TYP)time_cnt; // Update transmit time in port-time range

		// Check for clock wrap
		while (time_cnt >= NUM_PORT_TIMES)
		{	// Wait for whole sequence of all possible clock tick values (NUM_PORT_TIMES)
			pb4_tst @ tst_data_s.time_p <: tst_data_s.prev_qei; // Resend old value

			time_cnt -= (int)NUM_PORT_TIMES; // Decrement count by No. of values in sequence
		} // while (time_cnt >= NUM_PORT_TIMES)
	
		pb4_tst @ tst_data_s.time_p <: qei_val; // Now send new value at correct time
		tst_data_s.Big_Timer :> tst_data_s.tx_time; // Store transmission time
	} // else !(grace_time < MIN_WAIT_TIME)

	tst_data_s.prev_qei = qei_val; // Store QEI value

	if (tst_data_s.print)
	{
		// NB There is no time to print the data here, so send it to another core.
		c_disp <: DISP_CLASS_GENR8; // Signal transmission of QEI value
		c_disp <: qei_val; // Send QEI data
	} // if (tst_data_s.print)

} // do_qei_test
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
static void do_qei_vector( // Do all tests for one QEI test vector
	GENERATE_TST_TYP &tst_data_s, // Reference to structure of QEI test data
	streaming chanend c_tst, // Channel for sending test vecotrs to test checker
	streaming chanend c_disp, // Channel for sending display data to print scheduler core
	buffered port:4 out pb4_tst,  // current port on which to transmit test data
	int test_cnt // count-down test counter, (NB or -1 for un-used)
)
{
	int new_vect; // flag set if new test vector detected
	unsigned post_cnt; // Number of counts after origin fires


	// Loop through tests for current test vector
	while(test_cnt)
	{
		test_cnt--; // Decrement test counter

		// Check if origin expected to fire
		if (0 == tst_data_s.cnt)
		{ // Origin expected: Switch on Origin test
			assign_test_vector_origin( tst_data_s ,ORIG_ON ); // Set test vector to Origin Expected
		} // if (0 == tst_data_s.cnt)
		else
		{ // Origin NOT expected

			// Calculate counter ticks past origin
			post_cnt = ((unsigned)((int)tst_data_s.cnt * tst_data_s.inc)) & QEI_REV_MASK;

			// Check if time to switch off origin test
			if (2 == post_cnt)
			{ // Swithc off origin test
				assign_test_vector_origin( tst_data_s ,ORIG_OFF ); // Set test vector to No Origin
			} // if (1 == post_cnt)
		} // else !(0 == tst_data_s.cnt)

		new_vect = vector_compare( tst_data_s.curr_vect ,tst_data_s.prev_vect );
	
		// Check for new test-vector
		if (new_vect)
		{
			c_tst <: tst_data_s.curr_vect; // transmit new test vector details to test checker
	
			// Check if verbose printing required
			if (tst_data_s.print)
			{
				c_disp <: DISP_CLASS_VECT; // Signal transmission of test vector to print master
				c_disp <: tst_data_s.curr_vect; // Send test vector data
			} // if (tst_data_s.print)
	
			tst_data_s.prev_vect = tst_data_s.curr_vect; // update previous vector
		} // if (new_vect)

		do_qei_test( tst_data_s ,c_disp ,pb4_tst ); // Performs one QEI test

		// Check which speed mode being tested
		switch( tst_data_s.curr_vect.comp_state[SPEED] )
		{
			case ACCEL: // Accelerate
				tst_data_s.period = (tst_data_s.scale * tst_data_s.period + HALF_SCALE) >> SCALE_PRECISION; // Alter period to change speed

				// Check for end of acceleration phase
				if (tst_data_s.period < tst_data_s.hi_ticks)
				{
				  tst_data_s.period = tst_data_s.hi_ticks; // Clip into range
					test_cnt = 0; // End-test
				} // if (tst_data_s.period < tst_data_s.hi_ticks)
			break; // case ACCEL:
	
			case FAST: // Constant Fast Speed
			break; // case FAST:
	
			case DECEL: // Accelerate
				tst_data_s.period = (tst_data_s.scale * tst_data_s.period + HALF_SCALE) >> SCALE_PRECISION; // Alter period to change speed

				// Check for end of deceleration phase
				if (tst_data_s.period > tst_data_s.lo_ticks)
				{
				  tst_data_s.period = tst_data_s.lo_ticks; // Clip into range  
					test_cnt = 0; // End-test
				} // if (tst_data_s.period > tst_data_s.lo_ticks)
			break; // case ACCEL:
	
			case SLOW: // Constant Slow Speed
			break; // case FAST:
	
			default:
				acquire_lock(); // Acquire Display Mutex
				printstrln("ERROR: Unknown QEI Velocity-state");
				release_lock(); // Release Display Mutex
				assert(0 == 1);
			break; // default:
		} // switch( tst_data_s.curr_vect.comp_state[SPEED] )
	} // while(test_cnt)

} // do_qei_vector
/*****************************************************************************/
static void gen_motor_qei_test_data( // Generate QEI Test data for one motor
	GENERATE_TST_TYP &tst_data_s, // Reference to structure of QEI test data
	streaming chanend c_tst, // Channel for sending test vectors to test checker
	streaming chanend c_disp, // Channel for sending display data to print scheduler core
	buffered port:4 out pb4_tst  // current port on which to transmit test data
)
{
	acquire_lock(); // Acquire Display Mutex
	printstr( " Start Test Generation For Motor_"); printintln( tst_data_s.options.flags[TST_MOTOR] );
	release_lock(); // Release Display Mutex

	tst_data_s.Big_Timer :> tst_data_s.tx_time; // Get start time from Big Timer
	pb4_tst <: 0 @ tst_data_s.time_p; // Get start time form Port Timer

	// NB These tests assume QEI_FILTER = 0
	assign_test_vector_origin( tst_data_s ,ORIG_OFF ); // Set test vector to No Origin
	assign_test_vector_error( tst_data_s ,QEI_ERR_OFF ); // Set test vector to NO errors
	assign_test_vector_spin( tst_data_s ,CLOCK ); // Set test vector to Clock-wise spin
	assign_test_vector_speed( tst_data_s ,ACCEL ); // Set test vector to Accelerate

// #ifdef MB
	tst_data_s.curr_vect.comp_state[CNTRL] = SKIP; // Switch off testing
	do_qei_vector( tst_data_s ,c_tst ,c_disp ,pb4_tst ,(START_UP_CHANGES - 1) ); // Start-up phase

	tst_data_s.curr_vect.comp_state[CNTRL] = VALID; // Start-up complete, Switch on testing
	do_qei_vector( tst_data_s ,c_tst ,c_disp ,pb4_tst ,-1 ); // Accelerate up to max speed

	// NB Do Origin tests while motor running fast, to speed-up simulation

	assign_test_vector_speed( tst_data_s ,FAST ); // Set test vector to constant Fast speed
	tst_data_s.curr_vect.comp_state[CNTRL] = SKIP; // Switch off testing while motor settles
	do_qei_vector( tst_data_s ,c_tst ,c_disp ,pb4_tst ,3 );

	tst_data_s.curr_vect.comp_state[CNTRL] = VALID; // Settling complete, Switch on testing
	do_qei_vector( tst_data_s ,c_tst ,c_disp ,pb4_tst ,(MAX_TESTS >> 1) ); // NB Origin traversed

	// Check if Error-status test activated
	if (tst_data_s.options.flags[TST_ERROR])
	{ // Do Error-Status test
		assign_test_vector_error( tst_data_s ,QEI_ERR_ON ); // Switch on error-bit
		tst_data_s.curr_vect.comp_state[CNTRL] = SKIP; // Switch off testing, while server counts required consecutive errors
		do_qei_vector( tst_data_s ,c_tst ,c_disp ,pb4_tst ,MAX_QEI_STATUS_ERR );
	
		tst_data_s.curr_vect.comp_state[CNTRL] = VALID; // Switch on testing, now error-status should be set
		do_qei_vector( tst_data_s ,c_tst ,c_disp ,pb4_tst ,MAX_QEI_STATUS_ERR );
	
		assign_test_vector_error( tst_data_s ,QEI_ERR_OFF ); // Switch off error-bit
		tst_data_s.curr_vect.comp_state[CNTRL] = SKIP; // Switch off testing, while server counts required consecutive non-errors
		do_qei_vector( tst_data_s ,c_tst ,c_disp ,pb4_tst ,MAX_QEI_STATUS_ERR );
	
		tst_data_s.curr_vect.comp_state[CNTRL] = VALID; // Switch on testing, now error-status should be cleared
		do_qei_vector( tst_data_s ,c_tst ,c_disp ,pb4_tst ,((MAX_TESTS - (3 *MAX_QEI_STATUS_ERR)) >> 1) );
	} // if (tst_data_s.options.flags[TST_ERROR])
	else
	{ // Skip Error-Status test
		do_qei_vector( tst_data_s ,c_tst ,c_disp ,pb4_tst ,(MAX_TESTS >> 1) );
	} // else !(tst_data_s.options.flags[TST_ERROR])

	assign_test_vector_speed( tst_data_s ,DECEL ); // Set test vector to Decelerate
	tst_data_s.curr_vect.comp_state[CNTRL] = SKIP; // Switch off testing while motor starts braking
	do_qei_vector( tst_data_s ,c_tst ,c_disp ,pb4_tst ,1 );

	tst_data_s.curr_vect.comp_state[CNTRL] = VALID; // Braking started, Switch on testing
	do_qei_vector( tst_data_s ,c_tst ,c_disp ,pb4_tst ,-1 ); // Deccelerate down to min. speed
// #endif //MB~
//MB~ assign_test_vector_spin( tst_data_s ,ANTI); // Set test vector to Clock-wise spin

	assign_test_vector_speed( tst_data_s ,SLOW ); // Set test vector to constant Slow speed
	tst_data_s.curr_vect.comp_state[CNTRL] = SKIP; // Switch off testing while motor settles
	do_qei_vector( tst_data_s ,c_tst ,c_disp ,pb4_tst ,3 );

	tst_data_s.curr_vect.comp_state[CNTRL] = VALID; // Settling complete, Switch on testing
	do_qei_vector( tst_data_s ,c_tst ,c_disp ,pb4_tst ,MIN_TESTS );

	// Check if Anti-clockwise tests activated
	if (tst_data_s.options.flags[TST_ANTI])
	{ // Do Anti-Clockwise test
		assign_test_vector_spin( tst_data_s ,ANTI ); // Set test vector to Anti-clockwise spin
		tst_data_s.curr_vect.comp_state[CNTRL] = SKIP; // Switch off testing, until server is confident of new spin direction
		do_qei_vector( tst_data_s ,c_tst ,c_disp ,pb4_tst ,(MAX_CONFID + 2) );
	
		tst_data_s.curr_vect.comp_state[CNTRL] = VALID; // Server confident, Switch on testing
		do_qei_vector( tst_data_s ,c_tst ,c_disp ,pb4_tst ,MIN_TESTS );

		assign_test_vector_speed( tst_data_s ,ACCEL ); // Set test vector to Accelerate
		tst_data_s.curr_vect.comp_state[CNTRL] = SKIP; // Switch off testing while motor starts accelerating
		do_qei_vector( tst_data_s ,c_tst ,c_disp ,pb4_tst ,1 );
	
		tst_data_s.curr_vect.comp_state[CNTRL] = VALID; // Acceleration started, Switch on testing
		do_qei_vector( tst_data_s ,c_tst ,c_disp ,pb4_tst ,-1 ); // Accelerate up to max speed
	
		assign_test_vector_speed( tst_data_s ,FAST ); // Set test vector to constant Fast speed
		tst_data_s.curr_vect.comp_state[CNTRL] = SKIP; // Switch off testing while motor settles
		do_qei_vector( tst_data_s ,c_tst ,c_disp ,pb4_tst ,3 );
	
		tst_data_s.curr_vect.comp_state[CNTRL] = VALID; // Settling complete, Switch on testing
		do_qei_vector( tst_data_s ,c_tst ,c_disp ,pb4_tst ,MAX_TESTS );
	
		assign_test_vector_speed( tst_data_s ,DECEL ); // Set test vector to Decelerate
		tst_data_s.curr_vect.comp_state[CNTRL] = SKIP; // Switch off testing while motor starts braking
		do_qei_vector( tst_data_s ,c_tst ,c_disp ,pb4_tst ,1 );
	
		tst_data_s.curr_vect.comp_state[CNTRL] = VALID; // Braking started, Switch on testing
		do_qei_vector( tst_data_s ,c_tst ,c_disp ,pb4_tst ,-1 ); // Deccelerate down to min. speed
	} // if (tst_data_s.options.flags[TST_ANTI])

	tst_data_s.curr_vect.comp_state[CNTRL] = QUIT; // Signal that testing has ended for current motor
	do_qei_vector( tst_data_s ,c_tst ,c_disp ,pb4_tst ,1 );

} // gen_motor_qei_test_data
/*****************************************************************************/
void gen_all_qei_test_data( // Generate QEI Test data
	const COMMON_TST_TYP &comm_data_s, // Structure containing common test data
	streaming chanend c_tst, // Channel for sending test vecotrs to test checker
	streaming chanend c_disp, // Channel for sending display data to print scheduler core
	buffered port:4 out pb4_tst[]  // Array of ports on which to transmit test data
)
{
	GENERATE_TST_TYP tst_data_s; // Structure of QEI test data


	init_test_data( tst_data_s ,c_tst );

	init_motor_tests( tst_data_s ); // Initialise set of motor tests

	gen_motor_qei_test_data( tst_data_s ,c_tst ,c_disp ,pb4_tst[tst_data_s.options.flags[TST_MOTOR]] );

	acquire_lock(); // Acquire Display Mutex
	printstrln("");
	printstrln( "Test Generation Ends " );
	release_lock(); // Release Display Mutex

} // gen_all_qei_test_data
/*****************************************************************************/
