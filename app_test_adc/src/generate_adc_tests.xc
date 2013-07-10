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

#include "generate_adc_tests.h"

/*****************************************************************************/
static void parse_control_file( // Parse ADC control file and set up test options
	GENERATE_TST_TYP &tst_data_s // Reference to structure of ADC test data
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

  file_id = _open( "adc_tests.txt" ,O_RDONLY ,0 ); // Open control file for ADC tests

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
 
	return;
} // parse_control_file
/*****************************************************************************/
static void init_test_data( // Initialise ADC Test data
	GENERATE_TST_TYP &tst_data_s, // Reference to structure of ADC test data	
	streaming chanend c_chk // Channel for communication with Checker cores
)
{
	int opt_flag; // Options flag


	init_common_data( tst_data_s.common ); // Initialise data common to Generator and Checker
 
	tst_data_s.period = ADC_PERIOD; // Typical time between ADC capture in FOC motor control loop

	tst_data_s.print = PRINT_TST_ADC; // Set print mode
	tst_data_s.dbg = 0; // Set debug mode

	parse_control_file( tst_data_s ); 

	// Check for sensible options ...

	opt_flag =	tst_data_s.common.options.flags[TST_SMALL] || 
							tst_data_s.common.options.flags[TST_PACE] ||
							tst_data_s.common.options.flags[TST_SLOW];

	if (0 == opt_flag)
	{
		acquire_lock(); 
		printstrln("ERROR Reading Test-Options file. Please select one of Selection_1/Selection_2/Slow_Speed");
		printstrln("      Aborting Program");
		printstrln("");
		release_lock();
		_exit(0);
	} // if (0 == opt_flag)

	c_chk <: tst_data_s.common.options; // Send test options to checker core

	tst_data_s.curr_vect.comp_state[CNTRL] = NO_PACE; // Initialise to No pacing (Fast execution)
	tst_data_s.prev_vect.comp_state[CNTRL] = QUIT; // Initialise to something that will force an update

} // init_motor_tests
/*****************************************************************************/
static void assign_test_vector_sum( // Assign Zero-sum state of test vector
	GENERATE_TST_TYP &tst_data_s, // Reference to structure of ADC test data
	SUM_ADC_ENUM inp_sum // Input Zero-sum state
)
{
	tst_data_s.curr_vect.comp_state[SUM] = inp_sum; // Update Zero-Sum state of test vector
} // assign_test_vector_sum
/*****************************************************************************/
static void assign_test_vector_spin( // Assign Spin-state of test vector
	GENERATE_TST_TYP &tst_data_s, // Reference to structure of ADC test data
	SPIN_ADC_ENUM inp_spin // Input Spin-state
)
{
	tst_data_s.curr_vect.comp_state[SPIN] = inp_spin; // Update Spin-state of test vector
} // assign_test_vector_spin
/*****************************************************************************/
static void assign_test_vector_gain( // Assign Gain-state of test vector
	GENERATE_TST_TYP &tst_data_s, // Reference to structure of ADC test data
	GAIN_ADC_ENUM inp_gain // Input gain-state
)
{
	tst_data_s.curr_vect.comp_state[GAIN] = inp_gain; // Update gain-state of test vector
} // assign_test_vector_gain
/*****************************************************************************/
static void assign_test_vector_speed( // Assign Speed-state of test vector
	GENERATE_TST_TYP &tst_data_s, // Reference to structure of ADC test data
	SPEED_ADC_ENUM inp_speed // Input speed-state
)
{
	tst_data_s.curr_vect.comp_state[SPEED] = inp_speed; // Update speed-state of test vector
} // assign_test_vector_speed
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
static void do_adc_vector( // Do all tests for one ADC test vector
	GENERATE_TST_TYP &tst_data_s, // Reference to structure of ADC test data
	streaming chanend c_sin, // Channel for communication with Sine_Generator cores
	streaming chanend c_chk // Channel for communication with Checker cores
)
{
	int new_vect; // flag set if new test vector detected


	new_vect = vector_compare( tst_data_s.curr_vect ,tst_data_s.prev_vect );

	// Check for new test-vector
	if (new_vect)
	{
		c_chk <: tst_data_s.curr_vect; // transmit new test vector details to test checker
		c_sin <: tst_data_s.curr_vect; // transmit new test vector details to Sine_Generator

		// Check if verbose printing required
		if (tst_data_s.print)
		{
			print_test_vector( tst_data_s.common ,tst_data_s.curr_vect ,"" );
		} // if (tst_data_s.print)

// acquire_lock(); printstrln("GEN-S_Rchk"); release_lock(); // MB~
		c_chk :> int _; // Receive test complete signal
		tst_data_s.prev_vect = tst_data_s.curr_vect; // update previous vector
	} // if (new_vect)

} // do_adc_vector
/*****************************************************************************/
static void gen_motor_adc_test_data( // Generate ADC Test data for one motor
	GENERATE_TST_TYP &tst_data_s, // Reference to structure of ADC test data
	streaming chanend c_sin, // Channel for communication with Sine_Generator cores
	streaming chanend c_chk // Channel for communication with Checker cores
)
{
	if (tst_data_s.print)
	{
		acquire_lock(); // Acquire Display Mutex
		printstr( " Start Test Gen. For Motor_"); printintln( MOTOR_ID );
		release_lock(); // Release Display Mutex
	} // if (tst_data_s.print)

	// NB These tests assume ADC_FILTER = 0

	if (tst_data_s.common.options.flags[TST_SUM])
	{
		assign_test_vector_sum( tst_data_s ,SUM_ON ); // Set test vector to test zero-sum
	} // if (tst_data_s.common.options.flags[TST_SUM])
	else
	{
		assign_test_vector_sum( tst_data_s ,NO_SUM ); // Set test vector to skip zero-sum tests
	} // if (tst_data_s.common.options.flags[TST_SUM])

	// Check if Small-Gain tests activated
	if (tst_data_s.common.options.flags[TST_SMALL])
	{
		assign_test_vector_speed( tst_data_s ,FAST ); // Set test vector to Fast Speed to shorten time.
		assign_test_vector_gain( tst_data_s ,SMALL ); // Set test vector to Small gain
		assign_test_vector_spin( tst_data_s ,CLOCK ); // Set test vector to Clock-wise spin
		tst_data_s.curr_vect.comp_state[CNTRL] = NO_PACE; // Set control to No Pacing (Fast Execution)

		do_adc_vector( tst_data_s ,c_sin ,c_chk );
	} // if (tst_data_s.common.options.flags[TST_SMALL] || etc.

	// Check if Pacing tests activated
	if (tst_data_s.common.options.flags[TST_PACE])
	{
		assign_test_vector_speed( tst_data_s ,FAST ); // Set test vector to Fast Speed to shorten time.
		assign_test_vector_gain( tst_data_s ,LARGE ); // Set test vector to Large gain
		assign_test_vector_spin( tst_data_s ,ANTI ); // Set test vector to Anti-clockwise spin
		tst_data_s.curr_vect.comp_state[CNTRL] = PACE_ON; // Set control to Paced (Slow Execution)

		do_adc_vector( tst_data_s ,c_sin ,c_chk );
	} // if (tst_data_s.common.options.flags[TST_PACE] || etc.

	// Check if Slow test activated
	if (tst_data_s.common.options.flags[TST_SLOW])
	{ // Do Slow speed test
		assign_test_vector_speed( tst_data_s ,SLOW ); // Set test vector to Slow Speed (very long).
		assign_test_vector_gain( tst_data_s ,LARGE ); // Set test vector to Large gain
		assign_test_vector_spin( tst_data_s ,CLOCK ); // Set test vector to Clock-wise spin
		tst_data_s.curr_vect.comp_state[CNTRL] = NO_PACE; // Set control to No Pacing (Fast Execution)

		do_adc_vector( tst_data_s ,c_sin ,c_chk );
	} // if (tst_data_s.common.options.flags[TST_SLOW])

	assign_test_vector_gain( tst_data_s ,ZERO ); // Set test vector to Zero Gain (Stops ADC values)
	tst_data_s.curr_vect.comp_state[CNTRL] = QUIT; // Signal that testing has ended for current motor
	do_adc_vector( tst_data_s ,c_sin ,c_chk );

} // gen_motor_adc_test_data
/*****************************************************************************/
void gen_all_adc_test_data( // Generate ADC Test data
	streaming chanend c_chk, // Channel for communication with Checker cores
	streaming chanend c_sin // Channel for communication with Sine_Generator cores
)
{
	GENERATE_TST_TYP tst_data_s; // Structure of ADC test data


	init_test_data( tst_data_s ,c_chk );

	gen_motor_adc_test_data( tst_data_s ,c_sin ,c_chk );

	if (tst_data_s.print)
	{
		acquire_lock(); // Acquire Display Mutex
		printstrln( "Test Generation Ends " );
		release_lock(); // Release Display Mutex
	} // if (tst_data_s.print)
} // gen_all_adc_test_data
/*****************************************************************************/
