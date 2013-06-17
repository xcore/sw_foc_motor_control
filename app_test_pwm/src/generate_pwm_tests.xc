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

#include "generate_pwm_tests.h"

/*****************************************************************************/
static void parse_control_file( // Parse PWM control file and set up test options
	GENERATE_PWM_TYP &tst_data_s // Reference to structure of PWM test data
)
{
  unsigned char file_buf[FILE_SIZE];
  int char_cnt; // Character counter
  int line_cnt = 0; // line counter
  int test_cnt = 0; // test counter
  int new_line = 0; // flag if new-line found
  char curr_char; // Current character
  int file_id;


	// Initialise file buffer
  for (char_cnt = 0; char_cnt < FILE_SIZE; ++char_cnt) 
	{
    file_buf[char_cnt] = 0;
  } // for char_cnt

  file_id = _open( "pwm_tests.txt" ,O_RDONLY ,0 ); // Open control file for PWM tests

  if (-1 == file_id) 
	{
    printstrln("Error: _open failed");
    exit(1);
  } // if (-1 == file_id) 

  // An open file can be read using the *_read* system call
  _read( file_id ,file_buf ,FILE_SIZE );

  // We can then close an open file using the *_close* system call
  if (0 != _close(file_id)) 
	{
    printstrln("Error: _close failed.");
    exit(1);
  } // if (0 != _close(file_id)) 

	acquire_lock(); // Acquire Display Mutex
	printstrln("Read following Test Options ..." );

	// Parse the file buffer for test options
  for (char_cnt = 0; char_cnt < FILE_SIZE; ++char_cnt) 
	{
    curr_char = file_buf[char_cnt]; // Get next character

    if (!curr_char) break; // Check for end of file info.


		switch (curr_char)
		{
    	case '\n' : // End of line.
				new_line = 1; // Set flag for new-line
    	break; // case '\n' :

    	case '0' : // Opt out of test
				tst_data_s.common.options.flags[test_cnt] = 0;
				test_cnt++;
				new_line = 1; // Set flag for new-line
				printchar(curr_char);
				printchar(' ');
    	break; // case '0' :

    	case '1' : // Opt for this test
				tst_data_s.common.options.flags[test_cnt] = 1;
				test_cnt++;
				new_line = 1; // Set flag for new-line
				printchar(curr_char);
				printchar(' ');
    	break; // case '1' :

    	case '#' : // Start of comment
				new_line = 1; // Set flag for new-line
    	break; // case '1' :

    	default : // Whitespace
    	break; // case '\n' :
		} // switch (curr_char)

		// Check if we need to move to new-line
		if (new_line)
		{ // skip to next line
			while ('\n' != file_buf[char_cnt])
			{
				char_cnt++;
				assert(char_cnt < FILE_SIZE); // End-of-file found
				printchar( file_buf[char_cnt] );
			} // while ('\n' != file_buf[char_cnt])

			line_cnt++;
			new_line = 0; // Clear new_line flag
		} // if (new_line)
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
static void init_pwm( // Initialise PWM parameters for one motor
	PWM_COMMS_TYP &pwm_comms_s, // reference to structure containing PWM data
	chanend c_pwm, // PWM channel connecting Client & Server
	unsigned motor_id // Unique Motor identifier e.g. 0 or 1
)
{
	int phase_cnt; // phase counter


	pwm_comms_s.buf = 0; // Current double-buffer in use at shared memory address
	pwm_comms_s.params.id = motor_id; // Unique Motor identifier e.g. 0 or 1

	// initialise arrays
	for (phase_cnt = 0; phase_cnt < NUM_PWM_PHASES; phase_cnt++)
	{ 
		pwm_comms_s.params.widths[phase_cnt] = 0;
	} // for phase_cnt

	// Receive the address of PWM data structure from the PWM server, in case shared memory is used
	c_pwm :> pwm_comms_s.mem_addr; // Receive shared memory address from PWM server

	return;
} // init_pwm 
/*****************************************************************************/
static void init_test_data( // Initialise PWM Test data
	GENERATE_PWM_TYP &tst_data_s, // Reference to structure of PWM test data
	streaming chanend c_tst // Channel for sending test vecotrs to test checker
)
{
	init_common_data( tst_data_s.common ); // Initialise data common to Generator and Checker
 
	tst_data_s.print = PRINT_TST_PWM; // Set print mode
	tst_data_s.dbg = 0; // Set debug mode

	tst_data_s.period = PWM_PERIOD; // Set period between generations of PWM Client data
	tst_data_s.vector.comp_state[CNTRL] = SKIP; // Initialise to skipped test for set-up mode

	parse_control_file( tst_data_s ); 

	c_tst <: tst_data_s.common.options; // Send test options to checker core
} // init_motor_tests
/*****************************************************************************/
static void assign_test_vector_width( // Assign Width-state of test vector
	GENERATE_PWM_TYP &tst_data_s, // Reference to structure of PWM test data
	WIDTH_PWM_ENUM inp_width // Input Width-state
)
{
	tst_data_s.vector.comp_state[WIDTH] = inp_width; // Update speed-state of test vector

	tst_data_s.width = tst_data_s.common.pwm_wids[inp_width]; // Set pulse width for current width-state
} // assign_test_vector_width
/*****************************************************************************/
static void assign_test_vector_phase( // Assign Phase-state of test vector
	GENERATE_PWM_TYP &tst_data_s, // Reference to structure of PWM test data
	PWM_PHASE_ENUM inp_phase // Input phase-state
)
{
	tst_data_s.vector.comp_state[PHASE] = inp_phase; // Update phase-state of test vector
} // assign_test_vector_phase
/*****************************************************************************/
static void assign_test_vector_leg( // Assign PWM-leg state of test vector
	GENERATE_PWM_TYP &tst_data_s, // Reference to structure of PWM test data
	PWM_LEG_ENUM inp_leg // Input leg-state
)
{
	tst_data_s.vector.comp_state[LEG] = inp_leg; // Update PWM-leg state of test vector
} // assign_test_vector_leg
/*****************************************************************************/
static void assign_test_vector_adc( // Assign ADC_trigger state of test vector
	GENERATE_PWM_TYP &tst_data_s, // Reference to structure of PWM test data
	ADC_PWM_ENUM inp_adc	// Input adc-trigger state
)
{
	tst_data_s.vector.comp_state[ADC_TRIG] = inp_adc;
} // assign_test_vector_leg
/*****************************************************************************/
static void do_pwm_test( // Performs one PWM test
	GENERATE_PWM_TYP &tst_data_s, // Reference to structure of PWM test data
	chanend c_pwm 				// Channel between Client and Server
)
{
	timer chronometer; // timer


	tst_data_s.time += tst_data_s.period; // Update time for next PWM pulse generation

	// Load test data into PWM phase under test
	tst_data_s.pwm_comms.params.widths[tst_data_s.vector.comp_state[PHASE]] = tst_data_s.width;

// acquire_lock(); printstr( "GP=" ); printuintln( tst_data_s.width ); release_lock(); //MB~

	if (0 == tst_data_s.print)
	{
		printchar('.'); // Progress indicator
	} // if (0 == tst_data_s.print)

	chronometer when timerafter(tst_data_s.time) :> void;	// Wait till test period elapsed

	foc_pwm_put_parameters( tst_data_s.pwm_comms ,c_pwm ); // Update the PWM values

	if (tst_data_s.print)
	{
		acquire_lock(); // Acquire Display Mutex
		printstr( "PWM:" ); printintln( tst_data_s.width ); 
		release_lock(); // Release Display Mutex
	} // if (tst_data_s.print)

} // do_pwm_test
/*****************************************************************************/
static void do_pwm_vector( // Do all tests for one PWM test vector
	GENERATE_PWM_TYP &tst_data_s, // Reference to structure of PWM test data
	streaming chanend c_tst, // Channel for sending test vecotrs to test checker
	chanend c_pwm, 				// Channel between Client and Server
	int test_cnt // count-down test counter
)
{
	c_tst <: tst_data_s.vector; // transmit test vector details to test checker

	if (tst_data_s.print)
	{
		print_test_vector( tst_data_s.common ,tst_data_s.vector ,"" );
	} // if (tst_data_s.print)

	// Loop through tests for current test vector
	while(test_cnt)
	{
		do_pwm_test( tst_data_s ,c_pwm ); // Performs one PWM test

		test_cnt--; // Decrement test counter
	} // while(test_cnt)

} // do_pwm_vector
/*****************************************************************************/
static void gen_pwm_width_test( // Generate PWM Test data for testing one PWM Pulse-width
	GENERATE_PWM_TYP &tst_data_s, // Reference to structure of PWM test data
	streaming chanend c_tst, // Channel for sending test vecotrs to test checker
	chanend c_pwm, 				// Channel between Client and Server
	WIDTH_PWM_ENUM curr_wid // Current PWM Pulse-width under test
)
{
	assign_test_vector_width( tst_data_s ,curr_wid ); // Set test vector to Slow width
	
	tst_data_s.vector.comp_state[CNTRL] = SKIP; // Skip start-up
	do_pwm_vector( tst_data_s ,c_tst ,c_pwm ,2 );
	
	tst_data_s.vector.comp_state[CNTRL] = VALID; // Start-up complete, Switch on testing
	do_pwm_vector( tst_data_s ,c_tst ,c_pwm ,MAX_TESTS );
	do_pwm_vector( tst_data_s ,c_tst ,c_pwm ,1 );

} // gen_pwm_width_test
/*****************************************************************************/
static void gen_motor_pwm_test_data( // Generate PWM Test data for one motor
	GENERATE_PWM_TYP &tst_data_s, // Reference to structure of PWM test data
	streaming chanend c_tst, // Channel for sending test vecotrs to test checker
	chanend c_pwm 				// Channel between Client and Server
)
{
	timer chronometer; // timer


	chronometer :> tst_data_s.time;	// Get time

	if (tst_data_s.print)
	{
		acquire_lock(); // Acquire Display Mutex
		printstrln( " Start Test Generation");
		release_lock(); // Release Display Mutex
	} // if (tst_data_s.print)

	// NB These tests assume PWM_FILTER = 0

	if (tst_data_s.common.options.flags[TST_ADC])
	{
		assign_test_vector_adc( tst_data_s ,ADC_ON );
	} // if (tst_data_s.common.options.flags[TST_ADC])
	else
	{
		assign_test_vector_adc( tst_data_s ,NO_ADC );
	} // if (tst_data_s.common.options.flags[TST_ADC])

	assign_test_vector_leg( tst_data_s ,TEST_LEG); // Set which PWM-Leg to test
	assign_test_vector_phase( tst_data_s ,TEST_PHASE ); // Set PWM-phase to test

	// Do pulse-width tests ...
	if (tst_data_s.common.options.flags[TST_NARROW]) gen_pwm_width_test( tst_data_s ,c_tst ,c_pwm ,MINI );

	gen_pwm_width_test( tst_data_s ,c_tst ,c_pwm ,SMALL );

	if (tst_data_s.common.options.flags[TST_EQUAL]) gen_pwm_width_test( tst_data_s ,c_tst ,c_pwm ,EQUAL );

	gen_pwm_width_test( tst_data_s ,c_tst ,c_pwm ,LARGE );

	if (tst_data_s.common.options.flags[TST_NARROW]) gen_pwm_width_test( tst_data_s ,c_tst ,c_pwm ,MAXI );


	tst_data_s.vector.comp_state[CNTRL] = QUIT; // Signal that testing has ended for current motor
	do_pwm_vector( tst_data_s ,c_tst ,c_pwm ,1 );

} // gen_motor_pwm_test_data
/*****************************************************************************/
void gen_all_pwm_test_data( // Generate PWM Test data
	streaming chanend c_tst, // Channel for sending test vecotrs to test checker
	chanend c_pwm 				// Channel between Client and Server
)
{
	GENERATE_PWM_TYP tst_data_s; // Structure of PWM test data


	init_pwm( tst_data_s.pwm_comms ,c_pwm ,MOTOR_ID );	// Initialise PWM communication data

	init_test_data( tst_data_s ,c_tst );

	gen_motor_pwm_test_data( tst_data_s ,c_tst ,c_pwm );

	if (tst_data_s.print)
	{
		acquire_lock(); // Acquire Display Mutex
		printstrln( "Test Generation Ends " );
		release_lock(); // Release Display Mutex
	} // if (tst_data_s.print)
} // gen_all_pwm_test_data
/*****************************************************************************/
