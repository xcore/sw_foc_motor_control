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

#include "check_pwm_tests.h"

/*****************************************************************************/
static void init_check_data( // Initialise check data for PWM tests
	CHECK_PWM_TYP &chk_data_s // Reference to structure containing test check data
)
{
	int comp_cnt; // Counter for Test Vector components


	init_common_data( chk_data_s.common ); // Initialise data common to Generator and Checker

	safestrcpy( chk_data_s.padstr1 ,"                                             " );
	safestrcpy( chk_data_s.padstr2 ,"                              " );

	chk_data_s.fail_cnt = 0; // Clear count of failed tests.

	chk_data_s.print = PRINT_TST_PWM; // Set print mode
	chk_data_s.dbg = 0; // Set debug mode

	chk_data_s.curr_val = 0;
	chk_data_s.prev_val = chk_data_s.curr_val; // Initialise previous PWM value

	// Clear error and test counters for current motor
	for (comp_cnt=0; comp_cnt<NUM_VECT_COMPS; comp_cnt++)
	{
		chk_data_s.motor_errs[comp_cnt] = 0; 
		chk_data_s.motor_tsts[comp_cnt] = 0; 
	} // for comp_cnt

} // init_motor_checks
/*****************************************************************************/
static void print_pwm_parameters( // Print PWM parameters
	CHECK_PWM_TYP &chk_data_s // Reference to structure containing test check data
)
{
	acquire_lock(); // Acquire Display Mutex
	printstr( chk_data_s.padstr1 );

	printstr( "  P=" );
	printint( chk_data_s.curr_vect.comp_state[PHASE] );
	printstr( "  I=" );
	printint( chk_data_s.curr_val );
	printcharln(' ');
	release_lock(); // Release Display Mutex
} // print_pwm_parameters
/*****************************************************************************/
static void check_pwm_width( // Check for correct PWM width
	CHECK_PWM_TYP &chk_data_s // Reference to structure containing test check data
)
{
	int inp_val = chk_data_s.curr_val; // local copy of PWM value


	chk_data_s.motor_tsts[WIDTH]++;

	// Check for expected value
	if (inp_val == chk_data_s.wid_chk)
	{ // Found expected value (Test passed)
		chk_data_s.wid_cnt = 0; // Clear Time-out
	} // if (inp_val == chk_data_s.wid_chk)
	else
	{ // Not expected value
		if (0 < chk_data_s.wid_cnt)
		{ // test NOT yet timed-out
			chk_data_s.wid_cnt--; // Decrement 'timer'
		} // if (0 < chk_data_s.wid_cnt)
		else
		{ // test timed-out (Test failed)
			chk_data_s.motor_errs[WIDTH]++;

			acquire_lock(); // Acquire Display Mutex
			printcharln(' ');
			printstr( chk_data_s.padstr1 );

			switch( chk_data_s.curr_vect.comp_state[WIDTH] )
			{
				case FAST:
					printstrln("FAST FAILURE");
				break; // case FAST:
		
				case SLOW: // Start error_status test
					printstrln("SLOW FAILURE");
				break; // case SLOW:
		
				default:
					printstrln("WIDTH: Unknown PWM Width-state");
					assert(0 == 1);
				break; // default:
			} // switch( chk_data_s.curr_vect.comp_state[WIDTH] )

			release_lock(); // Release Display Mutex

			chk_data_s.wid_cnt = 0; //  // Clear Time-out
		} // if (0 < chk_data_s.wid_cnt)
	} // else !(inp_val == chk_data_s.wid_chk)

} // check_pwm_width
/*****************************************************************************/
static void check_pwm_parameters( // Check all PWM parameters
	CHECK_PWM_TYP &chk_data_s // Reference to structure containing test check data
)
{
	check_pwm_width( chk_data_s ); // Check PWM width
} // check_pwm_parameters
/*****************************************************************************/
static void test_new_pwm_input_value( // test new PWM input value
	CHECK_PWM_TYP &chk_data_s // Reference to structure containing test check data
)
{
	// Check for parameter change
	if (chk_data_s.curr_val != chk_data_s.prev_val)
	{ // Parameters changed

		if (chk_data_s.print)
		{
			print_pwm_parameters( chk_data_s ); // Print new PWM parameters
		} // if (chk_data_s.print)

		// Check if this current test vector is valid
		if (VALID == chk_data_s.curr_vect.comp_state[CNTRL])
		{
			check_pwm_parameters( chk_data_s ); // Check all PWM parameters
		} // if (VALID == chk_data_s.curr_vect.comp_state[CNTRL])

		chk_data_s.prev_val = chk_data_s.curr_val; // Store previous PWM value
	} // if (chk_data_s.curr_val != chk_data_s.prev_val)
} // test_new_pwm_input_value
/*****************************************************************************/
static void process_new_test_vector( // Process new test vector
	CHECK_PWM_TYP &chk_data_s // Reference to structure containing test check data
)
{
	int change = 0; // Clear flag indicating change in test vector detected


	// Check if error_status test
	if (chk_data_s.curr_vect.comp_state[WIDTH] != chk_data_s.prev_vect.comp_state[WIDTH])
	{ // Initialise error_status test

		// Check if test already running
		if (0 < chk_data_s.wid_cnt)
		{
			acquire_lock(); // Acquire Display Mutex
			printstr( chk_data_s.padstr1 );
			printstrln("WIDTH: Previous PWM-width test NOT completed");
			release_lock(); // Release Display Mutex
			assert(0 == 1); // Abort
		} // if (0 < chk_data_s.wid_cnt)
		else
		{ // Start new test
			chk_data_s.wid_chk = chk_data_s.curr_vect.comp_state[WIDTH]; // Expected error_status value
			chk_data_s.wid_cnt = WID_TIMEOUT; // Start count-down
		} // else !(0 < chk_data_s.wid_cnt)

		change = 1; // Set flag indicating change in test vector detected
	} // if (chk_data_s.curr_vect.comp_state[WIDTH] != chk_data_s.prev_vect.comp_state[WIDTH])

	// Check if test vector changed
	if (change)
	{
		chk_data_s.prev_vect = chk_data_s.curr_vect; // Update previous test-vector
	} // if (change)

	if (chk_data_s.print)
	{
		print_test_vector( chk_data_s.common ,chk_data_s.curr_vect ,chk_data_s.padstr1 ); // Print new test vector details
	} // if (chk_data_s.print)

} // process_new_test_vector
/*****************************************************************************/
void check_all_pwm_client_data( // Display PWM results for all motors
	streaming chanend c_tst, // Channel for receiving test vectors from test generator
	buffered in port:32 p32_tst_hi[], // array of PWM ports (High side)  
	buffered in port:32 p32_tst_lo[], // array of PWM ports (Low side)   
	chanend c_adc_trig // ADC trigger channel 
)
{
	CHECK_PWM_TYP chk_data_s; // Structure containing test check data
	unsigned inp_bufs[NUM_PWM_LEGS][NUM_INP_BUFS]; // Set of buffers for PWM widths for each PWM-leg
	timer chronometer; // XMOS timer
	int write_off[NUM_PWM_LEGS]; // buffer write offset for each PWM-leg 
	int read_off[NUM_PWM_LEGS]; // buffer write offset for each PWM-leg 
	int diff_off = 0; // Difference between Read and Write offsets
	int leg_id = 0; // PWM-leg identifier
	int comp_cnt; // Counter for Test Vector components
	int do_loop = 1;   // Flag set until loop-end condition found 
	int motor_errs = 0;   // Preset flag to NO errors for current motor
	int motor_tsts = 0;   // Clear test ccounter for current motor
	unsigned char cntrl_token; // control token


	init_check_data( chk_data_s ); // Initialise check data


	chronometer :> chk_data_s.time[0]; // Get start time
	chronometer when timerafter(chk_data_s.time[0] + (MICRO_SEC << 1)) :> chk_data_s.time[0]; // Wait for Test Generation to Start

	// Initialise port buffer offsets
	for (leg_id=0; leg_id<NUM_PWM_LEGS; leg_id++)
	{
		write_off[leg_id] = 0;
		read_off[leg_id] = 0;
		chk_data_s.time[leg_id] = chk_data_s.time[0] ;
	} // for leg_id

	leg_id = HI_LEG; // Do high-leg first
 
	acquire_lock(); // Acquire Display Mutex
	printcharln(' ');
	printstr( chk_data_s.padstr1 );
	printstrln("Start Checks"); 
	release_lock(); // Release Display Mutex

	c_tst :> chk_data_s.curr_vect; // Initialise test-vector structure with 1st test

	// special case: initialisation for first speed test
  chk_data_s.prev_vect = chk_data_s.curr_vect;

	if (chk_data_s.print)
	{
		print_test_vector( chk_data_s.common ,chk_data_s.curr_vect ,chk_data_s.padstr1 ); // Print new test vector details
	} // if (chk_data_s.print)

	while (1) {
#pragma ordered // If multiple cases fire at same time, service top-most first
		select {
			// Service any change on High-leg input port pins
			case p32_tst_hi[chk_data_s.curr_vect.comp_state[PHASE]] when pinsneq(inp_bufs[HI_LEG][write_off[HI_LEG]]):> inp_bufs[HI_LEG][write_off[HI_LEG]] :
			{
				chronometer :> chk_data_s.time[HI_LEG]; // Get new time stamp as soon as possible

				// Update circular buffer offset
				if (write_off[HI_LEG] < (NUM_INP_BUFS - 1))
				{
					write_off[HI_LEG]++;
				} // if (write_off[HI_LEG] < (NUM_INP_BUFS - 1))
				else
				{
					write_off[HI_LEG] = 0;
				} // else !(write_off[HI_LEG] < (NUM_INP_BUFS - 1))
			} // case
			break;

#ifdef MB
			// Service any change on Low-leg input port pins
#endif //MB~

			case inct_byref( c_adc_trig, cntrl_token ):
// ToDo MB~
			break;
	
			// Service any change on test channel
			case c_tst :> chk_data_s.curr_vect :
				// New test vector detected.
				process_new_test_vector( chk_data_s ); // Process new test vector

				// Check if testing has ended for current motor
				if (QUIT == chk_data_s.curr_vect.comp_state[CNTRL])
				{
					do_loop = 0; // Error flag signals end-of-loop
				} // if (QUIT == chk_data_s.curr_vect.comp_state[CNTRL])
			break; // c_tst 

			default:
				diff_off = write_off[leg_id] - read_off[leg_id]; // Calculate read/write offset for current leg

				// Check if any new data to read
				if (0 != diff_off)
				{
					chk_data_s.curr_val = inp_bufs[leg_id][ read_off[leg_id] ]; 

					// Update circular buffer offset
					if (read_off[leg_id] < (NUM_INP_BUFS - 1))
					{
						read_off[leg_id]++;
					} // if (read_off[leg_id] < (NUM_INP_BUFS - 1))
					else
					{
						read_off[leg_id] = 0;
					} // else !(read_off[leg_id] < (NUM_INP_BUFS - 1))

					leg_id = 1 - leg_id; // Switch to other leg

					test_new_pwm_input_value( chk_data_s ); // test new PWM data
				} // if (0 != diff_off)
			break; // default
		} // select
	}	// while (1)

	// Update error statistics for current motor
	for (comp_cnt=1; comp_cnt<NUM_VECT_COMPS; comp_cnt++)
	{
		motor_errs += chk_data_s.motor_errs[comp_cnt]; 
		motor_tsts += chk_data_s.motor_tsts[comp_cnt]; 
	} // for comp_cnt

	acquire_lock(); // Acquire Display Mutex
	printcharln(' ');
	printstr( chk_data_s.padstr1 );
	printint( motor_tsts );
	printstrln( " tests run" );

	// Check if this motor had any errors
	if (motor_errs)
	{
		printstr( chk_data_s.padstr1 );
		printint( motor_errs );
		printstrln( " tests FAILED, as follows:" );

		// Print Vector Component Names
		for (comp_cnt=1; comp_cnt<NUM_VECT_COMPS; comp_cnt++)
		{
			// Check if any test run for this component
			if (chk_data_s.motor_tsts[comp_cnt])
			{
				printstr( chk_data_s.padstr1 );
				printstr( chk_data_s.common.comp_data[comp_cnt].comp_name.str );
				printstr(" : ");
				printint( chk_data_s.motor_tsts[comp_cnt] );
				printstr( " tests run" );
	
				if (chk_data_s.motor_errs[comp_cnt])
				{
					printstr( ", " );
					printint( chk_data_s.motor_errs[comp_cnt] );
					printstr(" FAILURES");
				} // if (chk_data_s.motor_errs[comp_cnt])
				printcharln(' ');
			} // if (chk_data_s.motor_tsts[comp_cnt])
		} // for comp_cnt
	} // if (motor_errs)
	else
	{
		printstrln( "ALL TESTS PASSED" );
	} // else !(motor_errs)

	printcharln( ' ' );
	release_lock(); // Release Display Mutex
} // check_all_pwm_client_data
/*****************************************************************************/
