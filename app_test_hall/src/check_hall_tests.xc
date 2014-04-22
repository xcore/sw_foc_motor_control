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

#include "check_hall_tests.h"

/*****************************************************************************/
static void init_check_data( // Initialise check data for Hall tests
	CHECK_TST_TYP &chk_data_s // Reference to structure containing test check data
)
{
	init_common_data( chk_data_s.common ); // Initialise data common to Generator and Checker

	safestrcpy( chk_data_s.padstr1 ,"                                             " );
	safestrcpy( chk_data_s.padstr2 ,"                              " );

	chk_data_s.fail_cnt = 0; // Clear count of failed tests.

	chk_data_s.disp_str[HALL_BITS-1] = 0; // Add string terminator string

	chk_data_s.print_on = VERBOSE_PRINT; // Set print mode
	chk_data_s.print_cnt = 1; // Initialise print counter
	chk_data_s.dbg = 0; // Set debug mode

} // init_check_data
/*****************************************************************************/
static void init_motor_checks( // Initialise Hall parameter structure
	CHECK_TST_TYP &chk_data_s // Reference to structure containing test check data
)
{
	int comp_cnt; // Counter for Test Vector components


	chk_data_s.err_cnt = 0; // Clear count-down counter used in error_status test
	chk_data_s.err_chk = HALL_ERR_OFF; // Initialise expected error_status test result


	chk_data_s.curr_params.hall_val = 0; // Initialise with invalid Hall phase value
	chk_data_s.curr_params.err = 0; // No Errors

	chk_data_s.prev_params = chk_data_s.curr_params; // Initialise previous parameter values

	// Clear error and test counters for current motor
	for (comp_cnt=0; comp_cnt<NUM_VECT_COMPS; comp_cnt++)
	{
		chk_data_s.motor_errs[comp_cnt] = 0;
		chk_data_s.motor_tsts[comp_cnt] = 0;
	} // for comp_cnt

} // init_motor_checks
/*****************************************************************************/
static void print_progress( // Print progress indicator
	CHECK_TST_TYP &chk_data_s // Reference to structure containing test check data
)
{
	// Check for display-wrap
	if (PRINT_WID > chk_data_s.print_cnt)
	{
		printchar('.');
		chk_data_s.print_cnt++;
	} // if (PRINT_WID > chk_data_s.print_cnt)
	else
	{
		printcharln('.');
		chk_data_s.print_cnt = 1;
	} // if (PRINT_WID > chk_data_s.print_cnt)
} // print_progress
/*****************************************************************************/
static void print_hall_parameters( // Print Hall parameters
	CHECK_TST_TYP &chk_data_s // Reference to structure containing test check data
)
{
	convert_unsigned_to_binary_string( chk_data_s.disp_str ,chk_data_s.curr_params.hall_val ,(HALL_BITS - 1) );

	acquire_lock(); // Acquire Display Mutex
	printstr( chk_data_s.padstr1 );
	printstr( "E=" );
	printint( chk_data_s.curr_params.err );
	printstr( "  H=" );
	printstr(chk_data_s.disp_str);
	printcharln(' ');
	release_lock(); // Release Display Mutex
} // print_hall_parameters
/*****************************************************************************/
static void check_hall_error_status( // Check for correct update of error status due to Hall error-bit changes
	CHECK_TST_TYP &chk_data_s // Reference to structure containing test check data
)
{
	int inp_err = chk_data_s.curr_params.err; // local copy of error_status parameter


	chk_data_s.motor_tsts[ERROR]++;

	// Check for expected value
	if (inp_err == chk_data_s.err_chk)
	{ // Found expected value (Test passed)
		chk_data_s.err_cnt = 0; // Clear Time-out
	} // if (inp_err == chk_data_s.err_chk)
	else
	{ // Not expected value
		if (0 < chk_data_s.err_cnt)
		{ // test NOT yet timed-out
			chk_data_s.err_cnt--; // Decrement 'timer'
		} // if (0 < chk_data_s.err_cnt)
		else
		{ // test timed-out (Test failed)
			chk_data_s.motor_errs[ERROR]++;

			acquire_lock(); // Acquire Display Mutex
			printcharln(' ');
			printstr( chk_data_s.padstr1 );

			switch( chk_data_s.curr_vect.comp_state[ERROR] )
			{
				case HALL_ERR_OFF:
					printstrln("HALL_ERR_OFF FAILURE");
				break; // case HALL_ERR_OFF:

				case HALL_ERR_ON: // Start error_status test
					printstrln("HALL_ERR_ON FAILURE");
				break; // case HALL_ERR_ON:

				default:
					printstrln("ERROR: Unknown Hall Error-state");
					assert(0 == 1);
				break; // default:
			} // switch( chk_data_s.curr_vect.comp_state[ERROR] )

			release_lock(); // Release Display Mutex

			chk_data_s.err_cnt = 0; //  // Clear Time-out
		} // if (0 < chk_data_s.err_cnt)
	} // else !(inp_err == chk_data_s.err_chk)

} // check_hall_error_status
/*****************************************************************************/
static void check_hall_phase_change( // Check for valid phase change
	CHECK_TST_TYP &chk_data_s // Reference to structure containing test check data
)
{
	unsigned curr_val = chk_data_s.curr_params.hall_val; // local copy of current Hall phase value
	unsigned prev_val; // local copy of previous Hall phase value
	unsigned curr_off; // array offset for current phase value
	unsigned prev_off; // array offset for previous phase value
	int fail_cnt = 0; // Clear failure counter


	chk_data_s.motor_tsts[PHASE]++;

	// Check for correct phase value
	if ((curr_val < 1) || (HALL_PER_POLE < curr_val))
	{ // Invalid new Phase value
		fail_cnt++; // Increment count of failures

		acquire_lock(); // Acquire Display Mutex
		printcharln(' ');
		printstr( chk_data_s.padstr1 );
		printstrln("Invalid Phase FAILURE");
		release_lock(); // Release Display Mutex
	} // if ((curr_val < 1) || (HALL_PER_POLE < curr_val))
	else
	{ // Valid new Phase value
		prev_val = chk_data_s.prev_params.hall_val; // local copy of previous Hall phase value

		if ((prev_val < 1) || (HALL_PER_POLE < prev_val))
		{ // Invalid new Phase value
			fail_cnt++; // Increment count of failures

			acquire_lock(); // Acquire Display Mutex
			printcharln(' ');
			printstr( chk_data_s.padstr1 );
			printstrln("Invalid Phase FAILURE");
			release_lock(); // Release Display Mutex
		} // if ((prev_val < 1) || (HALL_PER_POLE < prev_val))
		else
		{ // Valid new Phase value
			curr_off = chk_data_s.common.inverse[curr_val];	// Get array offset for current phase value
			prev_off = chk_data_s.common.inverse[prev_val];	// Get array offset for previous phase value

			// Calculate change in phase offset
			chk_data_s.off_diff = HALL_PER_POLE + curr_off - prev_off; // Force +ve change
			if (HALL_PER_POLE <= chk_data_s.off_diff) chk_data_s.off_diff -= HALL_PER_POLE; // Force into phase-offset range

			// Check for valid offset difference
			switch( chk_data_s.off_diff )
			{
				case 0: // No Change
					fail_cnt++; // Increment count of failures

					acquire_lock(); // Acquire Display Mutex
					printcharln(' ');
					printstr( chk_data_s.padstr1 );
					printstrln("No-Change FAILURE");
					release_lock(); // Release Display Mutex
				break; // case 0:

				case 1: // Positive-spin
				break; // case 1:

				case (HALL_PER_POLE - 1): // Negative-spin
				break; // case (HALL_PER_POLE - 1):

				default:
					fail_cnt++; // Increment count of failures

					acquire_lock(); // Acquire Display Mutex
					printcharln(' ');
					printstr( chk_data_s.padstr1 );
					printstrln("Phase-Jump FAILURE");
					release_lock(); // Release Display Mutex
				break; // default:
			} // switch( chk_data_s.off_diff )
		} // else !((prev_val < 1) || (HALL_PER_POLE < prev_val))
	} // else !((curr_val < 1) || (HALL_PER_POLE < curr_val))

	// Check for any phase failures
	if (0 < fail_cnt)
	{
		chk_data_s.motor_errs[PHASE]++; // Increment failure count for current motor
	} // if (0 < fail_cnt)

} // check_hall_phase_change
/*****************************************************************************/
static void check_hall_spin_direction( // Check correct update of Hall spin direction
	CHECK_TST_TYP &chk_data_s // Reference to structure containing test check data
)
{
	chk_data_s.motor_tsts[SPIN]++;

	switch( chk_data_s.curr_vect.comp_state[SPIN] )
	{
		case POSITIVE: // Positive-spin
			if (1 != chk_data_s.off_diff)
			{
				chk_data_s.motor_errs[SPIN]++;

				acquire_lock(); // Acquire Display Mutex
				printcharln(' ');
				printstr( chk_data_s.padstr1 );
				printstrln("Positive-spin FAILURE");
				release_lock(); // Release Display Mutex
			} // if (1 != chk_data_s.off_diff)
		break; // case POSITIVE:

		case NEGATIVE: // Negative-spin
			if ((HALL_PER_POLE - 1) != chk_data_s.off_diff)
			{
				chk_data_s.motor_errs[SPIN]++;

				acquire_lock(); // Acquire Display Mutex
				printcharln(' ');
				printstr( chk_data_s.padstr1 );
				printstrln("Negative-spin FAILURE");
				release_lock(); // Release Display Mutex
			} // if ((HALL_PER_POLE - 1) != chk_data_s.off_diff)
		break; // case NEGATIVE:

		default:
			acquire_lock(); // Acquire Display Mutex
			printcharln(' ');
			printstr( chk_data_s.padstr1 );
			printstrln("ERROR: Unknown Hall Spin-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( chk_data_s.curr_vect.comp_state[SPIN] )

} // check_hall_spin_direction
/*****************************************************************************/
static void check_hall_parameters( // Check all Hall parameters
	CHECK_TST_TYP &chk_data_s // Reference to structure containing test check data
)
{
	// Check if Error-status test is activated
	if (chk_data_s.common.options.flags[TST_ERROR])
	{
		check_hall_error_status( chk_data_s ); // Check Hall error status
	} // if (chk_data_s.common.options.flags[TST_ERROR])

	check_hall_phase_change( chk_data_s ); // Check Hall phase changes

	check_hall_spin_direction( chk_data_s ); // Check Hall spin direction

	// NB There are currently NO Hall speed parameters
} // check_hall_parameters
/*****************************************************************************/
static int parameter_compare( // Check if 2 sets of Hall parameters are different
	HALL_PARAM_TYP &params_a,	// Structure containing 1st set of Hall parameters
	HALL_PARAM_TYP &params_b	// Structure containing 2nd set of Hall parameters
) // return TRUE (1) if sets are different, FALSE(0) if equal
{
	if (params_a.hall_val != params_b.hall_val) return 1;	// Check Hall value

	if (params_a.err != params_b.err) return 1;	// Check error_status

	return 0; // No differences found
} // parameter_compare
/*****************************************************************************/
static void get_new_hall_client_data( // Get next set of Hall parameters
	CHECK_TST_TYP &chk_data_s, // Reference to structure containing test check data
	streaming chanend c_hall // Hall channel between Client and Server
)
{
	int do_test = 0;	// Flag set when next test required


	// Get new parameter values from Client function under test
	foc_hall_get_parameters( chk_data_s.curr_params ,c_hall );

	// Check for change in non-speed parameters
	do_test = parameter_compare( chk_data_s.curr_params ,chk_data_s.prev_params );

	// Check for parameter change
	if (do_test)
	{ // Parameters changed

		if (chk_data_s.print_on)
		{
			print_hall_parameters( chk_data_s ); // Print new Hall parameters
		} // if (chk_data_s.print_on)

		// Check if this current test vector is valid
		if (VALID == chk_data_s.curr_vect.comp_state[CNTRL])
		{
			// Check if we have at least 2 parameter sets
			if (0 < chk_data_s.prev_params.hall_val)
			{
				check_hall_parameters( chk_data_s ); // Check new Hall parameters
			} // if (0 < chk_data_s.prev_params.hall_val)
		} // if (VALID == chk_data_s.curr_vect.comp_state[CNTRL])

		chk_data_s.prev_params = chk_data_s.curr_params; // Store previous parameter values
	} // if (chk_data_s.curr_params.theta != chk_data_s.curr_params.theta)
} // get_new_hall_client_data
/*****************************************************************************/
static void process_new_test_vector( // Process new test vector
	CHECK_TST_TYP &chk_data_s // Reference to structure containing test check data
)
{
	int change = 0; // Clear flag indicating change in test vector detected


	// Check if error_status test
	if (chk_data_s.curr_vect.comp_state[ERROR] != chk_data_s.prev_vect.comp_state[ERROR])
	{ // Initialise error_status test

		// Check if test already running
		if (0 < chk_data_s.err_cnt)
		{
			acquire_lock(); // Acquire Display Mutex
			printstr( chk_data_s.padstr1 );
			printstrln("ERROR: Previous Error-status test NOT completed");
			release_lock(); // Release Display Mutex
			assert(0 == 1); // Abort
		} // if (0 < chk_data_s.err_cnt)
		else
		{ // Start new test
			chk_data_s.err_chk = chk_data_s.curr_vect.comp_state[ERROR]; // Expected error_status value
			chk_data_s.err_cnt = ERR_TIMEOUT; // Start count-down
		} // else !(0 < chk_data_s.err_tst)

		change = 1; // Set flag indicating change in test vector detected
	} // if (chk_data_s.curr_vect.comp_state[ERROR] != chk_data_s.prev_vect.comp_state[ERROR])

	// Check if test vector changed
	if (change)
	{
		chk_data_s.prev_vect = chk_data_s.curr_vect; // Update previous test-vector
	} // if (change)

	if (chk_data_s.print_on)
	{
		print_test_vector( chk_data_s.common ,chk_data_s.curr_vect ,chk_data_s.padstr1 ); // Print new test vector details
	} // if (chk_data_s.print_on)

} // process_new_test_vector
/*****************************************************************************/
static void check_motor_hall_client_data( // Display Hall results for one motor
	CHECK_TST_TYP &chk_data_s, // Reference to structure containing test check data
	streaming chanend c_tst, // Channel for receiving test vectors from test generator
	streaming chanend c_hall // Hall channel between Client and Server
)
{
	timer chronometer; // XMOS timer
	unsigned cmd; // Hall Control Command
	int do_loop = 1;   // Flag set until loop-end condition found


	chronometer :> chk_data_s.time; // Get start time
	chronometer when timerafter(chk_data_s.time + (MICRO_SEC << 1)) :> chk_data_s.time; // Wait for Test Generation to Start

	// Wait for Hall server to start
	c_hall :> cmd;
	assert(HALL_CMD_ACK == cmd); // ERROR: Hall server did NOT send acknowledge signal

	acquire_lock(); // Acquire Display Mutex
	printstr( chk_data_s.padstr1 );
	printstr("Start Checks For Motor_"); printintln( chk_data_s.common.options.flags[TST_MOTOR] );
	release_lock(); // Release Display Mutex

	c_tst :> chk_data_s.curr_vect; // Initialise test-vector structure with 1st test

	// special case: initialisation for first speed test
  chk_data_s.prev_vect = chk_data_s.curr_vect;

	if (chk_data_s.print_on)
	{
		print_test_vector( chk_data_s.common ,chk_data_s.curr_vect ,chk_data_s.padstr1 ); // Print new test vector details
	} // if (chk_data_s.print_on)

	// Loop until end-of-testing condition found (QUIT)
	while( do_loop )
	{
		select {
			// Service any change on test channel
			case c_tst :> chk_data_s.curr_vect :
				// New test vector detected.
				process_new_test_vector( chk_data_s ); // Process new test vector

				// Check if testing has ended for current motor
				if (QUIT == chk_data_s.curr_vect.comp_state[CNTRL])
				{
					// Signal Hall Server to stop
					c_hall <: HALL_CMD_LOOP_STOP;

					do_loop = 0; // Error flag signals end-of-loop
				} // if (QUIT == chk_data_s.curr_vect.comp_state[CNTRL])
			break; // c_tst

			// Pace Hall Client requests, so as NOT to overload Hall server
			case chronometer when timerafter(chk_data_s.time + HALL_PERIOD) :> chk_data_s.time :
				get_new_hall_client_data( chk_data_s ,c_hall ); // Request data from server & check

				if (0 == chk_data_s.print_on)
				{
					print_progress( chk_data_s ); // Print progress indicator
				} // if (0 == chk_data_s.print_on)
			break; // case chronometer
		} // select
	} // while( loop )

	// Loop until Hall Server acknowledges termination request
	do // while(HALL_CMD_ACK != cmd)
	{
		c_hall :> cmd;

		if (0 == chk_data_s.print_on)
		{
			print_progress( chk_data_s ); // Print progress indicator
		} // if (0 == chk_data_s.print_on)
	} while(HALL_CMD_ACK != cmd);

} // check_motor_hall_client_data
/*****************************************************************************/
static void display_test_results( // Display test results for one motor
	CHECK_TST_TYP &chk_data_s // Reference to structure containing test check data
)
{
	int comp_cnt; // Counter for Test Vector components
	int check_errs = 0;   // Preset flag to NO check errors for current motor
	int num_checks = 0;   // Clear check counter for current motor
	int test_errs = 0;   // Preset flag to NO test errors for current motor
	int num_tests = 0;   // Clear test counter for current motor


	// Update error statistics for current motor
	for (comp_cnt=1; comp_cnt<NUM_VECT_COMPS; comp_cnt++)
	{
		// Check if any micro-tests where done for current test vector component
		if (0 < chk_data_s.motor_tsts[comp_cnt])
		{
			num_tests++; // Update macro-test counter
			num_checks += chk_data_s.motor_tsts[comp_cnt];

			// Check if any micro-errors where detected for current test vector component
			if (0 < chk_data_s.motor_errs[comp_cnt])
			{
				test_errs++; // Update macro-error counter
				check_errs += chk_data_s.motor_errs[comp_cnt];
			} // if (0 < chk_data_s.motor_errs[comp_cnt])
		} // if (0 < chk_data_s.motor_tsts[comp_cnt])
	} // for comp_cnt

	acquire_lock(); // Acquire Display Mutex
	printstrln("");
	printstr( chk_data_s.padstr1 );
	printint( num_tests );
	printstr( " Tests run" );

	// Check for verbose test output
	if (1 == MICRO_TESTS)
	{
		printstr(" (Comprising ");
		printint( num_checks );
		printstr( " checks)" );
	} // if (1 == MICRO_TESTS)

	printstrln("");

	// Check if this motor had any errors
	if (test_errs)
	{
		printstr( chk_data_s.padstr1 );
		printint( test_errs );
		printstrln( " Tests FAILED, as follows:" );

		// Print Vector Component Names
		for (comp_cnt=1; comp_cnt<NUM_VECT_COMPS; comp_cnt++)
		{
			// Check if any test run for this component
			if (chk_data_s.motor_tsts[comp_cnt])
			{
				printstr( chk_data_s.padstr1 );
				printstr( chk_data_s.common.comp_data[comp_cnt].comp_name.str );

				if (chk_data_s.motor_errs[comp_cnt])
				{
					printstr(" Test FAILED");

					// Check for verbose test output
					if (1 == MICRO_TESTS)
					{
						printstr(" (");
						printint( chk_data_s.motor_errs[comp_cnt] );
						printstr( " out of " );
						printint( chk_data_s.motor_tsts[comp_cnt] );
						printstr( " checks failed)" );
					} // if (1 == MICRO_TESTS)
				} // if (chk_data_s.motor_errs[comp_cnt])
				else
				{
					printstr(" Test Passed");

					// Check for verbose test output
					if (1 == MICRO_TESTS)
					{
						printstr(" (");
						printint( chk_data_s.motor_tsts[comp_cnt] );
						printstr( " checks run)" );
					} // if (1 == MICRO_TESTS)
				} // if (chk_data_s.motor_errs[comp_cnt])

				printstrln("");

			} // if (chk_data_s.motor_tsts[comp_cnt])
		} // for comp_cnt
	} // if (check_errs)
	else
	{
		printstr( chk_data_s.padstr1 );
		printstr( "All Motor_" );
		printint( chk_data_s.common.options.flags[TST_MOTOR] );
 		printstrln( " Tests Passed" );
	} // else !(check_errs)

	printstrln("");
	release_lock(); // Release Display Mutex

} // display_test_results
/*****************************************************************************/
void check_all_hall_client_data( // Display Hall results for all motors
	streaming chanend c_tst, // Channel for receiving test vectors from test generator
	streaming chanend c_hall[] // Array of Hall channel between Client and Server
)
{
	CHECK_TST_TYP chk_data_s; // Structure containing test check data


	init_check_data( chk_data_s ); // Initialise check data

	c_tst :> chk_data_s.common.options; // Get test options from generator core

	init_motor_checks( chk_data_s ); // Initialise Hall parameter structure

	check_motor_hall_client_data( chk_data_s ,c_tst ,c_hall[chk_data_s.common.options.flags[TST_MOTOR]] );

	display_test_results( chk_data_s );

	acquire_lock(); // Acquire Display Mutex
	printstr( chk_data_s.padstr1 );
	printstrln( "Test Check Ends " );
	release_lock(); // Release Display Mutex

	_Exit(0); // Exit without house-keeping
} // check_all_hall_client_data
/*****************************************************************************/
