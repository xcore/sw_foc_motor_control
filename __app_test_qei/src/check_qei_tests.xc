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

#include "check_qei_tests.h"

/*****************************************************************************/
static unsigned eval_speed_bound( // Evaluate error bounds for given speed
	unsigned veloc_val // input velocity
) // Returns error_bound
// Requirements documents suggest +/- 10% error. We will do better than that and target 6.25%
{
	unsigned bound; // Output error_bound


	bound = veloc_val >> SPEED_ERR_BITS; // Calculate speed bound

	return bound;
}	// eval_speed_bound
/*****************************************************************************/
static void init_check_data( // Initialise check data for QEI tests
	const COMMON_TST_TYP &comm_data_s, // Reference to structure containing common test data
	CHECK_TST_TYP &chk_data_s // Reference to structure containing test check data
)
{
	int tmp_val;	// temporary manipulation variable


	/* Calculate error bounds for angular position after reset:
		 In the worst case, the angular position may not be measured until
		 a whole QEI_PERIOD after the reset. The amount the position changes during this time
     depends on the speed. We need to calculate the following

		 ang_mar = (QEI_PERIOD / PLATFORM_REFERENCE_HZ) * (SPEED_IN_RPM / 60) * QEI_PER_REV

		 The maximum error margin occurs when using the maximum speed

		 This has to be done without overflow, and retaining as much precision as possible.
     So it is done in the following order ...
	*/
	tmp_val = QEI_PERIOD * HIGH_SPEED; // ~24 bits
	tmp_val = (tmp_val + (SECS_PER_MIN - 1)) / SECS_PER_MIN; // division with round-up // ~19 bits
	tmp_val *= QEI_PER_REV; // ~29 bits
	chk_data_s.ang_bound = (tmp_val + (PLATFORM_REFERENCE_HZ - 1)) / PLATFORM_REFERENCE_HZ; // division with round-up ~2 bits

	// Evaluate error bounds for speed checks
	chk_data_s.hi_bound = 1 + eval_speed_bound( HIGH_SPEED );
	chk_data_s.lo_bound = 1 + eval_speed_bound( LOW_SPEED );

	chk_data_s.prefix = comm_data_s.prefix[DISP_INP_CHK]; // local copy of display prefix

	chk_data_s.fail_cnt = 0; // Clear count of failed tests.

	chk_data_s.print_on = VERBOSE_PRINT; // Set print mode
	chk_data_s.dbg = 0; // Set debug mode

} // init_check_data
/*****************************************************************************/
static void init_motor_checks( // Initialise QEI parameter structure
	CHECK_TST_TYP &chk_data_s // Reference to structure containing test check data
)
{
	int comp_cnt; // Counter for Test Vector components


	chk_data_s.err_cnt = 0; // Clear count-down counter used in error_status test
	chk_data_s.err_chk = QEI_ERR_OFF; // Initialise expected error_status test result

	chk_data_s.orig_cnt = 0; // Clear count-down counter used in origin-bit test
	chk_data_s.orig_chk = ORIG_OFF; // Initialise expected origin-bit test result

	chk_data_s.curr_params.rev_cnt = 0;
	chk_data_s.curr_params.theta = 0;
	chk_data_s.curr_params.veloc = 0;
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
static void check_qei_position_reset( // Check for correct position reset after origin detection
	CHECK_TST_TYP &chk_data_s // Reference to structure containing test check data
)
{
	int ang_err;	// angular_position error


	chk_data_s.motor_tsts[ORIGIN]++;

	// Calculate angular position error ...

	ang_err = abs(chk_data_s.curr_params.theta); // Initialise to error about zero

	// Check if error too large
	if (HALF_QEI_POS < ang_err)
	{
		ang_err = QEI_PER_REV - ang_err; // Correct error by one revolution
	} // if (HALF_QEI_POS < ang_err)

	// Check for out-of-bounds angular position
	if (chk_data_s.ang_bound < ang_err)
	{
		chk_data_s.motor_errs[ORIGIN]++;

		acquire_lock(); // Acquire Display Mutex
		printcharln(' ');
		printstr( chk_data_s.prefix.str );
		printstrln("ORIG_ANG_POS FAILURE");
		release_lock(); // Release Display Mutex
	} // if (chk_data_s.ang_bound < ang_err)

} // check_qei_position_reset
/*****************************************************************************/
static void check_qei_origin_detection( // Check correct update of QEI parameters due to origin detection
	CHECK_TST_TYP &chk_data_s // Reference to structure containing test check data
)
{
	int rev_diff = chk_data_s.curr_params.rev_cnt - chk_data_s.prev_params.rev_cnt; // Change in rev. counter


	// Check for expected value
	if (rev_diff == chk_data_s.orig_chk)
	{ // Found expected value (Test passed)
		chk_data_s.motor_tsts[ORIGIN]++;

		// Check if position reset expected
		if (0 != chk_data_s.orig_chk)
		{
			check_qei_position_reset( chk_data_s ); // Check angular position has been correctly reset
		} // if (0 != chk_data_s.orig_chk)

		chk_data_s.orig_cnt = 0; // Clear Time-out
		chk_data_s.orig_chk = 0; // Set new expected origin-bit test result
	} // if (rev_diff == chk_data_s.orig_chk)
	else
	{ // Not expected value

		if (0 < chk_data_s.orig_cnt)
		{ // test NOT yet timed-out
			chk_data_s.orig_cnt--; // Decrement 'timer'
		} // if (0 < chk_data_s.orig_cnt)
		else
		{ // test timed-out (Test failed)
			chk_data_s.motor_tsts[ORIGIN]++;
			chk_data_s.motor_errs[ORIGIN]++;

			acquire_lock(); // Acquire Display Mutex
			printcharln(' ');
			printstr( chk_data_s.prefix.str );

			switch( chk_data_s.curr_vect.comp_state[ORIGIN] )
			{
				case ORIG_OFF:
					printstrln("ORIG_OFF FAILURE");
				break; // case ORIG_OFF:

				case ORIG_ON: // Start origin-bit test
					printstrln("ORIG_ON FAILURE");
				break; // case ORIG_ON:

				default:
					printstrln("ERROR: Unknown QEI Origin-state");
					assert(0 == 1);
				break; // default:
			} // switch( chk_data_s.curr_vect.comp_state[ORIGIN] )

			release_lock(); // Release Display Mutex

			chk_data_s.orig_cnt = 0; // Clear Time-out
			chk_data_s.orig_chk = 0; // Set new expected origin-bit test result
		} // if (0 < chk_data_s.orig_cnt)
	} // else !(rev_diff == chk_data_s.orig_chk)

} // check_qei_origin_detection
/*****************************************************************************/
static void check_qei_error_status( // Check for correct update of error status due to QEI error-bit changes
	CHECK_TST_TYP &chk_data_s // Reference to structure containing test check data
)
{
	int inp_err = chk_data_s.curr_params.err; // local copy of error_status parameter



	// Check for expected value
	if (inp_err == chk_data_s.err_chk)
	{ // Found expected value (Test passed)
		chk_data_s.motor_tsts[ERROR]++;

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
			chk_data_s.motor_tsts[ERROR]++;
			chk_data_s.motor_errs[ERROR]++;

			acquire_lock(); // Acquire Display Mutex
			printcharln(' ');
			printstr( chk_data_s.prefix.str );

			switch( chk_data_s.curr_vect.comp_state[ERROR] )
			{
				case QEI_ERR_OFF:
					printstrln("ERR_OFF FAILURE");
				break; // case QEI_ERR_OFF:

				case QEI_ERR_ON: // Start error_status test
					printstrln("ERR_ON FAILURE");
				break; // case QEI_ERR_ON:

				default:
					printstrln("ERROR: Unexpected QEI Error-state");
					assert(0 == 1);
				break; // default:
			} // switch( chk_data_s.curr_vect.comp_state[ERROR] )

			release_lock(); // Release Display Mutex

			chk_data_s.err_cnt = 0; //  // Clear Time-out
		} // if (0 < chk_data_s.err_cnt)
	} // else !(inp_err == chk_data_s.err_chk)

} // check_qei_error_status
/*****************************************************************************/
static void check_qei_spin_direction( // Check correct update of QEI spin direction
	CHECK_TST_TYP &chk_data_s // Reference to structure containing test check data
)
{
	int inp_vel = chk_data_s.curr_params.veloc; // local copy of angular_velocity parameter


	switch( chk_data_s.curr_vect.comp_state[SPIN] )
	{
		case POSITIVE: // Positive-spin
			chk_data_s.motor_tsts[SPIN]++;
			if (0 > inp_vel)
			{
				chk_data_s.motor_errs[SPIN]++;

				acquire_lock(); // Acquire Display Mutex
				printcharln(' ');
				printstr( chk_data_s.prefix.str );
				printstrln("Positive-spin FAILURE");
				release_lock(); // Release Display Mutex
			} // if (0 > inp_vel)
		break; // case POSITIVE:

		case NEGATIVE: // Negative-spin
			chk_data_s.motor_tsts[SPIN]++;
			if (0 < inp_vel)
			{
				chk_data_s.motor_errs[SPIN]++;

				acquire_lock(); // Acquire Display Mutex
				printcharln(' ');
				printstr( chk_data_s.prefix.str );
				printstrln("Negative-spin FAILURE");
				release_lock(); // Release Display Mutex
			} // if (0 < inp_vel)
		break; // case NEGATIVE:

		default:
			acquire_lock(); // Acquire Display Mutex
			printcharln(' ');
			printstr( chk_data_s.prefix.str );
			printstrln("ERROR: Unknown QEI Spin-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( chk_data_s.curr_vect.comp_state[SPIN] )

} // check_qei_spin_direction
/*****************************************************************************/
static void update_qei_angular_speed( // Update accumulators for calculating QEI speed
	CHECK_TST_TYP &chk_data_s // Reference to structure containing test check data
)
{
	int curr_speed = abs( chk_data_s.curr_params.veloc ); // current angular speed
	int prev_speed = abs( chk_data_s.prev_params.veloc ); // previous angular speed


	switch( chk_data_s.curr_vect.comp_state[SPEED] )
	{
 		case ACCEL: case DECEL: // Speed change
			chk_data_s.speed_sum += (curr_speed - prev_speed); // Accumulate speed-change
		break; // case ACCEL: DECEL:

		case FAST: case SLOW: // Constant Speed
			chk_data_s.speed_sum += curr_speed; // Accumulate constant-speed
		break; // case FAST: SLOW:

		default:
			acquire_lock(); // Acquire Display Mutex
			printcharln(' ');
			printstr( chk_data_s.prefix.str );
			printstrln("ERROR: Unknown QEI Speed-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( chk_data_s.curr_vect.comp_state[SPEED] )

	chk_data_s.speed_num++; // Update No. of samples in accumulator
} // update_qei_angular_speed
/*****************************************************************************/
static void check_qei_angular_speed( // Check all QEI speed as motor accelerates/decelerates
	CHECK_TST_TYP &chk_data_s, // Reference to structure containing test check data
	const SPEED_QEI_ENUM cur_speed	// Speed-state to check
)
{
	int mean; // Mean speed parameter under-test


	assert(0 < chk_data_s.speed_num); // ERROR: No test data collected. Unexpected behaviour

	// Calculate mean from accumulated data (with rounding to -infinity)
	if (0 > chk_data_s.speed_sum)
	{  // -ve Rounding
		mean = (chk_data_s.speed_sum - (chk_data_s.speed_num >> 1)) / chk_data_s.speed_num;
	} // if (0 > chk_data_s.speed_sum)
	else
	{  // +ve Rounding
		mean = (chk_data_s.speed_sum + (chk_data_s.speed_num >> 1)) / chk_data_s.speed_num;
	} // else !(0 > chk_data_s.speed_sum)

	switch( cur_speed )
	{
		case ACCEL: // Accelerate
			chk_data_s.motor_tsts[SPEED]++;

			if (0 >= mean)
			{
				chk_data_s.motor_errs[SPEED]++;

				acquire_lock(); // Acquire Display Mutex
				printcharln(' ');
				printstr( chk_data_s.prefix.str );
				printstrln("ACCEL FAILURE");
				release_lock(); // Release Display Mutex
			} // if (0 > mean)
		break; // case ACCEL:

		case FAST: // Constant Fast Speed
			chk_data_s.motor_tsts[SPEED]++;

			if (chk_data_s.hi_bound < abs(mean - HIGH_SPEED))
			{
				chk_data_s.motor_errs[SPEED]++;

				acquire_lock(); // Acquire Display Mutex
				printcharln(' ');
				printstr( chk_data_s.prefix.str );
				printstrln("FAST FAILURE");
				release_lock(); // Release Display Mutex
			} // if (chk_data_s.hi_bound < abs(mean - HIGH_SPEED))
		break; // case FAST:

		case DECEL: // Accelerate
			chk_data_s.motor_tsts[SPEED]++;

			if (0 <= mean)
			{
				chk_data_s.motor_errs[SPEED]++;

				acquire_lock(); // Acquire Display Mutex
				printcharln(' ');
				printstr( chk_data_s.prefix.str );
				printstrln("DECEL FAILURE");
				release_lock(); // Release Display Mutex
			} // if (0 < mean)
		break; // case DECEL:

		case SLOW: // Constant Slow Speed
			chk_data_s.motor_tsts[SPEED]++;

			if (chk_data_s.lo_bound < abs(mean - LOW_SPEED))
			{
				chk_data_s.motor_errs[SPEED]++;

				acquire_lock(); // Acquire Display Mutex
				printcharln(' ');
				printstr( chk_data_s.prefix.str );
				printstrln("SLOW FAILURE");
				release_lock(); // Release Display Mutex
			} // if (chk_data_s.hi_bound < abs(mean - HIGH_SPEED))
		break; // case SLOW:

		default:
			acquire_lock(); // Acquire Display Mutex
			printcharln(' ');
			printstr( chk_data_s.prefix.str );
			printstrln("ERROR: Unknown QEI Speed-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( cur_speedspeed )

} // check_qei_angular_speed
/*****************************************************************************/
static void check_qei_parameters( // Check all QEI parameters
	CHECK_TST_TYP &chk_data_s // Reference to structure containing test check data
)
{
	// Check if Error-status test is activated
	if (chk_data_s.options.flags[TST_ERROR])
	{
		check_qei_error_status( chk_data_s ); // Check QEI error status
	} // if (chk_data_s.options.flags[TST_ERROR])

	check_qei_origin_detection( chk_data_s ); // Check QEI origin detection

	check_qei_spin_direction( chk_data_s ); // Check QEI spin direction

	update_qei_angular_speed( chk_data_s ); // Update data on QEI angular speed
} // check_qei_parameters
/*****************************************************************************/
static int parameter_compare( // Check if 2 sets of QEI parameters are different
	QEI_PARAM_TYP &params_a,	// Structure containing 1st set of QEI parameters
	QEI_PARAM_TYP &params_b	// Structure containing 2nd set of QEI parameters
) // return TRUE (1) if sets are different, FALSE(0) if equal
{
	if (params_a.theta != params_b.theta) return 1;	// Check angular_position

	if (params_a.rev_cnt != params_b.rev_cnt) return 1;	// Check rev. counter

	if (params_a.err != params_b.err) return 1;	// Check error_status

	return 0; // No differences found
} // parameter_compare
/*****************************************************************************/
static void get_new_qei_client_data( // Get next set of QEI parameters
	CHECK_TST_TYP &chk_data_s, // Reference to structure containing test check data
	streaming chanend c_disp, // Channel for sending display data to print scheduler core
	streaming chanend c_qei // QEI channel between Client and Server
)
{
	int do_test = 0;	// Flag set when next test required


	// Get new parameter values from Client function under test
	foc_qei_get_parameters( chk_data_s.curr_params ,c_qei );

#if (USE_XSCOPE)
		xscope_int( 0 ,chk_data_s.curr_params.rev_cnt );
		xscope_int( 1 ,chk_data_s.curr_params.theta );
		xscope_int( 2 ,chk_data_s.curr_params.veloc );
		xscope_int( 3 ,chk_data_s.curr_params.err );
#endif // (USE_XSCOPE)

	// Check for change in non-speed parameters
	do_test = parameter_compare( chk_data_s.curr_params ,chk_data_s.prev_params );

	// Check for parameter change
	if (do_test)
	{ // Parameters changed

		if (chk_data_s.print_on)
		{
			c_disp <: DISP_CLASS_CHECK; // Signal tranmission of parameter data
			c_disp <: chk_data_s.curr_params; // Send parameter data
		} // if (chk_data_s.print_on)

		// Check if this current test vector is valid
		if (VALID == chk_data_s.curr_vect.comp_state[CNTRL])
		{
			check_qei_parameters( chk_data_s ); // Check new QEI parameters
		} // if (VALID == chk_data_s.curr_vect.comp_state[CNTRL])

		chk_data_s.prev_params = chk_data_s.curr_params; // Store previous parameter values
	} // if (chk_data_s.curr_params.theta != chk_data_s.curr_params.theta)
} // get_new_qei_client_data
/*****************************************************************************/
static void initialise_speed_test_vector( // Initialise data for new speed test
	CHECK_TST_TYP &chk_data_s // Reference to structure containing test check data
)
{
	// Clear accumulated data
	chk_data_s.speed_sum = 0;
	chk_data_s.speed_num = 0;
} // initialise_speed_test_vector
/*****************************************************************************/
static void finalise_speed_test_vector( // terminate speed test and check results
	CHECK_TST_TYP &chk_data_s, // Reference to structure containing test check data
	const SPEED_QEI_ENUM cur_speed	// Speed-state to finalise
)
{
	check_qei_angular_speed( chk_data_s ,cur_speed ); // Check QEI angular speed
} // finalise_speed_test_vector
/*****************************************************************************/
static void process_new_test_vector( // Process new test vector
	CHECK_TST_TYP &chk_data_s, // Reference to structure containing test check data
	streaming chanend c_disp // Channel for sending display data to print scheduler core
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
			printstr( chk_data_s.prefix.str );
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

	// Check if origin-bit test
	if (ORIG_ON == chk_data_s.curr_vect.comp_state[ORIGIN])
	{ // Initialise origin test

		// Check if test already running
		if (0 < chk_data_s.orig_cnt)
		{
			acquire_lock(); // Acquire Display Mutex
			printstr( chk_data_s.prefix.str );
			printstrln("ERROR: Previous Origin test NOT completed");
			release_lock(); // Release Display Mutex
			assert( 0 == 1); // Abort
		} // if (0 < chk_data_s.orig_cnt)
		else
		{ // Start new test
			switch( chk_data_s.curr_vect.comp_state[SPIN] )
			{
				case POSITIVE: // Positive-spin
					chk_data_s.orig_chk = 1; // Expected increment in rev. counter
				break; // case POSITIVE:

				case NEGATIVE: // Negative-spin
					chk_data_s.orig_chk = -1; // Expected decrement in rev. counter
				break; // case NEGATIVE:

				default:
					acquire_lock(); // Acquire Display Mutex
					printcharln(' ');
					printstr( chk_data_s.prefix.str );
					printstrln("ERROR: Unknown QEI Spin-state");
					release_lock(); // Release Display Mutex
					assert(0 == 1);
				break; // default:
			} // switch( chk_data_s.curr_vect.comp_state[SPIN] )

			chk_data_s.orig_cnt = ORIG_TIMEOUT; // Start count-down
		} // else !(0 < chk_data_s.orig_cnt)

		change = 1; // Set flag indicating change in test vector detected
	} // if (ORIG_ON == chk_data_s.curr_vect.comp_state[ORIGIN])

	// Check for change in speed test
	if ((chk_data_s.curr_vect.comp_state[SPEED] != chk_data_s.prev_vect.comp_state[SPEED])
		|| (chk_data_s.curr_vect.comp_state[SPIN] != chk_data_s.prev_vect.comp_state[SPIN]))
	{
		finalise_speed_test_vector( chk_data_s ,chk_data_s.prev_vect.comp_state[SPEED] );

		initialise_speed_test_vector( chk_data_s );

		change = 1; // Set flag indicating change in test vector detected
	} // if ((chk_data_s.curr_vect.comp_state[SPEED] ...

	// Check if test vector changed
	if (change)
	{
		chk_data_s.prev_vect = chk_data_s.curr_vect; // Update previous test-vector
	} // if (change)

	if (chk_data_s.print_on)
	{
		c_disp <: DISP_CLASS_VECT; // Signal transmission of test vector to print master
		c_disp <: chk_data_s.curr_vect; // Send test vector data
	} // if (chk_data_s.print_on)

} // process_new_test_vector
/*****************************************************************************/
static void check_motor_qei_client_data( // Check QEI results for one motor
	CHECK_TST_TYP &chk_data_s, // Reference to structure containing test check data
	streaming chanend c_tst, // Channel for receiving test vectors from test generator
	streaming chanend c_disp, // Channel for sending display data to print scheduler core
	streaming chanend c_qei // QEI channel between Client and Server
)
{
	TEST_VECT_TYP buffer[VECT_BUF_SIZ]; // Buffer for QEI test vectors (QEI conditions to be tested)
	timer chronometer; // XMOS timer
	unsigned cmd; // QEI Control Command
	int read_cnt = 0; // No of QEI values read from buffer
	int write_cnt = 0; // No of QEI values written to buffer
	unsigned read_off = 0; // read offset into buffer
	unsigned write_off = 0; // wtite offset into buffer
	int do_loop = 1;   // Flag set until loop-end condition found


	chronometer :> chk_data_s.time; // Get start time
	chronometer when timerafter(chk_data_s.time + (MICRO_SEC << 1)) :> chk_data_s.time; // Wait for Test Generation to Start

	// Wait for QEI server to start
	c_qei :> cmd;
	assert(QEI_CMD_ACK == cmd); // ERROR: QEI server did NOT send acknowledge signal

	acquire_lock(); // Acquire Display Mutex
	printcharln(' ');
	printstr( chk_data_s.prefix.str );
	printstr("Start Checks For Motor_"); printintln( chk_data_s.options.flags[TST_MOTOR] );
	release_lock(); // Release Display Mutex

	c_tst :> chk_data_s.curr_vect; // Initialise test-vector structure with 1st test

	// special case: initialisation for first speed test
  chk_data_s.prev_vect = chk_data_s.curr_vect;

	initialise_speed_test_vector( chk_data_s );

	if (chk_data_s.print_on)
	{
		c_disp <: DISP_CLASS_VECT; // Signal transmission of test vector to print master
		c_disp <: chk_data_s.curr_vect; // Send test vector data
	} // if (chk_data_s.print_on)

	// Loop until end-of-testing condition found (QUIT)
	while( do_loop )
	{
		select {
			// Service any change on test channel
			case c_tst :> buffer[write_off] : // Check for new test vector
				// New test vector detected.
				write_cnt++; // Increment write counter
				write_off = write_cnt & VECT_BUF_MASK; // Wrap into buffer range

				assert( (write_cnt - read_cnt) < VECT_BUF_MASK); // Check for buffer overflow
			break; // c_tst

			// Pace QEI Client requests, so as NOT to overload QEI server
			case chronometer when timerafter(chk_data_s.time + QEI_PERIOD) :> chk_data_s.time :
				get_new_qei_client_data( chk_data_s ,c_disp ,c_qei ); // Request data from server & check

				if (0 == chk_data_s.print_on)
				{
					c_disp <: DISP_CLASS_PROG; // Signal printing of progress indicator
				} // if (0 == chk_data_s.print_on)
			break; // case chronometer

			default :
				// Check if any test-vectors needs processing
				if (write_cnt > read_cnt)
				{
					chk_data_s.curr_vect = buffer[read_off];

					process_new_test_vector( chk_data_s ,c_disp ); // Process new test vector

					// Check if testing has ended for current motor
					if (QUIT == chk_data_s.curr_vect.comp_state[CNTRL])
					{
						// Signal QEI server to stop
						c_qei <: QEI_CMD_LOOP_STOP;

						do_loop = 0; // Error flag signals end-of-loop
					} // if (QUIT == chk_data_s.curr_vect.comp_state[CNTRL])

					read_cnt++; // Increment read counter
					read_off = read_cnt & VECT_BUF_MASK; // Wrap into buffer range
				} // if (write_off != read_off)
			break; // default
		} // select
	} // while( loop )

	// Loop until QEI Server acknowledges termination request
	do // while(QEI_CMD_ACK != cmd)
	{
		c_qei :> cmd;

		if (0 == chk_data_s.print_on)
		{
			c_disp <: DISP_CLASS_PROG; // Signal printing of progress indicator
		} // if (0 == chk_data_s.print_on)
	} while(QEI_CMD_ACK != cmd);

	// special case: finalisation for last speed test
	finalise_speed_test_vector( chk_data_s ,chk_data_s.curr_vect.comp_state[SPEED] );

} // check_motor_qei_client_data
/*****************************************************************************/
static void display_test_results( // Display test results for one motor
	const COMMON_TST_TYP &comm_data_s, // Reference to structure containing common test data
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
	printstr( chk_data_s.prefix.str );
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
		printstr( chk_data_s.prefix.str );
		printint( test_errs );
		printstrln( " Tests FAILED, as follows:" );

		// Print Vector Component Names
		for (comp_cnt=1; comp_cnt<NUM_VECT_COMPS; comp_cnt++)
		{
			// Check if any test run for this component
			if (chk_data_s.motor_tsts[comp_cnt])
			{
				printstr( chk_data_s.prefix.str );
				printstr( comm_data_s.comp_data[comp_cnt].comp_name.str );

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
		printstr( chk_data_s.prefix.str );
		printstr( "All Motor_" );
		printint( chk_data_s.options.flags[TST_MOTOR] );
 		printstrln( " Tests Passed" );
	} // else !(check_errs)

	printstrln("");
	release_lock(); // Release Display Mutex

} // display_test_results
/*****************************************************************************/
void check_all_qei_client_data( // Display QEI results for all motors
	const COMMON_TST_TYP &comm_data_s, // Reference to structure containing common test data
	streaming chanend c_tst, // Channel for receiving test vectors from test generator
	streaming chanend c_qei[], // Array of QEI channel between Client and Server
	streaming chanend c_disp // Channel for sending display data to print scheduler core
)
{
	CHECK_TST_TYP chk_data_s; // Structure containing test check data


	init_check_data( comm_data_s ,chk_data_s ); // Initialise check data

	c_tst :> chk_data_s.options; // Get test options from generator core

	init_motor_checks( chk_data_s ); // Initialise QEI parameter structure

	check_motor_qei_client_data( chk_data_s ,c_tst ,c_disp ,c_qei[ chk_data_s.options.flags[TST_MOTOR] ] );

	display_test_results( comm_data_s ,chk_data_s );

	acquire_lock(); // Acquire Display Mutex
	printstr( chk_data_s.prefix.str );
	printstrln( "Test Check Ends " );
	release_lock(); // Release Display Mutex

	_Exit(0); // Exit without house-keeping
} // check_all_qei_client_data
/*****************************************************************************/
