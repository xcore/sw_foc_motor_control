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
	int veloc_val // input velocity
) // Returns error_bound
/* The Maths for this is a bit complicated as it used non-linear arithmetic.
 * However, here goes ...
 * 
 * The Speed (S) and time-period between QEI phase changes (T) are related by the following equation: 
 *		Cf = S * T   (where Cf is a floating-point constant, but in this code
 * is represented as an integer with rounding errors (Ci + dC), 
 * Ci is TICKS_PER_MIN_PER_QEI , and  dC = SECS_PER_MIN/2 (currently 5859360 +/- 30 in qei_common.h)
 * 
 * The test-generator chooses a test Speed (Sg), this is converted to a time-period (T + dT) with rounding errors
 * which is used to generate raw QEI phase changes at the appropriate time.
 * 
 * The QEI Server, converts the time-period between QEI phase changes into a angular_velocity estimate,
 * this too has a rounding error.
 * 
 * The test-checker reads the QEI velocity parameter and converts it into a speed with rounding errors (Sc + dS).
 * The checker needs to calculate dS, in order to know the required error-bounds for Sc.
 * 
 *		T = C / Sg  ,  Therefore dT = dC/Sg. 			(NB Sg has no error)     --- (1)
 * Using integer division, and rounding-up:   dT = (dC + Sg - 1)/(int)Sg  or  dT ~= 1 + dC/(int)Sg    --- (2)
 * 
 *		Sc = C / T  ,  Therefore  dS ~= (T.dC + C.dT)/(T.T)  (differentiation of product rule)
 * Substituting for T for (1) gives:  dS = Sg(dC + Sg.dT)/Ci
 * Substituting for dT for (2) gives:  dS = Sg(2.dC + Sg)/Ci
 * Using integer division, and rounding-up:   dS = (Sg(2.dC + Sg) + Ci - 1)/(int)Ci  or  ...
 * 
 *		dS ~= 1 + (Sg(2.dC + Sg))/(int)Ci  --- (3)
 * 		----------------------------
 * 
 *  NB Currently, for all values of Sg <= 2390, dS is always 1
 */
{
	unsigned bound; // Output error_bound


	bound = veloc_val * (SECS_PER_MIN + veloc_val); // Compute numerator for integer division
	bound = 1 + (bound / TICKS_PER_MIN_PER_QEI);		// NB we already rounded-up in (3)

	/* When using the simulator, Occasionally QEI phases changes are not generated at the appropriate time
	 * This introduces another source of error, which has yet to be quantfied, 
	 * so an empirically found fudge factor is used!-(
	 */
	return (2 * bound); // Multiply by fudge factor
}	// eval_speed_bound
/*****************************************************************************/
static void init_check_data( // Initialise check data for QEI tests
	CHECK_QEI_TYP &chk_data_s // Reference to structure containing test check data
)
{
	int motor_cnt; // counts Motors 


	init_common_data( chk_data_s.common ); // Initialise data common to Generator and Checker

	safestrcpy( chk_data_s.padstr1 ,"                                             " );
	safestrcpy( chk_data_s.padstr2 ,"                              " );

	chk_data_s.fail_cnt = 0; // Clear count of failed tests.

	// Evaluate error bounds for speed checks
	chk_data_s.hi_bound = eval_speed_bound( HIGH_SPEED ); 
	chk_data_s.lo_bound = eval_speed_bound( LOW_SPEED ); 

	chk_data_s.print = PRINT_TST_QEI; // Set print mode
	chk_data_s.dbg = 0; // Set debug mode

	// Clear error and test accumulators
	for (motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++)
	{
		chk_data_s.all_errs[motor_cnt] = 0;
		chk_data_s.all_tsts[motor_cnt] = 0;
	} // for motor_cnt

} // init_check_data
/*****************************************************************************/
static void init_motor_checks( // Initialise QEI parameter structure
	CHECK_QEI_TYP &chk_data_s, // Reference to structure containing test check data
	int motor_id	// Unique Motor identifier
)
{
	int comp_cnt; // Counter for Test Vector components


	chk_data_s.id = motor_id; // Assign Motor identifier

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
static void print_qei_parameters( // Print QEI parameters
	CHECK_QEI_TYP &chk_data_s // Reference to structure containing test check data
)
{
	acquire_lock(); // Acquire Display Mutex
	printstr( chk_data_s.padstr1 );

	printint( chk_data_s.id );
	printstr( ":  E=" );
	printint( chk_data_s.curr_params.err );
	printstr( ":  R=" );
	printint( chk_data_s.curr_params.rev_cnt );
	printstr( "  P=" );
	printint( chk_data_s.curr_params.theta );
	printstr( "  V=" );
	printint( chk_data_s.curr_params.veloc );
	printcharln(' ');
	release_lock(); // Release Display Mutex
} // print_qei_parameters
/*****************************************************************************/
static void check_qei_position_reset( // Check for correct position reset after origin detection
	CHECK_QEI_TYP &chk_data_s // Reference to structure containing test check data
)
{
	int tmp_val;	// temporary manipulation variable
	int ang_bound;	// angular_position margin of error


	// Check for steady speed
	if ((FAST != chk_data_s.curr_vect.comp_state[SPEED]) && (SLOW != chk_data_s.curr_vect.comp_state[SPEED]))
	{ // NOT Steady Speed
		acquire_lock(); // Acquire Display Mutex
		printcharln(' ');
		printstr( chk_data_s.padstr1 );
		printstrln("INVALID POSITION RESET TEST. ABORTIMNG");
		release_lock(); // Release Display Mutex
		assert(0 == 1);
	} // if ((FAST != chk_data_s.curr_vect.comp_state[SPEED]) && (SLOW != chk_data_s.curr_vect.comp_state[SPEED]))

	// Steady Speed

	/* Calculate error bounds for angular position after reset: 
		 In the worst case, the angular position may not be measured until 
		 a whole QEI_PERIOD after the reset. The amount the position changes during this time
     depends on the STEADY speed used for the test. We need to calculate the following

		 ang_mar = (QEI_PERIOD / PLATFORM_REFERENCE_HZ) * (RPM / 60) * QEI_PER_REV

		 This has to be done without overflow, and retaining as much precision as possible.
     So it is done in the following order ...
	*/ 
	tmp_val = QEI_PERIOD * abs(chk_data_s.curr_params.veloc); // ~24 bits
	tmp_val = (tmp_val + (SECS_PER_MIN - 1)) / SECS_PER_MIN; // division with round-up // ~19 bits
	tmp_val *= QEI_PER_REV; // ~29 bits
	ang_bound = (tmp_val + (PLATFORM_REFERENCE_HZ - 1)) / PLATFORM_REFERENCE_HZ; // division with round-up // ~2 bits

	// Check for out-of-bounds angular position
	if (ang_bound < abs(chk_data_s.curr_params.theta))
	{
		acquire_lock(); // Acquire Display Mutex
		printcharln(' ');
		printstr( chk_data_s.padstr1 );
		printstrln("ORIG_ANG_POS FAILURE");
		release_lock(); // Release Display Mutex
	} // if (ang_bound < abs(chk_data_s.curr_params.theta))

} // check_qei_position_reset
/*****************************************************************************/
static void check_qei_origin_detection( // Check correct update of QEI parameters due to origin detection
	CHECK_QEI_TYP &chk_data_s // Reference to structure containing test check data
)
{
	int rev_diff = chk_data_s.curr_params.rev_cnt - chk_data_s.prev_params.rev_cnt; // Change in rev. counter


	chk_data_s.motor_tsts[ORIGIN]++;

	// Check for expected value
	if (rev_diff == chk_data_s.orig_chk)
	{ // Found expected value (Test passed)

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
			chk_data_s.motor_errs[ORIGIN]++;

			acquire_lock(); // Acquire Display Mutex
			printcharln(' ');
			printstr( chk_data_s.padstr1 );

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
	CHECK_QEI_TYP &chk_data_s // Reference to structure containing test check data
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
				case QEI_ERR_OFF:
					printstrln("ERR_OFF FAILURE");
				break; // case QEI_ERR_OFF:
		
				case QEI_ERR_ON: // Start error_status test
					printstrln("ERR_ON FAILURE");
				break; // case QEI_ERR_ON:
		
				default:
					printstrln("ERROR: Unknown QEI Error-state");
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
	CHECK_QEI_TYP &chk_data_s // Reference to structure containing test check data
)
{
	int inp_vel = chk_data_s.curr_params.veloc; // local copy of angular_velocity parameter


	switch( chk_data_s.curr_vect.comp_state[SPIN] )
	{
		case CLOCK: // Clock-wise
			chk_data_s.motor_tsts[SPIN]++;
			if (0 > inp_vel)
			{
				chk_data_s.motor_errs[SPIN]++;

				acquire_lock(); // Acquire Display Mutex
				printcharln(' ');
				printstr( chk_data_s.padstr1 );
				printstrln("Clock-Wise FAILURE");
				release_lock(); // Release Display Mutex
			} // if (0 > inp_vel)
		break; // case CLOCK:

		case ANTI: // Anti-clockwise
			chk_data_s.motor_tsts[SPIN]++;
			if (0 < inp_vel)
			{
				chk_data_s.motor_errs[SPIN]++;

				acquire_lock(); // Acquire Display Mutex
				printcharln(' ');
				printstr( chk_data_s.padstr1 );
				printstrln("Anti-Clock FAILURE");
				release_lock(); // Release Display Mutex
			} // if (0 < inp_vel)
		break; // case ANTI:

		default:
			acquire_lock(); // Acquire Display Mutex
			printcharln(' ');
			printstr( chk_data_s.padstr1 );
			printstrln("ERROR: Unknown QEI Spin-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( chk_data_s.curr_vect.comp_state[SPIN] )

} // check_qei_spin_direction
/*****************************************************************************/
static void update_qei_angular_speed( // Update accumulators for calculating QEI speed
	CHECK_QEI_TYP &chk_data_s // Reference to structure containing test check data
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
			printstr( chk_data_s.padstr1 );
			printstrln("ERROR: Unknown QEI Speed-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( chk_data_s.curr_vect.comp_state[SPEED] )

	chk_data_s.speed_num++; // Update No. of samples in accumulator
} // update_qei_angular_speed
/*****************************************************************************/
static void check_qei_angular_speed( // Check all QEI speed as motor accelerates/decelerates
	CHECK_QEI_TYP &chk_data_s, // Reference to structure containing test check data
	const SPEED_QEI_ENUM cur_speed	// Speed-state to check
)
{
	int mean; // Mean speed parameter under-test


	// Calculate mean from accumulated data (with rounding to -infinity)
	if (0 > chk_data_s.speed_sum)
	{  // -ve Rounding
		mean = (chk_data_s.speed_sum - (chk_data_s.speed_num >> 1)) / chk_data_s.speed_num;
	} // if (0 > chk_data_s.speed_sum)
	else
	{  // +ve Rounding
		mean = (chk_data_s.speed_sum + (chk_data_s.speed_num >> 1)) / chk_data_s.speed_num;
	} // else !(0 > chk_data_s.speed_sum)

// acquire_lock(); printstr("NUM="); printint( chk_data_s.speed_num ); printstr("  MEAN="); printintln( mean ); release_lock(); // MB~

	switch( cur_speed )
	{
		case ACCEL: // Accelerate
			chk_data_s.motor_tsts[SPEED]++;

			if (0 >= mean)
			{
				chk_data_s.motor_errs[SPEED]++;

				acquire_lock(); // Acquire Display Mutex
				printcharln(' ');
				printstr( chk_data_s.padstr1 );
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
				printstr( chk_data_s.padstr1 );
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
				printstr( chk_data_s.padstr1 );
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
				printstr( chk_data_s.padstr1 );
				printstrln("SLOW FAILURE");
				release_lock(); // Release Display Mutex
			} // if (chk_data_s.hi_bound < abs(mean - HIGH_SPEED))
		break; // case SLOW:

		default:
			acquire_lock(); // Acquire Display Mutex
			printcharln(' ');
			printstr( chk_data_s.padstr1 );
			printstrln("ERROR: Unknown QEI Speed-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( cur_speedspeed )

} // check_qei_angular_speed
/*****************************************************************************/
static void check_qei_parameters( // Check all QEI parameters
	CHECK_QEI_TYP &chk_data_s // Reference to structure containing test check data
)
{
	// Check if Error-status test is activated
	if (chk_data_s.common.options.flags[TST_ERROR])
	{
		check_qei_error_status( chk_data_s ); // Check QEI error status
	} // if (chk_data_s.common.options.flags[TST_ERROR])

	// Check if Origin-bit test is activated
	if (chk_data_s.common.options.flags[TST_ORIGIN])
	{
		check_qei_origin_detection( chk_data_s ); // Check QEI origin detection
	} // if (chk_data_s.common.options.flags[TST_ORIGIN])

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
	CHECK_QEI_TYP &chk_data_s, // Reference to structure containing test check data
	streaming chanend c_qei // QEI channel between Client and Server
)
{
	int do_test = 0;	// Flag set when next test required


	// Get new parameter values from Client function under test
	foc_qei_get_parameters( chk_data_s.curr_params ,c_qei );

	// Check for change in non-speed parameters
	do_test = parameter_compare( chk_data_s.curr_params ,chk_data_s.prev_params ); 

	// Check for parameter change
	if (do_test)
	{ // Parameters changed

		if (chk_data_s.print)
		{
			print_qei_parameters( chk_data_s ); // Print new QEI parameters
		} // if (chk_data_s.print)

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
	CHECK_QEI_TYP &chk_data_s // Reference to structure containing test check data
)
{
	// Clear accumulated data
	chk_data_s.speed_sum = 0; 
	chk_data_s.speed_num = 0; 
} // initialise_speed_test_vector 
/*****************************************************************************/
static void finalise_speed_test_vector( // terminate speed test and check results
	CHECK_QEI_TYP &chk_data_s, // Reference to structure containing test check data
	const SPEED_QEI_ENUM cur_speed	// Speed-state to finalise
)
{
	check_qei_angular_speed( chk_data_s ,cur_speed ); // Check QEI angular speed
} // finalise_speed_test_vector 
/*****************************************************************************/
static void process_new_test_vector( // Process new test vector
	CHECK_QEI_TYP &chk_data_s // Reference to structure containing test check data
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

	// Check if origin-bit test
	if (ORIG_ON == chk_data_s.curr_vect.comp_state[ORIGIN])
	{ // Initialise origin test

		// Check if test already running
		if (0 < chk_data_s.orig_cnt)
		{
			acquire_lock(); // Acquire Display Mutex
			printstr( chk_data_s.padstr1 );
			printstrln("ERROR: Previous Origin test NOT completed");
			release_lock(); // Release Display Mutex
			assert( 0 == 1); // Abort
		} // if (0 < chk_data_s.orig_cnt)
		else
		{ // Start new test
			switch( chk_data_s.curr_vect.comp_state[SPIN] )
			{
				case CLOCK: // Clock-wise
					chk_data_s.orig_chk = 1; // Expected increment in rev. counter
				break; // case CLOCK:
		
				case ANTI: // Anti-clockwise
					chk_data_s.orig_chk = -1; // Expected decrement in rev. counter
				break; // case ANTI:
		
				default:
					acquire_lock(); // Acquire Display Mutex
					printcharln(' ');
					printstr( chk_data_s.padstr1 );
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

// if ((chk_data_s.curr_vect.comp_state[SPEED] == FAST) &&	(chk_data_s.curr_vect.comp_state[SPIN] == ANTI)) chk_data_s.print = 1; //MB~

		initialise_speed_test_vector( chk_data_s );

		change = 1; // Set flag indicating change in test vector detected
	} // if ((chk_data_s.curr_vect.comp_state[SPEED] ...

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
static void check_motor_qei_client_data( // Display QEI results for one motor
	CHECK_QEI_TYP &chk_data_s, // Reference to structure containing test check data
	streaming chanend c_tst, // Channel for receiving test vectors from test generator
	streaming chanend c_qei // QEI channel between Client and Server
)
{
	timer chronometer; // XMOS timer
	int comp_cnt; // Counter for Test Vector components
	int do_loop = 1;   // Flag set until loop-end condition found 
	int motor_errs = 0;   // Preset flag to NO errors for current motor
	int motor_tsts = 0;   // Clear test ccounter for current motor


	chronometer :> chk_data_s.time; // Get start time
	chronometer when timerafter(chk_data_s.time + (MICRO_SEC << 1)) :> chk_data_s.time; // Wait for Test Generation to Start

	acquire_lock(); // Acquire Display Mutex
	printcharln(' ');
	printstr( chk_data_s.padstr1 );
	printstr("Start Checks For Motor_"); printintln( chk_data_s.id ); 
	release_lock(); // Release Display Mutex

	c_tst :> chk_data_s.curr_vect; // Initialise test-vector structure with 1st test

	// special case: initialisation for first speed test
  chk_data_s.prev_vect = chk_data_s.curr_vect;

	initialise_speed_test_vector( chk_data_s ); 

	if (chk_data_s.print)
	{
		print_test_vector( chk_data_s.common ,chk_data_s.curr_vect ,chk_data_s.padstr1 ); // Print new test vector details
	} // if (chk_data_s.print)

	// Loop until end condition found
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
					do_loop = 0; // Error flag signals end-of-loop
				} // if (QUIT == chk_data_s.curr_vect.comp_state[CNTRL])
			break; // c_tst 

			// Pace QEI Client requests, so as NOT to overload QEI server
			case chronometer when timerafter(chk_data_s.time + QEI_PERIOD) :> chk_data_s.time :
				get_new_qei_client_data( chk_data_s ,c_qei ); // Request data from server & check

				if (0 == chk_data_s.print)
				{
					printchar('.'); // Progress indicator
				} // if (0 == chk_data_s.print)
			break; // case chronometer
		} // select
	} // while( loop )

	// special case: finalisation for last speed test
	finalise_speed_test_vector( chk_data_s ,chk_data_s.curr_vect.comp_state[SPEED] ); 

	// Update error statistics for current motor
	for (comp_cnt=1; comp_cnt<NUM_VECT_COMPS; comp_cnt++)
	{
		motor_errs += chk_data_s.motor_errs[comp_cnt]; 
		motor_tsts += chk_data_s.motor_tsts[comp_cnt]; 
	} // for comp_cnt

	chk_data_s.all_errs[chk_data_s.id] += motor_errs;
	chk_data_s.all_tsts[chk_data_s.id] += motor_tsts;

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
		printstr( chk_data_s.padstr1 );
		printstr( "All Motor_" );
		printint( chk_data_s.id );
 		printstrln( " Tests PASSED" );
	} // else !(motor_errs)

	release_lock(); // Release Display Mutex

	chronometer when timerafter(chk_data_s.time + MILLI_SEC) :> chk_data_s.time; // Wait for Test Generation to End
} // check_motor_qei_client_data
/*****************************************************************************/
void check_all_qei_client_data( // Display QEI results for all motors
	streaming chanend c_tst, // Channel for receiving test vectors from test generator
	streaming chanend c_qei[] // Array of QEI channel between Client and Server
)
{
	CHECK_QEI_TYP chk_data_s; // Structure containing test check data
	int motor_cnt; // counts Motors 
	int errs_all = 0;	// Preset flag to NO errors for all motors
	int tsts_all = 0;	// Clear test counter for all motors


	init_check_data( chk_data_s ); // Initialise check data

	c_tst :> chk_data_s.common.options; // Get test options from generator core

	// Loop through motors, so we can print results serially
	for (motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++)
	{
		init_motor_checks( chk_data_s ,motor_cnt ); // Initialise QEI parameter structure

		check_motor_qei_client_data( chk_data_s ,c_tst ,c_qei[motor_cnt] );
	} // for motor_cnt

	acquire_lock(); // Acquire Display Mutex
	printstr( chk_data_s.padstr1 );
	printstrln( "Test Check Ends " );

	// Update error statistics for all motors
	for (motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++)
	{
		errs_all += chk_data_s.all_errs[motor_cnt]; 
		tsts_all += chk_data_s.all_tsts[motor_cnt]; 
	} // for motor_cnt

	printint( tsts_all );
	printstrln( " tests run" );

	// Check for any errors
	if (errs_all)
	{
		printint( errs_all );
		printstr( " TESTS FAILED");
		printstrln( "   (See individual motors for details)" );
	} // if (errs_all)
	else
	{
		printstrln( "ALL TESTS PASSED" );
	} // else !(errs_all)

	printcharln( ' ' );
	release_lock(); // Release Display Mutex
} // check_all_qei_client_data
/*****************************************************************************/
