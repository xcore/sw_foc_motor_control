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
static void init_check_data( // Initialise check data for QEI tests
	CHECK_QEI_TYP &chk_data_s // Reference to structure containing test check data
)
{
	int motor_cnt; // counts Motors 


	safestrcpy( chk_data_s.names[ERROR]		," Status" );
	safestrcpy( chk_data_s.names[ORIGIN]	," Origin" );
	safestrcpy( chk_data_s.names[SPIN]		,"  Spin " );
	safestrcpy( chk_data_s.names[SPEED]		," Speed " );

	chk_data_s.fail_cnt = 0; // Clear count of failed tests.

	// Clear error and test accumulators
	for (motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++)
	{
		chk_data_s.all_errs[motor_cnt] = 0;
		chk_data_s.all_tsts[motor_cnt] = 0;
	} // for motor_cnt

} // init_motor_checks
/*****************************************************************************/
static void init_motor_checks( // Initialise QEI parameter structure
	CHECK_QEI_TYP &chk_data_s, // Reference to structure containing test check data
	int motor_id	// Unique Motor identifier
)
{
	int comp_cnt; // Counter for Test Vector components


	chk_data_s.id = motor_id; // Assign Motor identifier
	chk_data_s.prev_ang = (QEI_REV_MASK >> 1); // Set to Non-Zero value

	chk_data_s.params.rev_cnt = 0;
	chk_data_s.params.theta = 0;
	chk_data_s.params.veloc = 0;
	chk_data_s.params.err = 0; // No Errors

	// Clear error and test counters for current motor
	for (comp_cnt=0; comp_cnt<NUM_VECT_COMPS; comp_cnt++)
	{
		chk_data_s.motor_errs[comp_cnt] = 0; 
		chk_data_s.motor_tsts[comp_cnt] = 0; 
	} // for comp_cnt

} // init_motor_checks
/*****************************************************************************/
static void print_test_vector( // Print state of test vector components 
	CHECK_QEI_TYP &chk_data_s // Reference to structure containing test check data
)
{
	acquire_lock(); // Acquire Display Mutex
	printstr( "                                        " );

	switch( chk_data_s.vector.err )
	{
		case ERR_OFF: // No Errors
			printstr( " No_Err" );
		break; // case ERR_OFF:

		case ERR_ON: // Force Error
			printstr( " Err_On" );
		break; // case ERR_OFF:

		default:
			printstrln("ERROR: Unknown QEI Error-state");
			assert(0 == 1);
		break; // default:
	} // switch( chk_data_s.vector.err )

	switch( chk_data_s.vector.orig )
	{
		case ORIG_OFF: // No Origin
			printstr( " No_Orig" );
		break; // case ORIG_OFF:

		case ORIG_ON: // Origin
			printstr( " Orig_On" );
		break; // case ORIG_OFF:

		default:
			printstrln("ERROR: Unknown QEI Origin-state");
			assert(0 == 1);
		break; // default:
	} // switch( chk_data_s.vector.orig )

	switch( chk_data_s.vector.spin )
	{
		case CLOCK: // Clock-wise
			printstr( " Clock-Wise" );
		break; // case CLOCK:

		case ANTI: // Anti-clockwise
			printstr( " Anti-Clock" );
		break; // case ANTI:

		default:
			printstrln("ERROR: Unknown QEI Spin-state");
			assert(0 == 1);
		break; // default:
	} // switch( chk_data_s.vector.spin )

	switch( chk_data_s.vector.speed )
	{
		case ACCEL: // Accelerate
			printstr( " Accelerating" );
		break; // case ACCEL:

		case FAST: // Constant Fast Speed
			printstr( " Fast-steady " );
		break; // case FAST:

		case DECEL: // Accelerate
			printstr( " Decelerating" );
		break; // case DECEL:

		case SLOW: // Constant Slow Speed
			printstr( " Slow-steady " );
		break; // case SLOW:

		default:
			printstrln("ERROR: Unknown QEI Velocity-state");
			assert(0 == 1);
		break; // default:
	} // switch( chk_data_s.vector.speed )

	printstr(" :");
	printintln( chk_data_s.vector.valid );
	release_lock(); // Release Display Mutex
} // print_test_vector
/*****************************************************************************/
static void print_qei_parameters( // Print QEI parameters
	CHECK_QEI_TYP &chk_data_s // Reference to structure containing test check data
)
{
	acquire_lock(); // Acquire Display Mutex
	printstr( "                                         " );
	printint( chk_data_s.id );
	printstr( ":  E=" );
	printint( chk_data_s.params.err );
	printstr( ":  R=" );
	printint( chk_data_s.params.rev_cnt );
	printstr( "  P=" );
	printint( chk_data_s.params.theta );
	printstr( "  V=" );
	printint( chk_data_s.params.veloc );
	printcharln(' ');
	release_lock(); // Release Display Mutex
} // print_qei_parameters
/*****************************************************************************/
static void check_qei_parameters( // Check QEI parameters
	CHECK_QEI_TYP &chk_data_s // Reference to structure containing test check data
)
{
	int inp_err = chk_data_s.params.err; // local copy of error_status parameter
	int inp_ang = chk_data_s.params.theta; // local copy of angular_position parameter
	int inp_vel = chk_data_s.params.veloc; // local copy of angular_velocity parameter
	int inp_rev = chk_data_s.params.rev_cnt; // local copy of revolution_counter parameter


	switch( chk_data_s.vector.err )
	{
		case ERR_OFF: // No Errors
			chk_data_s.motor_tsts[ERROR]++;
			if (ERR_OFF != inp_err)
			{
				chk_data_s.motor_errs[ERROR]++;

				acquire_lock(); // Acquire Display Mutex
				printstrln("ERR_OFF FAILURE");
				release_lock(); // Release Display Mutex
			} // if (ERR_OFF != inp_err)
		break; // case ERR_OFF:

		case ERR_ON: // Force Error
			chk_data_s.motor_tsts[ERROR]++;
			if (ERR_ON != inp_err)
			{
				chk_data_s.motor_errs[ERROR]++;

				acquire_lock(); // Acquire Display Mutex
				printstrln("ERR_ON FAILURE");
				release_lock(); // Release Display Mutex
			} // if (ERR_OFF != inp_err)
		break; // case ERR_OFF:

		default:
			acquire_lock(); // Acquire Display Mutex
			printstrln("ERROR: Unknown QEI Error-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( chk_data_s.vector.err )

	switch( chk_data_s.vector.orig )
	{
		case ORIG_OFF: // No Origin
		break; // case ORIG_OFF:

		case ORIG_ON: // Origin
		break; // case ORIG_OFF:

		default:
			acquire_lock(); // Acquire Display Mutex
			printstrln("ERROR: Unknown QEI Origin-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( chk_data_s.vector.orig )

	switch( chk_data_s.vector.spin )
	{
		case CLOCK: // Clock-wise
			chk_data_s.motor_tsts[SPIN]++;
			if (0 > inp_vel)
			{
				chk_data_s.motor_errs[SPIN]++;

				acquire_lock(); // Acquire Display Mutex
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
				printstrln("Anti-Clock FAILURE");
				release_lock(); // Release Display Mutex
			} // if (0 < inp_vel)
		break; // case ANTI:

		default:
			acquire_lock(); // Acquire Display Mutex
			printstrln("ERROR: Unknown QEI Spin-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( chk_data_s.vector.spin )

	switch( chk_data_s.vector.speed )
	{
		case ACCEL: // Accelerate
		break; // case ACCEL:

		case FAST: // Constant Fast Speed
		break; // case FAST:

		case DECEL: // Accelerate
		break; // case DECEL:

		case SLOW: // Constant Slow Speed
		break; // case SLOW:

		default:
			acquire_lock(); // Acquire Display Mutex
			printstrln("ERROR: Unknown QEI Velocity-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( chk_data_s.vector.speed )

} // check_qei_parameters
/*****************************************************************************/
static void check_vector_qei_client_data( // Display QEI results for one test vector
	CHECK_QEI_TYP &chk_data_s, // Reference to structure containing test check data
	streaming chanend c_qei // QEI channel between Client and Server
)
{
	foc_qei_get_parameters( chk_data_s.params ,c_qei ); // Client function under test

	// Check for difference in Angle supplied by Client
	if (chk_data_s.prev_ang != chk_data_s.params.theta)
	{
		print_qei_parameters( chk_data_s ); // Print new QEI parameters

		// Check if this vector is a valid test
		if (chk_data_s.vector.valid)
		{
			check_qei_parameters( chk_data_s ); // Check new QEI parameters
		} // if (chk_data_s.vector.valid)

		chk_data_s.prev_ang = chk_data_s.params.theta; // Store angle
	} // if (chk_data_s.prev_ang != chk_data_s.params.theta)
} // check_vector_qei_client_data
/*****************************************************************************/
static void check_motor_qei_client_data( // Display QEI results for one motor
	CHECK_QEI_TYP &chk_data_s, // Reference to structure containing test check data
	streaming chanend c_tst, // Channel for receiving test vectors from test generator
	streaming chanend c_qei // QEI channel between Client and Server
)
{
	timer chronometer; // XMOS timer
	int comp_cnt; // Counter for Test Vector components
	unsigned time_val;   // time value
	unsigned time_inc = (40 * MICRO_SEC); // time increment between QEI Client requests. (Typical value when using real motor)
	int do_loop = 1;   // Flag set until loop-end condition found 
	int motor_errs = 0;   // Preset flag to NO errors for current motor
	int motor_tsts = 0;   // Clear test ccounter for current motor


	chronometer :> time_val; // Get start time
	chronometer when timerafter(time_val + (MICRO_SEC << 1)) :> time_val; // Wait for Test Generation to Start

	acquire_lock(); // Acquire Display Mutex
	printstr( "                             " );
	printstr("Start Checks For Motor_"); printintln( chk_data_s.id ); 
	release_lock(); // Release Display Mutex

	chk_data_s.params.veloc = 9999; // Preset to large value 

	// Loop until end condition found
	while( do_loop )
	{
		select {
			// Service any change on test channel
			case c_tst :> chk_data_s.vector :
				// New test vector detected.
				print_test_vector( chk_data_s ); // Print current test vector details
			break; // c_tst 

			// Pace QEI Client requests, so as NOT to overload QEI server
			case chronometer when timerafter(time_val + time_inc) :> time_val :
				check_vector_qei_client_data( chk_data_s ,c_qei ); // Request data from server & check

				if (chk_data_s.params.err)
				{
					do_loop = 0; // Error flag signals end-of-loop
				} // if (chk_data_s.params.err)
			break; // case chronometer
		} // select
	} // while( loop )

	// Update error statistics for current motor
	for (comp_cnt=0; comp_cnt<NUM_VECT_COMPS; comp_cnt++)
	{
		motor_errs += chk_data_s.motor_errs[comp_cnt]; 
		motor_tsts += chk_data_s.motor_tsts[comp_cnt]; 
	} // for comp_cnt

	chk_data_s.all_errs[chk_data_s.id] += motor_errs;
	chk_data_s.all_tsts[chk_data_s.id] += motor_tsts;

	acquire_lock(); // Acquire Display Mutex

	printint( motor_tsts );
	printstr( " tests run:" );

	// Check if this motor had any errors
	if (motor_errs)
	{
		printstr( "  " );
		printint( motor_errs );
		printstrln( " tests FAILED, as follows:" );
		printcharln(' ');

		// Print Vector Component Names
		for (comp_cnt=0; comp_cnt<NUM_VECT_COMPS; comp_cnt++)
		{
			printstr( chk_data_s.names[comp_cnt] );
			printstr(" : ");
			printint( chk_data_s.motor_tsts[comp_cnt] );
			printstr( " tests run: " );

			if (chk_data_s.motor_errs[comp_cnt])
			{
				printint( chk_data_s.motor_errs[comp_cnt] );
				printstr(" FAILURES");
			} // if (chk_data_s.motor_errs[comp_cnt])
			printcharln(' ');
		} // for comp_cnt
	} // if (motor_errs)
	else
	{
		printstrln( " ALL TESTS PASSED" );
	} // else !(motor_errs)

	release_lock(); // Release Display Mutex

	chronometer when timerafter(time_val + MILLI_SEC) :> time_val; // Wait for Test Generation to End
} // check_motor_qei_client_data
/*****************************************************************************/
void check_all_qei_client_data( // Display QEI results for all motors
	streaming chanend c_tst, // Channel for receiving test vectors from test generator
	streaming chanend c_qei[] // Array of QEI channel between Client and Server
)
{
	CHECK_QEI_TYP chk_data_s; // Structure containing test check data
	int motor_cnt; // counts Motors 


	init_check_data( chk_data_s ); // Initialise check data

	// Loop through motors, so we can print results serially
	for (motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++)
	{
		init_motor_checks( chk_data_s ,motor_cnt ); // Initialise QEI parameter structure

		check_motor_qei_client_data( chk_data_s ,c_tst ,c_qei[motor_cnt] );
	} // for motor_cnt

	acquire_lock(); // Acquire Display Mutex
	printstr( "                             " );
	printstrln( "Test Check Ends " );
	release_lock(); // Release Display Mutex
} // check_all_qei_client_data
/*****************************************************************************/
