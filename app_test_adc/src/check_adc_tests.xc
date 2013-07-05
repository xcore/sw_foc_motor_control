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

#include "check_adc_tests.h"

/*****************************************************************************/
static unsigned eval_speed_bound( // Evaluate error bounds for given speed 
	int veloc_val // input velocity
) // Returns error_bound
{
	unsigned bound; // Output error_bound

	bound = (veloc_val + 8) >> 4; // 1/16 //MB~ ToDo

	return bound;
}	// eval_speed_bound
/*****************************************************************************/
static void init_check_data( // Initialise check data for ADC tests
	CHECK_ADC_TYP &chk_data_s // Reference to structure containing test check data
)
{
	VECT_COMP_ENUM comp_cnt; // Counter for Test Vector components
	ADC_PHASE_ENUM phase_cnt; // Counter for ADC phases


	init_common_data( chk_data_s.common ); // Initialise data common to Generator and Checker

	safestrcpy( chk_data_s.padstr1 ,"                                             " );
	safestrcpy( chk_data_s.padstr2 ,"                              " );

	chk_data_s.fail_cnt = 0; // Clear count of failed tests.

	// Evaluate error bounds for speed checks
	chk_data_s.hi_bound = eval_speed_bound( HIGH_SPEED ); 
	chk_data_s.lo_bound = eval_speed_bound( LOW_SPEED ); 

	chk_data_s.print = PRINT_TST_ADC; // Set print mode
	chk_data_s.dbg = 0; // Set debug mode

	chk_data_s.all_errs = 0;
	chk_data_s.all_tsts = 0;

	for (phase_cnt=0; phase_cnt<NUM_ADC_PHASES; phase_cnt++)
	{
		chk_data_s.curr_params.vals[phase_cnt] = 0;
	} // for phase_cnt

	chk_data_s.prev_params = chk_data_s.curr_params; // Initialise previous parameter values

	// Clear error and test counters for current motor
	for (comp_cnt=0; comp_cnt<NUM_VECT_COMPS; comp_cnt++)
	{
		chk_data_s.motor_errs[comp_cnt] = 0; 
		chk_data_s.motor_tsts[comp_cnt] = 0; 
	} // for comp_cnt

} // init_check_data
/*****************************************************************************/
static void print_adc_parameters( // Print ADC parameters
	CHECK_ADC_TYP &chk_data_s // Reference to structure containing test check data
)
{
	ADC_PHASE_ENUM phase_cnt; // Counter for ADC phases


	acquire_lock(); // Acquire Display Mutex
	printstr( chk_data_s.padstr1 );

	printint( chk_data_s.curr_time );
	printstr( " <" );
	for (phase_cnt=0; phase_cnt<NUM_ADC_PHASES; phase_cnt++)
	{
		printstr( ": " );
		printint( chk_data_s.curr_params.vals[phase_cnt] );
		printchar(' ');
	} // for phase_cnt
	printcharln(' ');
	release_lock(); // Release Display Mutex
} // print_adc_parameters
//*****************************************************************************/
static void check_adc_sum( // Check 3 phases sum to zero
	CHECK_ADC_TYP &chk_data_s // Reference to structure containing test check data
)
{
	ADC_PHASE_ENUM phase_cnt; // Counter for ADC phases
	int sum_val = 0; // Clear sum of adc values


	switch( chk_data_s.curr_vect.comp_state[SUM] )
	{
		case SUM_ON: // Do Zero-Sum test
			chk_data_s.motor_tsts[SUM]++;
			for (phase_cnt=0; phase_cnt<NUM_ADC_PHASES; phase_cnt++)
			{
				sum_val += chk_data_s.curr_params.vals[phase_cnt];
			} // for phase_cnt

			if (0 != sum_val)
			{
				chk_data_s.motor_errs[SUM]++;

				acquire_lock(); // Acquire Display Mutex
				printcharln(' ');
				printstr( chk_data_s.padstr1 );
				printstrln("Zero-Sum FAILURE");
				release_lock(); // Release Display Mutex
			} // if (0 > inp_vel)
		break; // case SUM_ON:

		default:
			acquire_lock(); // Acquire Display Mutex
			printcharln(' ');
			printstr( chk_data_s.padstr1 );
			printstrln("ERROR: Unknown ADC Zero-Sum test state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( chk_data_s.curr_vect.comp_state[SUM] )

} // check_adc_sum
/*****************************************************************************/
static void check_adc_spin_direction( // Check correct update of ADC spin direction
	CHECK_ADC_TYP &chk_data_s // Reference to structure containing test check data
)
{
#ifdef MB
	int inp_vel = 0; //MB~ ToDo Evaluate angular velocity


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
			printstrln("ERROR: Unknown ADC Spin-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( chk_data_s.curr_vect.comp_state[SPIN] )
#endif //MB

} // check_adc_spin_direction
/*****************************************************************************/
static void update_adc_angular_speed( // Update accumulators for calculating ADC speed
	CHECK_ADC_TYP &chk_data_s // Reference to structure containing test check data
)
{
	int curr_speed = 0; //MB~ ToDo Evaluate angular velocity


	switch( chk_data_s.curr_vect.comp_state[SPEED] )
	{
		case FAST: case SLOW: // Constant Speed
			chk_data_s.speed_sum += curr_speed; // Accumulate constant-speed
		break; // case FAST: SLOW: 

		default:
			acquire_lock(); // Acquire Display Mutex
			printcharln(' ');
			printstr( chk_data_s.padstr1 );
			printstrln("ERROR: Unknown ADC Speed-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( chk_data_s.curr_vect.comp_state[SPEED] )

	chk_data_s.speed_num++; // Update No. of samples in accumulator

	// Check if we have enough data for test MB~ ToDo
	if (10 == chk_data_s.speed_num)
	{
		chk_data_s.more = 0; // Clear flag indicating NO more data required
	} // if (10 == chk_data_s.speed_num)
} // update_adc_angular_speed
/*****************************************************************************/
static void check_adc_angular_speed( // Check all ADC speed as motor accelerates/decelerates
	CHECK_ADC_TYP &chk_data_s, // Reference to structure containing test check data
	const SPEED_ADC_ENUM cur_speed	// Speed-state to check
)
{
	int mean; // Mean speed parameter under-test


	assert( 0 != chk_data_s.speed_num); // ERROR: No speed samples

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
			printstrln("ERROR: Unknown ADC Speed-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( cur_speedspeed )

} // check_adc_angular_speed
/*****************************************************************************/
static void check_adc_parameters( // Check all ADC parameters
	CHECK_ADC_TYP &chk_data_s // Reference to structure containing test check data
)
{
	check_adc_sum( chk_data_s ); // Check ADC zero-sum
	check_adc_spin_direction( chk_data_s ); // Check ADC spin direction

	update_adc_angular_speed( chk_data_s ); // Update data on ADC angular speed
} // check_adc_parameters
/*****************************************************************************/
static int parameter_compare( // Check if 2 sets of ADC parameters are different
	ADC_PARAM_TYP &params_a,	// Structure containing 1st set of ADC parameters
	ADC_PARAM_TYP &params_b	// Structure containing 2nd set of ADC parameters
) // return TRUE (1) if sets are different, FALSE(0) if equal
{
	ADC_PHASE_ENUM phase_cnt; // Counter for ADC phases


	for (phase_cnt=0; phase_cnt<NUM_ADC_PHASES; phase_cnt++)
	{
		if (params_a.vals[phase_cnt] != params_b.vals[phase_cnt]) return 1;	// Check each phase value
	} // for phase_cnt

	return 0; // No differences found
} // parameter_compare
/*****************************************************************************/
static void get_new_adc_client_data( // Get next set of ADC parameters
	CHECK_ADC_TYP &chk_data_s, // Reference to structure containing test check data
	streaming chanend c_sin, // Channel for communication with Sine_Generator cores
	streaming chanend c_adc // ADC channel between Client and Server
)
{
	int do_test = 0;	// Flag set when next test required


	c_sin :> chk_data_s.curr_time; // get time-stamp for new ADC parameters

	// Loop until parameter change
	do // while(!do_test)
	{
		// Get new parameter values from Client function under test
		foc_adc_get_parameters( chk_data_s.curr_params ,c_adc );

		// Check for change in non-speed parameters
		do_test = parameter_compare( chk_data_s.curr_params ,chk_data_s.prev_params ); 
	} while(!do_test);

	c_sin <: (int)TST_REQ_CMD; // Request next ADC value

	if (chk_data_s.print)
	{
		print_adc_parameters( chk_data_s ); // Print new ADC parameters
	} // if (chk_data_s.print)

	// Check if this current test vector is valid
	if (VALID == chk_data_s.curr_vect.comp_state[CNTRL])
	{
		check_adc_parameters( chk_data_s ); // Check new ADC parameters
	} // if (VALID == chk_data_s.curr_vect.comp_state[CNTRL])

	chk_data_s.prev_time = chk_data_s.curr_time; // Store previous time-stamp
	chk_data_s.prev_params = chk_data_s.curr_params; // Store previous parameter values
} // get_new_adc_client_data
/*****************************************************************************/
static void initialise_speed_test_vector( // Initialise data for new speed test
	CHECK_ADC_TYP &chk_data_s // Reference to structure containing test check data
)
{
	// Clear accumulated data
	chk_data_s.speed_sum = 0; 
	chk_data_s.speed_num = 0; 
} // initialise_speed_test_vector 
/*****************************************************************************/
static void finalise_speed_test_vector( // terminate speed test and check results
	CHECK_ADC_TYP &chk_data_s, // Reference to structure containing test check data
	const SPEED_ADC_ENUM cur_speed	// Speed-state to finalise
)
{
//MB~	check_adc_angular_speed( chk_data_s ,cur_speed ); // Check ADC angular speed
} // finalise_speed_test_vector 
/*****************************************************************************/
static void check_motor_adc_client_data( // Display ADC results for one motor
	CHECK_ADC_TYP &chk_data_s, // Reference to structure containing test check data
	streaming chanend c_tst, // Channel for receiving test vectors from test generator
	streaming chanend c_sin, // Channel for communication with Sine_Generator cores
	streaming chanend c_adc // ADC channel between Client and Server
)
{
	int comp_cnt; // Counter for Test Vector components
	int do_loop = 1;   // Flag set until loop-end condition found 
	int motor_errs = 0;   // Preset flag to NO errors for current motor
	int motor_tsts = 0;   // Clear test ccounter for current motor


	acquire_lock(); // Acquire Display Mutex
	printcharln(' ');
	printstr( chk_data_s.padstr1 );
	printstr("Start Checks For Motor_"); printintln( MOTOR_ID ); 
	release_lock(); // Release Display Mutex

	// Loop until end condition found
	while( do_loop )
	{
		c_tst :> chk_data_s.curr_vect; // get new test-vector

		if (chk_data_s.print)
		{
			print_test_vector( chk_data_s.common ,chk_data_s.curr_vect ,chk_data_s.padstr1 ); // Print new test vector details
		} // if (chk_data_s.print)

		// Check if testing has ended for current motor
		if (QUIT == chk_data_s.curr_vect.comp_state[CNTRL])
		{
			do_loop = 0; // Error flag signals end-of-loop
		} // if (QUIT == chk_data_s.curr_vect.comp_state[CNTRL])
		else
		{ // Do new test
			initialise_speed_test_vector( chk_data_s ); // Initialise accumulators

			chk_data_s.more = 1; // Initialise Flag to 'more data required'

			// Loop until enough data collected
			while(chk_data_s.more)
			{
				get_new_adc_client_data( chk_data_s ,c_sin ,c_adc ); // Request data from server & check

				if (0 == chk_data_s.print)
				{
					printchar('.'); // Progress indicator
				} // if (0 == chk_data_s.print)
			} // while(chk_data_s.more)

			finalise_speed_test_vector( chk_data_s ,chk_data_s.curr_vect.comp_state[SPEED] ); 
		} // else !(QUIT == chk_data_s.curr_vect.comp_state[CNTRL])

		c_tst <: (int)END_TST_CMD; // Signal to test generator that this test is complete
	} // while( loop )

	// Update error statistics for current motor
	for (comp_cnt=1; comp_cnt<NUM_VECT_COMPS; comp_cnt++)
	{
		motor_errs += chk_data_s.motor_errs[comp_cnt]; 
		motor_tsts += chk_data_s.motor_tsts[comp_cnt]; 
	} // for comp_cnt

	chk_data_s.all_errs += motor_errs;
	chk_data_s.all_tsts += motor_tsts;

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
		printint( MOTOR_ID );
 		printstrln( " Tests PASSED" );
	} // else !(motor_errs)

	release_lock(); // Release Display Mutex
} // check_motor_adc_client_data
/*****************************************************************************/
void check_all_adc_client_data( // Display ADC results for all motors
	streaming chanend c_adc[], // Array of channel for communication with ADC_Server
	streaming chanend c_sin, // Channel for communication with Sine_Generator cores
	streaming chanend c_tst // Channel for communication with Test_Generator
)
{
	CHECK_ADC_TYP chk_data_s; // Structure containing test check data



	c_sin <: (int)TST_REQ_CMD; // Request next ADC value

	// Initialise parameter values by calling Client function
	foc_adc_get_parameters( chk_data_s.curr_params ,c_adc[MOTOR_ID]  );

	c_sin <: (int)TST_REQ_CMD; // Request next ADC value

	init_check_data( chk_data_s ); // Initialise check data

	c_tst :> chk_data_s.common.options; // Get test options from generator core

	check_motor_adc_client_data( chk_data_s ,c_tst ,c_sin ,c_adc[MOTOR_ID] );

	acquire_lock(); // Acquire Display Mutex
	printstr( chk_data_s.padstr1 );
	printstrln( "Test Check Ends " );
	release_lock(); // Release Display Mutex
} // check_all_adc_client_data
/*****************************************************************************/
