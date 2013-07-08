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
static unsigned eval_phase_bound( // Evaluate error bounds for given gain 
	int veloc_val // input velocity
) // Returns error_bound
{
	unsigned bound; // Output error_bound

	bound = (veloc_val + 8) >> 4; // 1/16 //MB~ ToDo

	return bound;
}	// eval_phase_bound
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

	// Evaluate error bounds for gain checks
	chk_data_s.hi_bound = eval_phase_bound( MAX_GAIN ); 
	chk_data_s.lo_bound = eval_phase_bound( MIN_GAIN ); 

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
static int update_adc_phase( // Update accumulators for calculating ADC phase
	CHECK_ADC_TYP &chk_data_s, // Reference to structure containing test check data
	ADC_PHASE_ENUM curr_phase	// current phase to update
) // Returns flag indicating if more data required for this phase
{
	ADC_TYP curr_val; // Current ADC value
	int diff_val; // Current ADC difference value
	int state_change; // flag indicating state change
	int change_id; // Index into state changes array
	int quad_id; // Index into quadrant-times array
	int quad_cnt; // quadrant counter 
	unsigned period_time; // period time for ADC Sine-wave


	state_change = 0; // Preset to NO state change

	curr_val = chk_data_s.curr_params.vals[curr_phase]; 
	diff_val = curr_val - chk_data_s.prev_params.vals[curr_phase]; 

	switch(chk_data_s.stats[curr_phase].state)
	{
		case POSI_RISE :
			if (0 > diff_val)
			{
				state_change = 1; // Set flag indicating state change detected
				chk_data_s.stats[curr_phase].state = POSI_FALL; // Update adc state					
			} // if (0 > diff_val)
		break; // POSI_RISE

		case POSI_FALL :
			if (0 > curr_val)
			{
				state_change = 1; // Set flag indicating state change detected
				chk_data_s.stats[curr_phase].state = NEGA_FALL; // Update adc state					
			} // if (0 > curr_val)
		break; // POSI_FALL

		case NEGA_FALL :
			if (0 <= diff_val)
			{
				state_change = 1; // Set flag indicating state change detected
				chk_data_s.stats[curr_phase].state = NEGA_RISE; // Update adc state					
			} // if (0 <= diff_val)
		break; // NEGA_FALL

		case NEGA_RISE :
			if (0 <= curr_val)
			{
				state_change = 1; // Set flag indicating state change detected
				chk_data_s.stats[curr_phase].state = POSI_RISE; // Update adc state					
			} // if (0 <= curr_val)
		break; // NEGA_RISE

		default :
			assert(0 == 1); // ERROR: Invalid ADC-state detected
		break; // default
	} // switch(chk_data_s.stats[curr_phase].state)

	if (state_change)
	{
		change_id = chk_data_s.stats[curr_phase].num_changes; // Get state change index

		// Skip 1st change
		if (0 < change_id)
		{ 
			quad_id = chk_data_s.stats[curr_phase].quad_off; // Get quadrant offset
			// Calculate new quadrant time
			chk_data_s.stats[curr_phase].quad_times[quad_id] = chk_data_s.curr_time - chk_data_s.stats[curr_phase].change_time;

			// Check if we have enough quadrants NB require 5 state-shanges for quadrants
			if (QUAD_MASK < change_id)
			{
				period_time = chk_data_s.stats[curr_phase].quad_times[0]; // Initialise period time with 1st quadrant time

				for (quad_cnt=1; quad_cnt<NUM_QUADS; quad_cnt++)
				{
					period_time += chk_data_s.stats[curr_phase].quad_times[quad_cnt]; // Accumulate quadrant times
				} // for quad_cnt

				chk_data_s.stats[curr_phase].period_times[change_id - NUM_QUADS] = period_time;
			} // if (QUAD_MASK < change_id)

			// Increment quadrant offset
			quad_id++;
			chk_data_s.stats[curr_phase].quad_off = (quad_id & QUAD_MASK); // Wrap into range [0..(NUM_QUADS-1)]
		} // if (0 < change_id)
		else
		{
			chk_data_s.stats[curr_phase].period_times[0] = 0; // Initialise 1st period time
		} // else !(0 < change_id)

		chk_data_s.stats[curr_phase].change_time = chk_data_s.curr_time; // Store time of state-change
		chk_data_s.stats[curr_phase].num_changes++; // Increment No of state changes detected
	} // if (state_change)

	chk_data_s.stats[curr_phase].adc_diff = diff_val; // Store for next iteration

	// Check if we have enough data for test
	if (NUM_CHANGES > chk_data_s.stats[curr_phase].num_changes)
	{
		return 1; // Set flag indicating more data required
	} // if (NUM_CHANGES > chk_data_s.stats[curr_phase].num_changes)
	else
	{
		return 0;
	} // else !(NUM_CHANGES > chk_data_s.stats[curr_phase].num_changes)
} // update_adc_phase
/*****************************************************************************/
static void update_adc_data( // Update accumulators for calculating ADC phase
	CHECK_ADC_TYP &chk_data_s // Reference to structure containing test check data
)
{
	ADC_PHASE_ENUM phase_cnt; // Counter for ADC phases
	int more_data; // Preset flag to NO more data required


	chk_data_s.more = 0; // Preset flag to NO more data required


	for (phase_cnt=0; phase_cnt<NUM_ADC_PHASES; phase_cnt++)
	{
		more_data = update_adc_phase( chk_data_s, phase_cnt );

		chk_data_s.more |= more_data; // NB Yes its another ambiguous evaluation work-around
	} // for phase_cnt


} // update_adc_data
/*****************************************************************************/
static void check_one_adc_phase( // Check one set of ADC phase data
	CHECK_ADC_TYP &chk_data_s, // Reference to structure containing test check data
	ADC_PHASE_ENUM curr_phase,	// current phase to update
	int period_id // Index into state changes array
)
{
	printint( chk_data_s.stats[curr_phase].period_times[period_id]); 
	printstr(" : ");
} // check_one_adc_phase
/*****************************************************************************/
static void check_all_adc_phase_data( // Check all ADC phase data
	CHECK_ADC_TYP &chk_data_s // Reference to structure containing test check data
)
{
	int period_cnt; // Index into state changes array
	ADC_PHASE_ENUM phase_cnt; // Counter for ADC phases


	for (period_cnt=0; period_cnt<NUM_PERIODS; period_cnt++)
	{
		acquire_lock(); // Acquire Display Mutex
		printint( period_cnt );
		printstr("= ");

		for (phase_cnt=0; phase_cnt<NUM_ADC_PHASES; phase_cnt++)
		{
			check_one_adc_phase( chk_data_s ,phase_cnt ,period_cnt ); // Check one set of ADC phase data
		} // for phase_cnt

		printcharln(' ');
		release_lock(); // Release Display Mutex
	} // for period_cnt
} // check_all_adc_phase_data
/*****************************************************************************/
static void check_adc_parameters( // Check all ADC parameters
	CHECK_ADC_TYP &chk_data_s // Reference to structure containing test check data
)
{
	check_adc_sum( chk_data_s ); // Check ADC zero-sum
	check_adc_spin_direction( chk_data_s ); // Check ADC spin direction

	update_adc_data( chk_data_s ); // Update data on ADC angular speed
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
static void initialise_one_phase_test_vector( // Initialise one set of ADC phase data
	CHECK_ADC_TYP &chk_data_s, // Reference to structure containing test check data
	ADC_PHASE_ENUM curr_phase, // current ADC phase
	ADC_STATE_ENUM init_state // initial ADC-state
)
{
	chk_data_s.stats[curr_phase].state = init_state;
	chk_data_s.stats[curr_phase].num_changes = 0;
	chk_data_s.stats[curr_phase].change_time = 0;
	chk_data_s.stats[curr_phase].adc_diff = 0;
	chk_data_s.stats[curr_phase].quad_off = 0;

} // initialise_one_phase_test_vector 
/*****************************************************************************/
static void initialise_all_phases_test_vector( // Initialise all ADC phase data
	CHECK_ADC_TYP &chk_data_s // Reference to structure containing test check data
)
{


	// Initialise ADC state for each phase
	initialise_one_phase_test_vector( chk_data_s ,ADC_PHASE_A ,POSI_RISE );
	initialise_one_phase_test_vector( chk_data_s ,ADC_PHASE_B ,NEGA_FALL );
	initialise_one_phase_test_vector( chk_data_s ,ADC_PHASE_C ,POSI_FALL );

} // initialise_all_phase_test_vector 
/*****************************************************************************/
static void finalise_phase_test_vector( // terminate ADC phase test and check results
	CHECK_ADC_TYP &chk_data_s, // Reference to structure containing test check data
	const GAIN_ADC_ENUM cur_speed	// Speed-state to finalise
)
{
	check_all_adc_phase_data( chk_data_s ); // Check all ADC phase data
} // finalise_phase_test_vector 
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
			initialise_all_phases_test_vector( chk_data_s ); // Initialise accumulators

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

			finalise_phase_test_vector( chk_data_s ,chk_data_s.curr_vect.comp_state[GAIN] ); 
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



	c_sin <: (int)TST_REQ_CMD; // Request 1st ADC value

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
