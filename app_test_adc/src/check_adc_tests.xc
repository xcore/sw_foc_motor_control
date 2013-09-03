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
static void init_check_data( // Initialise check data for ADC tests
	CHECK_TST_TYP &chk_data_s // Reference to structure containing test check data
)
{
	VECT_COMP_ENUM comp_cnt; // Counter for Test Vector components
	ADC_PHASE_ENUM phase_cnt; // Counter for ADC phases


	init_common_data( chk_data_s.common ); // Initialise data common to Generator and Checker

	safestrcpy( chk_data_s.padstr1 ,"                                             " );
	safestrcpy( chk_data_s.padstr2 ,"                              " );

	chk_data_s.skips = LO_SKIP_CHANGES; // Initialise No. of skipped state-changes while settling

	chk_data_s.print_on = VERBOSE_PRINT; // Set print mode
	chk_data_s.print_cnt = 1; // Initialise print counter
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
static void init_ang_velocity( // Initialises angular velocity
	CHECK_TST_TYP &chk_data_s // Reference to structure containing test check data
)
{
	switch( chk_data_s.curr_vect.comp_state[SPIN] )
	{
		case CLOCK: // Clock-wise
			chk_data_s.sign = 1; 
		break; // case CLOCK:

		case ANTI: // Anti-clockwise
			chk_data_s.sign = -1; 
		break; // case ANTI:

		default:
			acquire_lock(); // Acquire Display Mutex
			printstrln("ERROR: Unknown ADC Spin-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( inp_spin )

	switch( chk_data_s.curr_vect.comp_state[SPEED] )
	{
		case FAST: // Fast Speed
			chk_data_s.speed = HI_SPEED; // Set fast speed (in RPM)
		break; // case FAST:

		case SLOW: // Slow Speed
			chk_data_s.speed = LO_SPEED; // Set slow speed (in RPM)
		break; // case SLOW:

		default:
			acquire_lock(); // Acquire Display Mutex
			printstrln("ERROR: Unknown ADC Speed-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( inp_speed )

	chk_data_s.veloc = chk_data_s.speed * chk_data_s.sign;

// acquire_lock(); printstr(chk_data_s.padstr1); printstr("VEL="); printintln(chk_data_s.veloc); release_lock(); // MB~
} // init_ang_velocity
/*****************************************************************************/
static void init_gain( // Initialises ADC gain
	CHECK_TST_TYP &chk_data_s // Reference to structure containing test check data
)
{
	switch( chk_data_s.curr_vect.comp_state[GAIN] )
	{
		case SMALL: // Small amplitude
			chk_data_s.skips = HI_SKIP_CHANGES; 
			chk_data_s.chk_ampli = (MIN_GAIN * (MAX_ADC_VAL - MIN_ADC_VAL)) / 255 ; // Check value for ADC Gain
			chk_data_s.ampli_bound = (MIN_GAIN + 1); // Error Bound for ADC Gain test
			chk_data_s.mean_bound = (SMALL_BOUND * MIN_GAIN) / chk_data_s.speed;  // Error Bound for Zero-Mean test
		break; // case SMALL

		case LARGE: // large amplitude
			chk_data_s.skips = LO_SKIP_CHANGES; 
			chk_data_s.chk_ampli = (MAX_GAIN * (MAX_ADC_VAL - MIN_ADC_VAL)) / 255 ; // Check value for ADC Gain
			chk_data_s.ampli_bound = (MAX_GAIN + 1); // Error Bound for ADC Gain test
			chk_data_s.mean_bound = (LARGE_BOUND * MAX_GAIN) / chk_data_s.speed;  // Error Bound for Zero-Mean test 
		break; // case LARGE

		default:
			acquire_lock(); // Acquire Display Mutex
			printstrln("ERROR: Unknown ADC Gain-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default
	} // switch(chk_data_s.curr_vect.comp_state[GAIN])

// acquire_lock(); printstr("M_B="); printintln(chk_data_s.mean_bound); release_lock(); // MB~
	assert(1 < chk_data_s.skips); // ERROR: A minimum of 3 state-changes (2 skips) are required to define a wave period
} // init_gain
/*****************************************************************************/
static void initialise_one_phase_test_vector( // Initialise one set of ADC phase data
	CHECK_TST_TYP &chk_data_s, // Reference to structure containing test check data
	ADC_PHASE_ENUM curr_phase, // current ADC phase
	ADC_STATE_ENUM init_state // initial ADC-state
)
{
	chk_data_s.stats[curr_phase].state = init_state;
	chk_data_s.stats[curr_phase].num_changes = 0;
	chk_data_s.stats[curr_phase].sum_periods = 0; // Clear accumulator for 'period durations'
	chk_data_s.stats[curr_phase].sum_adcs = 0; // Clear accumulator for adc values
	chk_data_s.stats[curr_phase].change_time = 0; // Clear time of ADC state change
	chk_data_s.stats[curr_phase].half_period = 0; // Clear 'half of period duration'
	chk_data_s.stats[curr_phase].half_adc = 0; // Clear 'half of adc sum'
	chk_data_s.stats[curr_phase].done = 0; // Clear 'test complete' flag
	chk_data_s.stats[curr_phase].max = MIN_ADC_VAL; // Initialise to minimum possible value
	chk_data_s.stats[curr_phase].min = MAX_ADC_VAL; // Initialise to maximum possible value

} // initialise_one_phase_test_vector 
/*****************************************************************************/
static void initialise_test_vector( // Initialise all ADC phase data
	CHECK_TST_TYP &chk_data_s // Reference to structure containing test check data
)
{
	unsigned tmp_val; // temporary manipulation variable


	init_ang_velocity( chk_data_s ); // NB Do this BEFORE init_gain
	init_gain( chk_data_s );

	tmp_val = (PLATFORM_REFERENCE_HZ + (NUM_POLE_PAIRS >> 1)) / NUM_POLE_PAIRS; // Divide with rounding (25-bits)
 	tmp_val *= SECS_PER_MIN; // 31-bits
	chk_data_s.chk_period = (tmp_val + (chk_data_s.speed >> 1)) / chk_data_s.speed; // (19-bits)
	chk_data_s.period_bound = (chk_data_s.chk_period + 64) >> 6; // Set error bound at ~0.8%

// acquire_lock(); printstr("CHK="); printintln(chk_data_s.chk_period); release_lock(); // MB~

	// Initialise ADC state for each phase
	initialise_one_phase_test_vector( chk_data_s ,ADC_PHASE_A ,POSITIVE );
	initialise_one_phase_test_vector( chk_data_s ,ADC_PHASE_B ,NEGATIVE );
	initialise_one_phase_test_vector( chk_data_s ,ADC_PHASE_C ,POSITIVE );

} // initialise_test_vector 
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
static void print_adc_parameters( // Print ADC parameters
	CHECK_TST_TYP &chk_data_s // Reference to structure containing test check data
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
static void check_adc_sum( // Check all phases sum to zero
	CHECK_TST_TYP &chk_data_s // Reference to structure containing test check data
)
{
	ADC_PHASE_ENUM phase_cnt; // Counter for ADC phases
	int sum_val = 0; // Clear sum of adc values


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
} // check_adc_sum
//*****************************************************************************/
static void check_adc_gain( // Check gain of wave-train is approximately correct
	CHECK_TST_TYP &chk_data_s, // Reference to structure containing test check data
	ADC_TYP inp_max, 					// Maximum measured ADC value
	ADC_TYP inp_min 					// Minimum measured ADC value
)
{
	int amplitude = (inp_max - inp_min); // peak-to-peak amplitude
	int ampli_err; // Error in measured amplitude


	chk_data_s.motor_tsts[GAIN]++;
	ampli_err = amplitude - chk_data_s.chk_ampli; // Measurement error

	if (chk_data_s.ampli_bound < abs(ampli_err))
	{
		chk_data_s.motor_errs[GAIN]++;

		acquire_lock(); // Acquire Display Mutex
		printcharln(' ');
		printstr( chk_data_s.padstr1 );
		printstrln("ADC-Gain FAILURE");
		release_lock(); // Release Display Mutex
	} // if (chk_data_s.ampli_bound < abs(ampli_err))
} // check_adc_gain
//*****************************************************************************/
static void check_adc_mean( // Check all mean ADC value is approximately zero
	CHECK_TST_TYP &chk_data_s, // Reference to structure containing test check data
	int inp_mean 								// Input mean ADC value to test
)
{
	chk_data_s.motor_tsts[MEAN]++;

	if (chk_data_s.mean_bound < abs(inp_mean))
	{
		chk_data_s.motor_errs[MEAN]++;

		acquire_lock(); // Acquire Display Mutex
		printcharln(' ');
		printstr( chk_data_s.padstr1 );
		printstrln("Zero-Mean FAILURE");
		release_lock(); // Release Display Mutex
	} // if (0 > inp_vel)
} // check_adc_mean
/*****************************************************************************/
static void check_adc_spin_direction( // Check for correct ADC spin direction
	CHECK_TST_TYP &chk_data_s, // Reference to structure containing test check data
	ADC_PHASE_ENUM curr_phase	// current phase to update
)
// NB Difference in phase index is of opposite sign to spin
{
	int phase_diff = chk_data_s.prev_phase - curr_phase; // phase difference (sign reversed)


	chk_data_s.motor_tsts[SPIN]++;

	if (0 > phase_diff) phase_diff += NUM_ADC_PHASES;	//Check for wrap

	if (1 != phase_diff) phase_diff -= NUM_ADC_PHASES; // If NOT 1, create -1 value

	// Now we are ready to check the sign of the phase order
	if (chk_data_s.sign != phase_diff)
	{
		chk_data_s.motor_errs[SPIN]++;

		acquire_lock(); // Acquire Display Mutex
		printcharln(' ');
		printstr( chk_data_s.padstr1 );
		printstrln("Spin FAILURE");
		release_lock(); // Release Display Mutex
	} // if (chk_data_s.sign != phase_diff)

} // check_adc_spin_direction
/*****************************************************************************/
static void check_adc_period( // Check one set of ADC phase data
	CHECK_TST_TYP &chk_data_s, // Reference to structure containing test check data
	ADC_PHASE_ENUM curr_phase	// current phase to update
)
{
	unsigned mean_period; // Mean of accumulated period durations
	int period_err; // Error in measured period


	chk_data_s.motor_tsts[SPEED]++;

	// Calculate average period duration (with rounding)
	mean_period = (chk_data_s.stats[curr_phase].sum_periods + HALF_PERIODS) >> PERIOD_BITS; 

	period_err = mean_period - chk_data_s.chk_period; // Measurement error

	// Check accuracy
	if (chk_data_s.period_bound < abs(period_err))
	{
		chk_data_s.motor_errs[SPEED]++;

		acquire_lock(); // Acquire Display Mutex
		printcharln(' ');
		printstr( chk_data_s.padstr1 );
		printstrln("Speed FAILURE");
		release_lock(); // Release Display Mutex
	} // if (ADC_PERIOD < abs(period_err))

} // check_adc_period
/*****************************************************************************/
static void check_all_adc_phase_data( // Check all ADC phase data
	CHECK_TST_TYP &chk_data_s // Reference to structure containing test check data
)
{
	ADC_PHASE_ENUM phase_cnt; // Counter for ADC phases


	for (phase_cnt=0; phase_cnt<NUM_ADC_PHASES; phase_cnt++)
	{
		check_adc_period( chk_data_s ,phase_cnt ); // Check speed(period duration) for one ADC phase
	} // for phase_cnt
} // check_all_adc_phase_data
/*****************************************************************************/
static void process_state_change( // Update accumulators due to state-change, and check spin
	CHECK_TST_TYP &chk_data_s, // Reference to structure containing test check data
	ADC_PHASE_ENUM curr_phase	// current phase to update
)
{
	int period_id; // Index into array of period durations
	unsigned duration; // time difference between ADC state-changes (half period)
	int mean_val; // mean ADC value over one period


	// Calculate new half period duration
	duration = chk_data_s.curr_time - chk_data_s.stats[curr_phase].change_time;

	// Calculate trial period array index
	period_id = chk_data_s.stats[curr_phase].num_changes - chk_data_s.skips;
// acquire_lock(); printstrln(""); printstr("H="); printint(chk_data_s.stats[curr_phase].half_adc); printstr(" S="); printintln(chk_data_s.stats[curr_phase].sum_adcs); release_lock(); // MB~

	// Skip early changes while Sine-wave settles
	if (0 <= period_id)
	{ // Settled, so calculate add new period time
		// Calculate mean ADC value over previous period
		mean_val = 	chk_data_s.stats[curr_phase].half_adc + chk_data_s.stats[curr_phase].sum_adcs;

		check_adc_gain( chk_data_s ,chk_data_s.stats[curr_phase].max ,chk_data_s.stats[curr_phase].min ); // Check ADC Gain
		check_adc_mean( chk_data_s ,mean_val ); // Check ADC Mean value
		check_adc_spin_direction( chk_data_s ,curr_phase ); // Check ADC spin direction

		// Check if we have enough data for period test
		if (NUM_PERIODS > period_id)
		{
			chk_data_s.stats[curr_phase].sum_periods += (duration + chk_data_s.stats[curr_phase].half_period);
// acquire_lock(); printstrln(""); printstr("H="); printint(chk_data_s.stats[curr_phase].half_period); printstr(" T="); printintln(duration + chk_data_s.stats[curr_phase].half_period); release_lock(); // MB~

			// Check if we NOW have enough data for period test
			if ((NUM_PERIODS - 1) <= period_id)
			{
				chk_data_s.stats[curr_phase].done = 1; // Set flag indicating NO more data required
			} // if (NUM_PERIODS - 1) <= period_id)
		} // if (NUM_PERIODS > period_id)
	} // if (0 <= period_id)

	// store key data for next iteration
	chk_data_s.prev_phase = curr_phase;
	chk_data_s.stats[curr_phase].half_period = duration;
	chk_data_s.stats[curr_phase].change_time = chk_data_s.curr_time;
	chk_data_s.stats[curr_phase].half_adc = chk_data_s.stats[curr_phase].sum_adcs;
	chk_data_s.stats[curr_phase].sum_adcs = 0; // Clear accumulator ready for next half period

	chk_data_s.stats[curr_phase].num_changes++; // Increment No of state changes detected

} // process_state_change
/*****************************************************************************/
static void detect_phase_state_change( // Detect if state-change occured for current ADC phase, and check spin
	CHECK_TST_TYP &chk_data_s, // Reference to structure containing test check data
	ADC_PHASE_ENUM curr_phase	// current phase to update
)
/* A noise threshold has to be crossed before a state-change is accepted.
 * If the noise threshold is large enough, state-change jitter is prevented.
 * However, this introduces some hysteresis into the state-change cycle
 */
{
	ADC_TYP curr_val = chk_data_s.curr_params.vals[curr_phase]; // Current ADC value	


	chk_data_s.stats[curr_phase].sum_adcs += (int)curr_val; // Update ADC value accumulator

	// Check for change of state
	if (POSITIVE == chk_data_s.stats[curr_phase].state)
	{
		// Check if max value needs updating
		if (chk_data_s.stats[curr_phase].max < curr_val)
		{
			chk_data_s.stats[curr_phase].max = curr_val;
		} // if (chk_data_s.stats[curr_phase].max < curr_val)

		// Check if ADC value is sufficiently negative
		if (-NOISE_THRESH > curr_val)
		{
			process_state_change( chk_data_s ,curr_phase );

			chk_data_s.stats[curr_phase].min = curr_val; // Initialise new minimum test

			chk_data_s.stats[curr_phase].state = NEGATIVE; // Update adc state					
		} // if (0 > curr_val)
	} // if (POSITIVE == chk_data_s.stats[curr_phase].state)
	else
	{ // NEGATIVE
		// Check if min value needs updating
		if (chk_data_s.stats[curr_phase].min > curr_val)
		{
			chk_data_s.stats[curr_phase].min = curr_val;
		} // if (chk_data_s.stats[curr_phase].min < curr_val)

		// Check if ADC value is sufficiently positive
		if (NOISE_THRESH <= curr_val)
		{
			process_state_change( chk_data_s ,curr_phase );

			chk_data_s.stats[curr_phase].max = curr_val; // Initialise new maximum test

			chk_data_s.stats[curr_phase].state = POSITIVE; // Update adc state					
		} // if (0 <= curr_val)
	} // else !if (POSITIVE == chk_data_s.stats[curr_phase].state)

} // detect_phase_state_change
/*****************************************************************************/
static void update_adc_phase_data( // Updates phase data and checks spin
	CHECK_TST_TYP &chk_data_s // Reference to structure containing test check data
)
{
	ADC_PHASE_ENUM phase_cnt; // Counter for ADC phases


	chk_data_s.done = 1; // Preset flag to data collection complete


	// Loop through all phases
	for (phase_cnt=0; phase_cnt<NUM_ADC_PHASES; phase_cnt++)
	{
		detect_phase_state_change( chk_data_s, phase_cnt );

		chk_data_s.done &= chk_data_s.stats[phase_cnt].done; // Combine 'done' flags
	} // for phase_cnt

} // update_adc_phase_data
/*****************************************************************************/
static void process_new_adc_parameters( // Process new ADC parameters
	CHECK_TST_TYP &chk_data_s // Reference to structure containing test check data
)
{
	check_adc_sum( chk_data_s ); // Check ADC zero-sum

	update_adc_phase_data( chk_data_s ); // Update phase data and check spin
} // process_new_adc_parameters
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
static void get_adc_client_data( // Get next set of ADC parameters
	CHECK_TST_TYP &chk_data_s, // Reference to structure containing test check data
	streaming chanend c_gen, // Channel for communication with Test_Generator
	streaming chanend c_adc // ADC channel between Client and Server
)
{
	int diff_params = 0;	// Flag set when parameters change detected


	chk_data_s.done = 0; // Initialise Flag to 'more data required'

	// Loop until enough data collected for phase tests
	while (0 == chk_data_s.done)
	{
		c_gen :> chk_data_s.curr_time; // get time-stamp for new ADC parameters

		// To improve speed, a speculative request is issued (NB the last one will NOT be used)
		c_gen <: (int)TST_REQ_CMD; // Request next ADC value
	
		// Get new parameter values from Client function under test
		foc_adc_get_parameters( chk_data_s.curr_params ,c_adc );

#if (USE_XSCOPE)
		xscope_int( 0 ,chk_data_s.curr_params.vals[ADC_PHASE_A] );
		xscope_int( 1 ,chk_data_s.curr_params.vals[ADC_PHASE_B] );
		xscope_int( 2 ,chk_data_s.curr_params.vals[ADC_PHASE_C] );
#endif // (USE_XSCOPE)
	
		// Check for change in non-speed parameters
		diff_params = parameter_compare( chk_data_s.curr_params ,chk_data_s.prev_params ); 
	
		if (chk_data_s.print_on & diff_params )
		{
			print_adc_parameters( chk_data_s ); // Print new ADC parameters
		} // if (chk_data_s.print_on & diff_params )
	
		process_new_adc_parameters( chk_data_s ); // Process new ADC parameters
	
		chk_data_s.prev_time = chk_data_s.curr_time; // Store previous time-stamp
		chk_data_s.prev_params = chk_data_s.curr_params; // Store previous parameter values

		if (0 == chk_data_s.print_on)
		{
			print_progress( chk_data_s ); // Progress indicator
		} // if (0 == chk_data_s.print_on)

	} // while (0 == chk_data_s.done)

	// Eat last unused time-stamp (from speculative request)
	c_gen :> chk_data_s.curr_time;
	
} // get_adc_client_data
/*****************************************************************************/
static void finalise_phase_test_vector( // terminate ADC phase test and check results
	CHECK_TST_TYP &chk_data_s // Reference to structure containing test check data
)
{
	check_all_adc_phase_data( chk_data_s ); // Check all ADC phase data
} // finalise_phase_test_vector 
/*****************************************************************************/
static void check_motor_adc_client_data( // Display ADC results for one motor
	CHECK_TST_TYP &chk_data_s, // Reference to structure containing test check data
	streaming chanend c_gen, // Channel for receiving test vectors from test generator
	streaming chanend c_adc // ADC channel communication with between ADC Client
)
{
	int do_loop = 1;   // Flag set until loop-end condition found 

	acquire_lock(); // Acquire Display Mutex
	printcharln(' ');
	printstr( chk_data_s.padstr1 );
	printstr("Start Checks For Motor_"); printintln( chk_data_s.common.options.flags[TST_MOTOR] ); 
	release_lock(); // Release Display Mutex

	// Loop until end condition found
	while( do_loop )
	{
		c_gen :> chk_data_s.curr_vect; // get new test-vector

		if (chk_data_s.print_on)
		{
			print_test_vector( chk_data_s.common ,chk_data_s.curr_vect ,chk_data_s.padstr1 ); // Print new test vector details
		} // if (chk_data_s.print_on)

		// Check if testing has ended for current motor
		if (QUIT == chk_data_s.curr_vect.comp_state[CNTRL])
		{
			c_adc <: ADC_CMD_LOOP_STOP;	// Signal ADC Server to terminate

			do_loop = 0; // Error flag signals end-of-loop
		} // if (QUIT == chk_data_s.curr_vect.comp_state[CNTRL])
		else
		{ // Do new test
			c_gen <: (int)TST_REQ_CMD; // Request next ADC value
	
			initialise_test_vector( chk_data_s ); // Do Initialisation for new test

			// Get next set of ADC parameters
			get_adc_client_data( chk_data_s ,c_gen ,c_adc );

			finalise_phase_test_vector( chk_data_s ); 
		} // else !(QUIT == chk_data_s.curr_vect.comp_state[CNTRL])

		c_gen <: (int)END_TST_CMD; // Signal to test generator that this test is complete
	} // while( loop )

} // check_motor_adc_client_data
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
void check_all_adc_client_data( // Display ADC results for all motors
	streaming chanend c_adc[], // Array of channel for communication with ADC_Server
	streaming chanend c_gen // Channel for communication with Test_Generator
)
{
	CHECK_TST_TYP chk_data_s; // Structure containing test check data


	c_gen :> chk_data_s.common.options; // Get test options from generator core

	// Initialise parameter values by calling Client function
	foc_adc_get_parameters( chk_data_s.curr_params ,c_adc[chk_data_s.common.options.flags[TST_MOTOR]]  );

	init_check_data( chk_data_s ); // Initialise check data

	check_motor_adc_client_data( chk_data_s ,c_gen ,c_adc[chk_data_s.common.options.flags[TST_MOTOR]] );

	display_test_results( chk_data_s );

	acquire_lock(); // Acquire Display Mutex
	printstr( chk_data_s.padstr1 );
	printstrln( "Test Check Ends " );
	release_lock(); // Release Display Mutex

	_Exit(0); // Exit without house-keeping
} // check_all_adc_client_data
/*****************************************************************************/
