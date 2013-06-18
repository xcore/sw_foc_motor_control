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
	VECT_COMP_ENUM comp_cnt; // Counter for Test Vector components


	init_common_data( chk_data_s.common ); // Initialise data common to Generator and Checker

	safestrcpy( chk_data_s.padstr1 ,"                                             " );
	safestrcpy( chk_data_s.padstr2 ,"                              " );

	chk_data_s.print = PRINT_TST_PWM; // Set print mode
	chk_data_s.dbg = 0; // Set debug mode

	// Clear error and test counters for current motor
	for (comp_cnt=0; comp_cnt<NUM_VECT_COMPS; comp_cnt++)
	{
		chk_data_s.motor_errs[comp_cnt] = 0; 
		chk_data_s.motor_tsts[comp_cnt] = 0; 
	} // for comp_cnt
} // init_check_data
/*****************************************************************************/
static void init_wave_data( // Initialise data structure for one PWM wave
	CHECK_PWM_TYP &chk_data_s, // Reference to structure containing test check data
	PWM_WAVE_TYP &wave_data_s // input wave-data structure to be initialised
)
{
	wave_data_s.hi_time = 0;
	wave_data_s.lo_time = 0;
	wave_data_s.hi_sum = 0;
	wave_data_s.lo_sum = 0;
	wave_data_s.hi_num = 0;
	wave_data_s.lo_num = 0;

	wave_data_s.curr_data.first = 0;
	wave_data_s.curr_data.last = 0;
	wave_data_s.curr_data.port_data.pattern = 0;
	wave_data_s.curr_data.port_data.time_off = 0;

	wave_data_s.prev_data = wave_data_s.curr_data; // Initialise previous PWM value
} // init_one_wave
/*****************************************************************************/
static void print_pwm_pulse( // Print PWM Pulse data
	CHECK_PWM_TYP &chk_data_s, // Reference to structure containing test check data
	unsigned pulse_width, // Width of input pulse
	const char pulse_name[] // name for input pulse
)
{
	acquire_lock(); // Acquire Display Mutex
	printstr( chk_data_s.padstr1 );
	printstr(pulse_name);
	printstr( "= " );
	printuint( pulse_width );
	printcharln(' ');
	release_lock(); // Release Display Mutex
} // print_pwm_pulse
/*****************************************************************************/
static void print_pwm_sample( // Print PWM parameters
	CHECK_PWM_TYP &chk_data_s, // Reference to structure containing test check data
	PWM_WAVE_TYP &wave_data_s // Reference to wave-data structure to be printed
)
{
	acquire_lock(); // Acquire Display Mutex
	printstr( chk_data_s.padstr1 );
	printstr( "  X=" );
	printhex( wave_data_s.curr_data.port_data.pattern );
	printstr( "  T=" );
	printint( wave_data_s.curr_data.port_data.time_off );
	printcharln(' ');
	release_lock(); // Release Display Mutex
} // print_pwm_sample
/*****************************************************************************/
static int find_edge_offset_from_ls( // Finds first bit-change in 32-bit pattern (measured from LS end)
	unsigned inp_patn  // input 32-bit pattern
) // Return bit offset
{
	int bit_off = 0; // Initialise bit offset to LS-end of pattern
	unsigned ls_bit = (0x1 & inp_patn); // Get LS bit value from input pattern


	// loop until bit-change found
	while (inp_patn)
	{
		bit_off++; // Increment bit-offset
		inp_patn >>= 1; // shift out checked-bit

		if (ls_bit != (0x1 & inp_patn)) break; // Exit loop if bit-change found
	} // while (inp_patn)

	// Check if edge found
	if (PWM_PORT_WID <= bit_off)
	{
		assert(0 == 1); // ERROR: Edge not found
	} // if (PWM_PORT_WID <= bit_off)

	return bit_off;
} // find_edge_offset_from_ls
/*****************************************************************************/
static void init_sample_data( // Initialise PWM sample data, and if necessary correct edge-time
	PWM_SAMP_TYP &curr_pwm_s, // Reference to structure containing current data for one PWM sample
	PWM_SAMP_TYP &prev_pwm_s // Reference to structure containing previous data for one PWM sample
)
{
	unsigned curr_patn = curr_pwm_s.port_data.pattern; // get current PWM sample pattern


	curr_pwm_s.first = (0x1 & curr_patn); // mask-out first (LS) bit
	curr_pwm_s.last = (PWM_MS_MASK & curr_patn) >> (PWM_PORT_WID - 1); // mask-out last (MS) bit

	// Rebase time-offset to just before first (LS) bit
	curr_pwm_s.port_data.time_off -= PWM_PORT_WID; // Correct time_offset

	// Check for edge ...

	/* NB The following logic relies on the fact that some combinations are unreachable,
	 * because the minimum pulse width is PWM_PORT_WID (32) 
	 */
	if (curr_pwm_s.first != curr_pwm_s.last)
	{ // Edge found
		PORT_TIME_TYP bit_off = (PORT_TIME_TYP)find_edge_offset_from_ls( curr_patn ); // Get edge offset
		curr_pwm_s.port_data.time_off += bit_off; // Place time-offset at edge
	} // if (curr_pwm_s.first != curr_pwm_s.last)

} // init_sample_data
/*****************************************************************************/
static void check_adc_trigger( // Check timing of ADC trigger
	CHECK_PWM_TYP &chk_data_s, // Reference to structure containing test check data
	PWM_WAVE_TYP &wave_data_s, // Reference to wave-data structure to be updated
	PORT_TIME_TYP curr_time,		// current PWM time-offset
	const ADC_PWM_ENUM curr_adc	// ADC-trigger state to check
)
{
	PORT_TIME_TYP cent_time; // port time for centre of High-Pulse
	int err_time; // ADC error time


	// Calculate ADC trigger error. (distance from centre of High-pulse)
	cent_time = curr_time + (PWM_MAX_VALUE - (wave_data_s.hi_time >> 1)); // time_offset for centre of High-Pulse
	cent_time &= PWM_MASK; // mask into range [0..PWM_MASK]
	err_time = (wave_data_s.adc_time + QUART_PWM_MAX - ADC_DELAY) + (PWM_MAX_VALUE - cent_time); // error for ADC trigger;
	err_time &= PWM_MASK; // mask into range [0..PWM_MASK]
	if (err_time > (PWM_MAX_VALUE >> 1)) err_time -= PWM_MAX_VALUE; // Max. absolute error is half PWM period 

	chk_data_s.motor_tsts[ADC_TRIG]++;

	if (HALF_PORT_WID <= abs(err_time))
	{
		chk_data_s.motor_errs[ADC_TRIG]++;

		acquire_lock(); // Acquire Display Mutex
		printcharln(' ');
		printstr( chk_data_s.padstr1 );
		printstr( chk_data_s.common.comp_data[ADC_TRIG].state_names[curr_adc].str	);
		printstr(" FAILURE: Error_Cycles=");
		printintln(err_time);
		release_lock(); // Release Display Mutex
	} // if (HALF_PORT_WID <= abs(err_time))

} // check_adc_trigger
/*****************************************************************************/
static void update_pwm_data( // Update PWM data
	CHECK_PWM_TYP &chk_data_s, // Reference to structure containing test check data
	PWM_WAVE_TYP &wave_data_s // Reference to wave-data structure to be updated
)
{
	PWM_SAMP_TYP curr_pwm_s = wave_data_s.curr_data; // Reference to structure containing current PWM sample data
	PWM_SAMP_TYP prev_pwm_s = wave_data_s.prev_data; // Reference to structure containing previous PWM sample data
	PORT_TIME_TYP curr_time = (PORT_TIME_TYP)curr_pwm_s.port_data.time_off; // current PWM time-offset
	PORT_TIME_TYP prev_time = (PORT_TIME_TYP)prev_pwm_s.port_data.time_off; // previous PWM time-offset
	PORT_TIME_TYP diff_time = (curr_time - prev_time); // Elapsed time


	// Update pulse times ...

	/* NB The following logic relies on the fact that some combinations are unreachable,
	 * because the minimum pulse width is PWM_PORT_WID (32) 
	 */

	if (curr_pwm_s.last)
	{ // Currently High
		if (prev_pwm_s.last)
		{ // Constant High
			wave_data_s.hi_time +=  diff_time; // Update time for high portion of pulse
		} // if (prev_pwm_s.last)
		else
		{ // Rising edge
			wave_data_s.lo_time +=  diff_time; // Finalise time for low portion of pulse
			wave_data_s.hi_time = 0; // Initialise time for high portion of pulse
			wave_data_s.lo_sum += wave_data_s.lo_time;
			wave_data_s.lo_num++;

			if (chk_data_s.print)
			{
				print_pwm_pulse( chk_data_s ,wave_data_s.lo_time ," Low_Width" ); // Print new PWM pulse data
			} // if (chk_data_s.print)
		} // else !(prev_pwm_s.last)
	} // if (curr_pwm_s.last)
	else
	{ // Currently Low
		if (prev_pwm_s.last)
		{ // Falling Edge
			wave_data_s.hi_time += diff_time; // Finalise time for high portion of pulse
			wave_data_s.lo_time = 0; // Initialise time for low portion of pulse
			wave_data_s.hi_sum += wave_data_s.hi_time;
			wave_data_s.hi_num++;

			// Check if ADC-trigger being tested
			if (chk_data_s.common.options.flags[TST_ADC])
			{ 
				check_adc_trigger( chk_data_s ,wave_data_s ,curr_time ,chk_data_s.curr_vect.comp_state[ADC_TRIG] );
			} // if (chk_data_s.common.options.flags[TST_ADC])

			if (chk_data_s.print)
			{
				print_pwm_pulse( chk_data_s ,wave_data_s.hi_time ,"High_Width" ); // Print new PWM pulse data
			} // if (chk_data_s.print)
		} // if (prev_pwm_s.last)
		else
		{ // Constant Low
			wave_data_s.lo_time += diff_time; // Update time for low portion of pulse
		} // else !(prev_pwm_s.last)
	} // else !(curr_pwm_s.last)
} // update_pwm_data
/*****************************************************************************/
static void check_pwm_parameters( // Check all PWM parameters
	CHECK_PWM_TYP &chk_data_s, // Reference to structure containing test check data
	PWM_WAVE_TYP &wave_data_s // Reference to wave-data structure to be checked
)
{

} // check_pwm_parameters
/*****************************************************************************/
static void check_pwm_pulse_levels( // Check PWM mean voltage
	CHECK_PWM_TYP &chk_data_s, // Reference to structure containing test check data
	PWM_WAVE_TYP &wave_data_s, // Reference to wave-data structure containing mean to be evaluated
	const WIDTH_PWM_ENUM cur_width	// PWM-width state to check
)
{
	unsigned chk_wid = chk_data_s.common.pwm_wids[cur_width]; // Get Check widths for this width-state 


	chk_data_s.motor_tsts[WIDTH]++;

	if (chk_data_s.bound < abs(wave_data_s.meas_wid - chk_wid))
	{
		chk_data_s.motor_errs[WIDTH]++;

		acquire_lock(); // Acquire Display Mutex
		printcharln(' ');
		printstr( chk_data_s.padstr1 );
		printstr( chk_data_s.common.comp_data[WIDTH].state_names[cur_width].str	);
		printstrln(" FAILURE");
		release_lock(); // Release Display Mutex
	} // if (chk_data_s.hi_bound < abs(wave_data_s.meas_wid - MAX_PWM))
} // check_pwm_pulse_levels
/*****************************************************************************/
static void measure_pwm_width( // Calculate PWM-width from captured PWM wave data
	CHECK_PWM_TYP &chk_data_s, // Reference to structure containing test check data
	PWM_WAVE_TYP &wave_data_s // Reference to wave-data structure containing wave_data_s.meas_wid to be evaluated
)
{
	int hi_mean = 0; // Clear Mean of time PWM was high
	int lo_mean = 0; // Clear Mean of time PWM was low
	int period = 0; // Clear Mean PWM wave period


	wave_data_s.meas_wid = 0; // Clear Mean PWM wave period

	// Calculate Mean high-level time
	if (wave_data_s.hi_num)
	{
		hi_mean = (wave_data_s.hi_sum + (wave_data_s.hi_num >> 1)) / wave_data_s.hi_num;
	} // if (wave_data_s.hi_num)

	// Calculate Mean high-level time
	if (wave_data_s.lo_num)
	{
		lo_mean = (wave_data_s.lo_sum + (wave_data_s.lo_num >> 1)) / wave_data_s.lo_num;
	} // if (wave_data_s.lo_num)

	period = hi_mean + lo_mean;	// Calculate mean PWM wave period

	// Calculate equivalent measured PWM-width
	if (period) 
	{
		wave_data_s.meas_wid = ((PWM_MAX_VALUE * hi_mean) + (period >> 1))/period;
	} // if (period) 

} // measure_pwm_width
/*****************************************************************************/
static void initialise_pwm_width_test( // Initialise data for new Pulse-width test
	CHECK_PWM_TYP &chk_data_s, // Reference to structure containing test check data
	PWM_WAVE_TYP &wave_data_s // Reference to wave-data structure to be initialise
)
{
	// Clear accumulated data
	init_wave_data( chk_data_s ,wave_data_s ); // Initialise data for one PWM wave

	// Evaluate error bounds for Pulse-width checks (~2%)
	chk_data_s.bound = 1 + (chk_data_s.common.pwm_wids[chk_data_s.curr_vect.comp_state[WIDTH]] >> 6); 

} // initialise_pwm_width_test 
/*****************************************************************************/
static void finalise_pwm_width_test( // terminate pulse-width test and check results
	CHECK_PWM_TYP &chk_data_s, // Reference to structure containing test check data
	PWM_WAVE_TYP &wave_data_s, // Reference to wave-data structure to be finalised
	const WIDTH_PWM_ENUM cur_width	// PWM-width state to check
)
{
	measure_pwm_width( chk_data_s ,wave_data_s );

	if (chk_data_s.print)
	{
		acquire_lock(); // Acquire Display Mutex
		printstr( chk_data_s.padstr1 );
		printstr( "Mean_Width= " );
		printuint(wave_data_s.meas_wid);
		printcharln(' ');
		release_lock(); // Release Display Mutex
	} // if (chk_data_s.print)

	check_pwm_pulse_levels( chk_data_s ,wave_data_s ,cur_width ); // Check PWM pulse width

} // finalise_pwm_width_test 
/*****************************************************************************/
static int pwm_data_compare( // Check if 2 sets of PWM data are different
	PWM_PORT_TYP &data_a,	// Structure containing 1st set of PWM parameters
	PWM_PORT_TYP &data_b	// Structure containing 2nd set of PWM parameters
) // return TRUE (1) if sets are different, FALSE(0) if equal
{
	if (data_a.pattern != data_b.pattern) return 1;	// Check Pattern value

	if (data_a.time_off != data_b.time_off) return 1;	// Check time offset

	return 0; // No differences found
} // pwm_data_compare
/*****************************************************************************/
static void test_new_pwm_input_value( // test new PWM input value
	CHECK_PWM_TYP &chk_data_s, // Reference to structure containing test check data
	PWM_WAVE_TYP &wave_data_s // Reference to wave-data structure to be tested
)
{
	int do_test = 0;	// Flag set when next test required


	// Check for change in PWM data
	do_test = pwm_data_compare( wave_data_s.curr_data.port_data  ,wave_data_s.prev_data.port_data ); 

	// Check for parameter change
	if (do_test)
	{ // Parameters changed

		if (chk_data_s.print)
		{
//			print_pwm_sample( chk_data_s ,wave_data_s ); // Print new PWM sample
		} // if (chk_data_s.print)

		init_sample_data( wave_data_s.curr_data ,wave_data_s.prev_data ); 

		// Check if this current test vector is valid
		if (VALID == chk_data_s.curr_vect.comp_state[CNTRL])
		{
			update_pwm_data( chk_data_s ,wave_data_s ); // Update PWM data

			check_pwm_parameters( chk_data_s ,wave_data_s ); // Check all PWM parameters
		} // if (VALID == chk_data_s.curr_vect.comp_state[CNTRL])

		// Store previous PWM data
		wave_data_s.prev_data = wave_data_s.curr_data;
	} // if (do_test)
} // test_new_pwm_input_value
/*****************************************************************************/
static void process_new_test_vector( // Process new test vector
	CHECK_PWM_TYP &chk_data_s, // Reference to structure containing test check data
	PWM_WAVE_TYP &wave_data_s // Reference to wave-data structure under test
)
{
	int change = 0; // Clear flag indicating change in test vector detected


	// Check for change in Pulse-width
	if (chk_data_s.curr_vect.comp_state[WIDTH] != chk_data_s.prev_vect.comp_state[WIDTH])
	{
		finalise_pwm_width_test( chk_data_s ,wave_data_s ,chk_data_s.prev_vect.comp_state[WIDTH] );

		initialise_pwm_width_test( chk_data_s ,wave_data_s );

		change = 1; // Set flag indicating change in test vector detected
	} // if ((chk_data_s.curr_vect.comp_state[WIDTH] ...

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
void check_pwm_client_data( // Display PWM results for all motors
	streaming chanend c_chk[], // Array of Channels for receiving PWM data
	streaming chanend c_tst // Channel for receiving test vectors from test generator
)
{
	CHECK_PWM_TYP chk_data_s; // Structure containing test check data
	PWM_WAVE_TYP wave_data_s; // A wave-data structure for one PWM leg and one phase
	PWM_PORT_TYP port_data_bufs[NUM_INP_BUFS]; // Set of buffers for PWM widths for each PWM-leg
	int comp_cnt; // Counter for Test Vector components
	int do_loop = 1;   // Flag set until loop-end condition found 
	int motor_errs = 0;   // Preset flag to NO errors for current motor
	int motor_tsts = 0;   // Clear test ccounter for current motor

	int write_cnt = 0; // No of buffer writes
	int read_cnt = 0; // No of buffer reads
	int chan_cnt = 0; // No of channel reads
	unsigned write_off = 0; // buffer write offset
	unsigned read_off = 0; // buffer read offset
	unsigned chan_off = 0; // channel offset


	init_check_data( chk_data_s ); // Initialise check data

	acquire_lock(); // Acquire Display Mutex
	printcharln(' ');
	printstr( chk_data_s.padstr1 );
	printstrln("Start Checks"); 
	release_lock(); // Release Display Mutex


	c_tst :> chk_data_s.common.options; // Get test options from generator core

	c_tst :> chk_data_s.curr_vect; // Initialise test-vector structure with 1st test

	init_wave_data( chk_data_s ,wave_data_s ); // Initialise wave data

	// special case: initialisation for first test
  chk_data_s.prev_vect = chk_data_s.curr_vect;

	if (chk_data_s.print)
	{
		print_test_vector( chk_data_s.common ,chk_data_s.curr_vect ,chk_data_s.padstr1 ); // Print new test vector details
	} // if (chk_data_s.print)

	initialise_pwm_width_test( chk_data_s ,wave_data_s ); 

	while (do_loop) {
#pragma ordered // If multiple cases fire at same time, service top-most first
		select {
			// Service any new PWM data on input channel
			case c_chk[chan_off] :> port_data_bufs[write_off] :
// acquire_lock(); printstr("                "); printuint(write_off); printstr("P="); printhexln(port_data_bufs[write_off].pattern); release_lock(); //MB~

				// Update circular buffer offsets
				write_cnt++; // Increment write counter
				write_off = (((unsigned)write_cnt) & INP_BUF_MASK); // Wrap offset into range [0..INP_BUF_MASK];

				chan_cnt++; // Increment channe lcounter
				chan_off = (((unsigned)chan_cnt) & CHAN_MASK); // Wrap offset into range [0..CHAN_MASK];
			break;

			// Service any change on test channel
			case c_tst :> chk_data_s.curr_vect :
				// New test vector detected.
				process_new_test_vector( chk_data_s ,wave_data_s ); // Process new test vector

				// Check if testing has ended for current motor
				if (QUIT == chk_data_s.curr_vect.comp_state[CNTRL])
				{
					do_loop = 0; // Error flag signals end-of-loop
				} // if (QUIT == chk_data_s.curr_vect.comp_state[CNTRL])
			break; // c_tst 

			default:
				assert(write_cnt <= (read_cnt + NUM_INP_BUFS)); // Check we have enough input buffers
	
				// Check if any new data to read
				if (write_cnt > read_cnt)
				{
					wave_data_s.curr_data.port_data = port_data_bufs[read_off]; // Update PWM data
		
					// Update circular buffer offset
					read_cnt++; // Increment write counter
					read_off = (((unsigned)read_cnt) & INP_BUF_MASK); // Wrap offset into range [0..INP_BUF_MASK];

					// Check for special case of ADC trigger		
					if (ADC_PATN != wave_data_s.curr_data.port_data.pattern)
					{ 
						// test new PWM data for current wave
						test_new_pwm_input_value( chk_data_s ,wave_data_s );
					} // if (ADC_PATN != wave_data_s.curr_data.port_data.pattern)
					else
					{ // Update ADC trigger time
						wave_data_s.adc_time = wave_data_s.curr_data.port_data.time_off;
					} // if (ADC_PATN != wave_data_s.curr_data.port_data.pattern)
				} // if (write_cnt > read_cnt)
			break; // default
		} // select
	}	// while (do_loop)

	// special case: finalisation for last pulse-width test
	finalise_pwm_width_test( chk_data_s ,wave_data_s ,chk_data_s.curr_vect.comp_state[WIDTH] ); 

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
} // check_pwm_client_data
/*****************************************************************************/
