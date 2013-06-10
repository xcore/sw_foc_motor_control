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
	PWM_LEG_ENUM leg_cnt; // Counter for PWM_legs


	init_common_data( chk_data_s.common ); // Initialise data common to Generator and Checker

	safestrcpy( chk_data_s.padstr1 ,"                                             " );
	safestrcpy( chk_data_s.padstr2 ,"                              " );

	chk_data_s.hi_sum = 0;
	chk_data_s.lo_sum = 0;
	chk_data_s.hi_num = 0;
	chk_data_s.lo_num = 0;

	chk_data_s.fail_cnt = 0; // Clear count of failed tests.

	chk_data_s.print = PRINT_TST_PWM; // Set print mode
	chk_data_s.dbg = 0; // Set debug mode

	// Clear error and test counters for current motor
	for (leg_cnt=0; leg_cnt<NUM_PWM_LEGS; leg_cnt++)
	{
		chk_data_s.legs[leg_cnt].hi_time = 0; 
		chk_data_s.legs[leg_cnt].lo_time = 0; 

		chk_data_s.legs[leg_cnt].curr_data.leg_id = leg_cnt;
		chk_data_s.legs[leg_cnt].curr_data.first = 0;
		chk_data_s.legs[leg_cnt].curr_data.last = 0;
		chk_data_s.legs[leg_cnt].curr_data.port_data.pattern = 0;
		chk_data_s.legs[leg_cnt].curr_data.port_data.time_off = 0;
		chk_data_s.legs[leg_cnt].prev_data = chk_data_s.legs[leg_cnt].curr_data; // Initialise previous PWM value
	} // for leg_cnt

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
	printstr( "  X=" );
	printhex( chk_data_s.legs[chk_data_s.curr_leg].curr_data.port_data.pattern );
	printstr( "  T=" );
	printint( chk_data_s.legs[chk_data_s.curr_leg].curr_data.port_data.time_off );
	printstr( "  D=" );
	printint( (PORT_TIME_TYP)(chk_data_s.legs[chk_data_s.curr_leg].curr_data.port_data.time_off 
		- chk_data_s.legs[chk_data_s.curr_leg].prev_data.port_data.time_off) );
	printcharln(' ');
	release_lock(); // Release Display Mutex
} // print_pwm_parameters
/*****************************************************************************/
static void check_pwm_width( // Check for correct PWM width
	CHECK_PWM_TYP &chk_data_s // Reference to structure containing test check data
)
{
	unsigned curr_patn = chk_data_s.legs[chk_data_s.curr_leg].curr_data.port_data.pattern; // local copy of current PWM pattern


	chk_data_s.motor_tsts[WIDTH]++;

	// Check for expected value
	if (curr_patn == chk_data_s.wid_chk)
	{ // Found expected value (Test passed)
		chk_data_s.wid_cnt = 0; // Clear Time-out
	} // if (curr_patn == chk_data_s.wid_chk)
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
	} // else !(curr_patn == chk_data_s.wid_chk)

} // check_pwm_width
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
	PWM_DATA_TYP &curr_pwm_s, // Reference to structure containing current data for one PWM sample
	PWM_DATA_TYP &prev_pwm_s // Reference to structure containing previous data for one PWM sample
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
static void update_pwm_data( // Update PWM data
	CHECK_PWM_TYP &chk_data_s // Reference to structure containing test check data
)
{
	PWM_DATA_TYP curr_pwm_s = chk_data_s.legs[chk_data_s.curr_leg].curr_data; // Reference to structure containing current PWM sample data
	PWM_DATA_TYP prev_pwm_s = chk_data_s.legs[chk_data_s.curr_leg].prev_data; // Reference to structure containing previous PWM sample data
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
			chk_data_s.legs[chk_data_s.curr_leg].hi_time +=  diff_time; // Update time for high portion of pulse
		} // if (prev_pwm_s.last)
		else
		{ // Rising edge
			chk_data_s.legs[chk_data_s.curr_leg].lo_time +=  diff_time; // Finalise time for low portion of pulse
			chk_data_s.legs[chk_data_s.curr_leg].hi_time = 0; // Initialise time for high portion of pulse
acquire_lock();	printstr(chk_data_s.padstr1); printstr("LT="); printuintln(chk_data_s.legs[chk_data_s.curr_leg].lo_time);	release_lock();
			chk_data_s.lo_sum += chk_data_s.legs[chk_data_s.curr_leg].lo_time;
			chk_data_s.lo_num++;
		} // else !(prev_pwm_s.last)
	} // if (curr_pwm_s.last)
	else
	{ // Currently Low
		if (prev_pwm_s.last)
		{ // Falling Edge
			chk_data_s.legs[chk_data_s.curr_leg].hi_time += diff_time; // Finalise time for high portion of pulse
			chk_data_s.legs[chk_data_s.curr_leg].lo_time = 0; // Initialise time for low portion of pulse
acquire_lock();	printstr(chk_data_s.padstr1); printstr("HT="); printuintln(chk_data_s.legs[chk_data_s.curr_leg].hi_time);	release_lock();
			chk_data_s.hi_sum += chk_data_s.legs[chk_data_s.curr_leg].hi_time;
			chk_data_s.hi_num++;
		} // if (prev_pwm_s.last)
		else
		{ // Constant Low
			chk_data_s.legs[chk_data_s.curr_leg].lo_time += diff_time; // Update time for low portion of pulse
		} // else !(prev_pwm_s.last)
	} // else !(curr_pwm_s.last)
} // update_pwm_data
/*****************************************************************************/
static void check_pwm_parameters( // Check all PWM parameters
	CHECK_PWM_TYP &chk_data_s // Reference to structure containing test check data
)
{
	check_pwm_width( chk_data_s ); // Check PWM width
} // check_pwm_parameters
/*****************************************************************************/
static int pwm_data_compare( // Check if 2 sets of PWM data are different
	PWM_PORT_TYP &data_a,	// Structure containing 1st set of QEI parameters
	PWM_PORT_TYP &data_b	// Structure containing 2nd set of QEI parameters
) // return TRUE (1) if sets are different, FALSE(0) if equal
{
	if (data_a.pattern != data_b.pattern) return 1;	// Check Pattern value

	if (data_a.time_off != data_b.time_off) return 1;	// Check time offset

	return 0; // No differences found
} // pwm_data_compare
/*****************************************************************************/
static void test_new_pwm_input_value( // test new PWM input value
	CHECK_PWM_TYP &chk_data_s // Reference to structure containing test check data
)
{
	int do_test = 0;	// Flag set when next test required


	// Check for change in PWM data
	do_test = pwm_data_compare( chk_data_s.legs[chk_data_s.curr_leg].curr_data.port_data  ,chk_data_s.legs[chk_data_s.curr_leg].prev_data.port_data  ); 

	// Check for parameter change
	if (do_test)
	{ // Parameters changed

		if (chk_data_s.print)
		{
			print_pwm_parameters( chk_data_s ); // Print new PWM parameters
		} // if (chk_data_s.print)

		init_sample_data( chk_data_s.legs[chk_data_s.curr_leg].curr_data ,chk_data_s.legs[chk_data_s.curr_leg].prev_data ); 

		update_pwm_data( chk_data_s ); // Update PWM data

		// Check if this current test vector is valid
		if (VALID == chk_data_s.curr_vect.comp_state[CNTRL])
		{
			check_pwm_parameters( chk_data_s ); // Check all PWM parameters
		} // if (VALID == chk_data_s.curr_vect.comp_state[CNTRL])

		// Store previous PWM data
		chk_data_s.legs[chk_data_s.curr_leg].prev_data = chk_data_s.legs[chk_data_s.curr_leg].curr_data;
	} // if (do_test)
} // test_new_pwm_input_value
/*****************************************************************************/
static void eval_mean(
	CHECK_PWM_TYP &chk_data_s // Reference to structure containing test check data
)
{
	int hi_mean = 0;
	int lo_mean = 0;
	int wid = 0;


	if (chk_data_s.hi_num) hi_mean = (chk_data_s.hi_sum + (chk_data_s.hi_num >> 1)) / chk_data_s.hi_num;
	if (chk_data_s.lo_num) lo_mean = (chk_data_s.lo_sum + (chk_data_s.lo_num >> 1)) / chk_data_s.lo_num;

	wid = hi_mean + lo_mean;

	if (wid) wid = ((PWM_MAX_VALUE * hi_mean) + (wid >> 1))/wid;

	acquire_lock(); printstr("WID="); printuint(wid); release_lock(); //MB~
} // eval_mean
/*****************************************************************************/
static void process_new_test_vector( // Process new test vector
	CHECK_PWM_TYP &chk_data_s // Reference to structure containing test check data
)
{
	int change = 0; // Clear flag indicating change in test vector detected

	eval_mean( chk_data_s );

	chk_data_s.hi_sum = 0;
	chk_data_s.lo_sum = 0;
	chk_data_s.hi_num = 0;
	chk_data_s.lo_num = 0;

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
	PWM_DATA_TYP port_data_bufs[NUM_INP_BUFS]; // Set of buffers for PWM widths for each PWM-leg
	timer chronometer; // XMOS timer
	unsigned write_off; // buffer write offset
	unsigned read_off; // buffer read offset
	unsigned circ_off = 0; // Circular offset between Write and Read and offsets
	unsigned prev_off; // previous circular offset
	unsigned hi_pins = 0x55555555; // Initialise High-leg input pins to impossible value
//MB~	unsigned lo_pins = 0x55555555; // Initialise Low-leg input pins to impossible value
	int comp_cnt; // Counter for Test Vector components
	int do_loop = 1;   // Flag set until loop-end condition found 
	int motor_errs = 0;   // Preset flag to NO errors for current motor
	int motor_tsts = 0;   // Clear test ccounter for current motor
	unsigned char cntrl_token; // control token
	PORT_TIME_TYP port_time; // Time when port read


	init_check_data( chk_data_s ); // Initialise check data


//	chronometer :> chk_data_s.time_off[0]; // Get start time
//	chronometer when timerafter(chk_data_s.time_off[0] + (MICRO_SEC << 1)) :> chk_data_s.time_off[0]; // Wait for Test Generation to Start

	write_off = 0;
	read_off= 0;
	prev_off = 0;

	acquire_lock(); // Acquire Display Mutex
	printcharln(' ');
	printstr( chk_data_s.padstr1 );
	printstrln("Start Checks"); 
	release_lock(); // Release Display Mutex

	c_tst :> chk_data_s.curr_vect; // Initialise test-vector structure with 1st test

	// special case: initialisation for first test
  chk_data_s.prev_vect = chk_data_s.curr_vect;

	if (chk_data_s.print)
	{
		print_test_vector( chk_data_s.common ,chk_data_s.curr_vect ,chk_data_s.padstr1 ); // Print new test vector details
	} // if (chk_data_s.print)

	while (do_loop) {
#pragma ordered // If multiple cases fire at same time, service top-most first
		select {
			// Service any change on High-leg input port pins
			case p32_tst_hi[chk_data_s.curr_vect.comp_state[PHASE]] when pinsneq(hi_pins):> hi_pins @ port_time :
			{
//				chronometer :> port_data_bufs[write_off].port_data.time_off; // Get new time stamp as soon as possible
				port_data_bufs[write_off].port_data.time_off = port_time;
				port_data_bufs[write_off].port_data.pattern = hi_pins;
				port_data_bufs[write_off].leg_id = HI_LEG;
// acquire_lock(); printstr("T="); printuint(port_data_bufs[write_off].port_data.time_off); printstr(" P="); printhexln(hi_pins); release_lock(); //MB~

				// Update circular buffer offset
				if (write_off < BUF_MASK)
				{
					write_off++;
				} // if (write_off < BUF_MASK)
				else
				{
					write_off = 0;
				} // else !(write_off < BUF_MASK)
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
				circ_off = write_off - read_off; // Difference between write & read offsets
				circ_off &= BUF_MASK; // Wrap offset into range [0..BUF_MASK];

				assert(circ_off < NUM_INP_BUFS); // Check we have enough input buffers
	
				// Check if any new data to read
				if (0 != circ_off)
				{
					chk_data_s.curr_leg = port_data_bufs[read_off].leg_id; // Identifier which PWM-leg is next to be processed
					chk_data_s.legs[chk_data_s.curr_leg].curr_data = port_data_bufs[read_off]; // Update data for current PWM-leg

					// Update circular buffer offset
					if (read_off < BUF_MASK)
					{
						read_off++;
					} // if (read_off < BUF_MASK)
					else
					{
						read_off = 0;
					} // else !(read_off < BUF_MASK)

					test_new_pwm_input_value( chk_data_s ); // test new PWM data
				} // if (0 != circ_off)
				else
				{
					assert(2 > prev_off); // Check we have NOT overflowed input buffer
				} // else !(0 != circ_off)

				prev_off = circ_off; // Store circular offset				
			break; // default
		} // select
	}	// while (do_loop)

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
