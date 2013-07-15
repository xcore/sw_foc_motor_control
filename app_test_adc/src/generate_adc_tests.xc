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

#include "generate_adc_tests.h"

/*****************************************************************************/
static void parse_control_file( // Parse ADC control file and set up test options
	GENERATE_TST_TYP &gen_data_s // Reference to structure of ADC test data
)
{
  unsigned char file_buf[FILE_SIZE];
  int char_cnt; // Character counter
  int line_cnt = 0; // line counter
  int test_cnt = 0; // test counter
  int new_line = 0; // flag if new-line found
  int tst_line = 0; // flag set if test-option found
  char curr_char; // Current character
  int file_id; // File identifier
  int status = 0; // Error status


	// Initialise file buffer
  for (char_cnt = 0; char_cnt < FILE_SIZE; ++char_cnt) 
	{
    file_buf[char_cnt] = 0;
  } // for char_cnt

  file_id = _open( "adc_tests.txt" ,O_RDONLY ,0 ); // Open control file for ADC tests

  assert(-1 != file_id); // ERROR: Open file failed (_open)

  _read( file_id ,file_buf ,FILE_SIZE );  // Read file into buffer

  status = _close(file_id);	// Close file
  assert(0 == status);	// ERROR: Close file failed (_close)

	acquire_lock(); // Acquire Display Mutex
	printcharln(' ');
	printstrln("Read following Test Options ..." );

	// Parse the file buffer for test options
  for (char_cnt = 0; char_cnt < FILE_SIZE; ++char_cnt) 
	{
    curr_char = file_buf[char_cnt]; // Get next character

    if (!curr_char) break; // Check for end of file info.


		switch (curr_char)
		{
    	case '0' : // Opt out of test
				gen_data_s.common.options.flags[test_cnt] = 0;
				tst_line = 1; // Flag test-option found
    	break; // case '0' :

    	case '1' : // Opt for this test
				gen_data_s.common.options.flags[test_cnt] = 1;
				tst_line = 1; // Flag test-option found
    	break; // case '1' :

    	case '#' : // Start of comment
				new_line = 1; // Set flag for new-line
    	break; // case '1' :

    	case '\n' : // End of line.
				new_line = 1; // Set flag for new-line
    	break; // case '\n' :

    	default : // Whitespace
				// Un-determined line
    	break; // case '\n' :
		} // switch (curr_char)

		// Check if we have a test-option line
		if (tst_line)
		{ // Process test-option line
			test_cnt++;
			printchar(curr_char);
			printchar(' ');

			while ('\n' != file_buf[char_cnt])
			{
				char_cnt++;
				assert(char_cnt < FILE_SIZE); // End-of-file found
				printchar( file_buf[char_cnt] );
			} // while ('\n' != file_buf[char_cnt])

			line_cnt++;
			tst_line = 0; // Clear test-option flag
			new_line = 0; // Clear new_line flag
		} // if (tst_line)
		else
		{ // Process other line
			// Check if we need to move to new-line
			if (new_line)
			{ // skip to next line
				while ('\n' != file_buf[char_cnt])
				{
					char_cnt++;
					assert(char_cnt < FILE_SIZE); // End-of-file found
				} // while ('\n' != file_buf[char_cnt])
	
				line_cnt++;
				new_line = 0; // Clear new_line flag
			} // if (new_line)
		} // else !(tst_line)
  } // for char_cnt

	printcharln(' ');
	release_lock(); // Release Display Mutex

	// Do some checks ...
	assert(test_cnt == NUM_TEST_OPTS); // Check read required number of test options found
	assert(NUM_TEST_OPTS <= line_cnt); // Check enough file lines read
	assert(test_cnt <= line_cnt); // Check no more than one test/line
 
	return;
} // parse_control_file
/*****************************************************************************/
static void read_sine_data( // read in tabulated sine values from file and set up test options
	GENERATE_TST_TYP &gen_data_s // Reference to structure of ADC test data
)
{
  int bytes_read; // No of bytes read
  int file_id; // File identifier
  int status = 0; // Error status


  file_id = _open( SIN_NAME ,O_RDONLY ,0 ); // Open control file for ADC tests

  assert(-1 != file_id); // ERROR: Open file failed (_open)

  bytes_read = _read( file_id ,(gen_data_s.table ,unsigned char[]) ,SIN_BUF_SIZE );  // Read file into buffer
  assert(bytes_read == SIN_BUF_SIZE);	// ERROR: File read failed (_read)

  status = _close(file_id);	// Close file
  assert(0 == status);	// ERROR: Close file failed (_close)

	return;
} // read_sine_data
/*****************************************************************************/
static void init_sine_data( // Initialise Sine data
	GENERATE_TST_TYP &gen_data_s // Reference to structure of Sine data
)
{
	timer chronometer; // local big-timer
	S64_T num_64; // 64-bit numerator value
	S64_T div_64; // 64-bit divisor value


	read_sine_data( gen_data_s );

	gen_data_s.first = 1; // Set flag for 1st data
	gen_data_s.err = 0; // Clear diffusion error
	gen_data_s.veloc = MAX_SPEC_RPM; // Initialise to Maximum speed
	gen_data_s.gain = MAX_GAIN; // Initialise to Maximum Gain
	gen_data_s.curr_mtim = 0; // Clear mock time

	// Calculate scaling factor for bit-shift division ...

	num_64 = (S64_T)1 << (SIN_RES_BITS + DIV_BITS); // 40-bit
	div_64 = (S64_T)(SECS_PER_MIN >> 2) * (S64_T)PLATFORM_REFERENCE_HZ; // 31-bit

	gen_data_s.scale = (int)((num_64 + (div_64 >> 1)) / div_64); // 10-bit
	gen_data_s.scale *= NUM_POLE_PAIRS; // 12-bit

	chronometer :> gen_data_s.real_time; // Initialise real-time
} // init_sine_data
/*****************************************************************************/
static void init_test_data( // Initialise ADC Test data
	GENERATE_TST_TYP &gen_data_s, // Reference to structure of ADC test data	
	streaming chanend c_chk, // Channel for communication with Checker cores
	streaming chanend c_adc // Channel for communication with ADC_Interface core
)
{
	int opt_flag; // Options flag


	init_common_data( gen_data_s.common ); // Initialise data common to Generator and Checker
 
	init_sine_data( gen_data_s );

	gen_data_s.period = ADC_PERIOD; // Typical time between ADC capture in FOC motor control loop

	gen_data_s.print = PRINT_TST_ADC; // Set print mode
	gen_data_s.dbg = 0; // Set debug mode

	parse_control_file( gen_data_s ); 

	// Check for sensible options ...

	opt_flag =	gen_data_s.common.options.flags[TST_SMALL] || 
							gen_data_s.common.options.flags[TST_PACE] ||
							gen_data_s.common.options.flags[TST_SLOW];

	if (0 == opt_flag)
	{
		acquire_lock(); 
		printstrln("ERROR Reading Test-Options file. Please select one of Selection_1/Selection_2/Slow_Speed");
		printstrln("      Aborting Program");
		printstrln("");
		release_lock();
		_exit(0);
	} // if (0 == opt_flag)

	c_chk <: gen_data_s.common.options; // Send test options to checker core

	c_adc <: gen_data_s.common.options.flags[TST_MOTOR] ; // Send Motor_Id to ADC-Interface

	gen_data_s.curr_vect.comp_state[CNTRL] = NO_PACE; // Initialise to No pacing (Fast execution)
	gen_data_s.prev_vect.comp_state[CNTRL] = QUIT; // Initialise to something that will force an update

} // init_motor_tests
/*****************************************************************************/
static ADC_GEN_TYP calc_one_adc_val( // Calculates one ADC value from angular position value
	GENERATE_TST_TYP &gen_data_s, // Reference to structure of ADC test data
	S64_T ang_val // 64-bit angular position value (measured in RPM*Clock_Ticks)
) // Returns and ADC value

/* Theory for look-up table
 * ------------------------
 * The table converts timer ticks into an values from a Sine Wave.
 * The table has NUM_SIN_VALS (n=2^12) entries representing values in the range [0 .. PI/2]
 * The first entry (0) is for angle PI/(4n), the last value (4095) is for angle (2n-1).PI/4n
 * The Sine values [0..1] are upscaled into the range [0..65535]
 *
 * The angle is dependent on angular speed(f) and time(t). A = 2.PI.f.t Where f is measured in revs/sec.
 * Therefore the time period for a whole revolution Tr = 1/f
 * There are NUM_POLE_PAIRS (P=4) electrical cycles for every revolution of the motor. 
 * Therefore electrical (Sine) period Ts = 1/(P.f)
 * Therefore time for a Sine Quadrant Tq = 1/(4.P.f)
 * Therefore time between Sine table entries is Ti = 1/(4.P.f.n)
 *
 * To convert a timer clocks tick (Tc) to a table entry we convert ticks to seconds (S)
 * using the Reference Frequency (Fr)       S = Tc/Fr
 * Now how many table entries(i) does S equate to ...
 *   S = Tc/Fr = (2.i + 1)/(8.P.f.n)
 * Therefore i = (8.P.n.(f.Tc)/Fr - 1)/2
 *
 * In the above equation f and Tc are the only variables. 
 * The angular frequency is supplied as RPM, therefore let f = R/60, were R is measured in RPM.
 * 	i = (8.P.n.(R.Tc)/(60.Fr) - 1)/2
 *  Let A = R.Tc  the angular position measured in (RPM * Clock_Ticks)
 *
 * The constants can be pre-computed.    Re-arranging ...
 *  i = (2.P.n.A - (15.Fr))/(30.Fr)
 *  i = (Scalar.A - Constant)/Divisor
 *
 * To improve speed we need a Divisor that is a power-of-2
 * Let (15.Fr) = (2^m)/X, then
 *  i = (2.P.n.X.A - (2^m))/(2^(m+1))       remembering that n is a power-of-2, Let n = 2^j, then
 * 
 *  i = (2.P.(2^j).X.A - (2^m))/(2^(m+1))
 *  i = (P.X.A - (2^(m-j-1))/(2^(m-j))
 * 
 * For Fr = 100MHz we choose X =733 and m=40 (then 15.100MHz is approx. (2^40)/733)
 *
 * Therefore with j = 12 we have
 * 
 * i = (733.P.A - (2^27)/(2^28)       and using bit-shift division
 * i = (733.P.A - (2^27) + (2^27)) >> 28;   // Conveniently rounding cancels with constant 
 * i = 733.P.A >> 28;   // Conveniently rounding cancels with constant 
 *
 * Therefore let    i = scale.A >> 28,      where  scale = 733.P
 */
{
	ADC_GEN_TYP out_adc; // Returned ADC value
  int index; // Index into Sine table
  int sin_val; // 17-bit Sine value (-65535 .. 65535)
	S64_T tmp_val; // 64-bit value


	tmp_val = (S64_T)gen_data_s.scale * ang_val; // 50-bits
	tmp_val = tmp_val >> DIV_BITS; 
	index = (int)tmp_val; // Convert to 32-bit


	index &= MAX_SINx4_ID;	// Wrap index into lowest Sine period range

	// Determine which quadrant to simulate
	if (index <= MAX_SINx2_ID)
	{ // 1st or 2nd Quadrant
		if (index <= MAX_SIN_INDEX)
		{ // 1st Quadrant
			sin_val = gen_data_s.table[index];
		} // if (index <= MAX_SIN_INDEX)
		else
		{ // 2nd Quadrant
			sin_val = gen_data_s.table[MAX_SINx2_ID - index]; // Mirror around PI/2
		} // else !(index <= MAX_SIN_INDEX)
	} // if (index <= MAX_SINx2_ID)
	else
	{ // 3rd or 4th Quadrant
		if (index <= MAX_SINx3_ID)
		{ // 3rd Quadrant
			sin_val = -gen_data_s.table[index - NUM_SINx2_VALS]; // Mirror around X-axis
		} // if (index <= MAX_SINx3_ID)
		else
		{ // 4th Quadrant
			sin_val = -gen_data_s.table[MAX_SINx4_ID - index]; // Mirror around X-axis * 3.PI/2
		} // else !(index <= MAX_SINx3_ID)
	} // else !(index <= MAX_SINx2_ID)

	// NB Adding sign bit makes sin_val a 17-bit number

	out_adc = (ADC_GEN_TYP)(gen_data_s.gain * sin_val);	// Adjust amplitude
	// NB This value is down-scaled in the ADC_interface to simulate the ADC chip behaviour

	gen_data_s.prev_mtim = gen_data_s.curr_mtim; // Store mock time for next iteration

	return out_adc;
} // calc_one_adc_val
/*****************************************************************************/
static void calc_pair_of_adc_vals( // Calculates Phase_A and Phase_B ADC values from Gain and time data
	GENERATE_TST_TYP &gen_data_s // Reference to structure of ADC test data
)
{
	unsigned diff_time;
	ADC_GEN_TYP adc_val; // ADC value
	S64_T ang_val; // 64-bit value


	// Compute angular distance rotated ...

	if (gen_data_s.first)
	{ // Special case of 1st data
		gen_data_s.sum_time = 0;
		gen_data_s.first = 0; // Switch off first data flag
	} // if (gen_data_s.first)
	else
	{ // NOT 1st data

		// Calculate time difference from previous request
		diff_time = gen_data_s.curr_mtim - gen_data_s.prev_mtim; // NB handles clock wrap

		// Check for clock wrap
		if (gen_data_s.sum_time < ((unsigned)0xFFFFFFFF - diff_time))
		{
			gen_data_s.sum_time += diff_time; // Convert to 32-bit time 
		} // if (gen_data_s.sum_time < ((unsigned)0xFFFFFFFF - diff_time))
		else
		{ // Clock Overflow
			assert(0 == 1); // ERROR: Clock overflow
			/* Well its was thought this would never happen in a realistic time-frame.
			 * However, now clock-wrap must be implemented as follows:
			 * The clock-time needs to be reduced by an amount that changes the sine-angle by 2.PI
       * So the amount of time to subtract is inversely proportional to the angular velocity.
       */
		} // if (gen_data_s.sum_time < ((unsigned)0xFFFFFFFF - diff_time))
	} // else !(gen_data_s.first)

	ang_val = (S64_T)gen_data_s.veloc * (S64_T)gen_data_s.sum_time; // 'Angular position' 40-bits

	adc_val = calc_one_adc_val( gen_data_s ,ang_val );
	gen_data_s.adc_a = adc_val;

	ang_val += (S64_T)FORTY_SECS; // Step on angular value by 2/3 of a rev (NB measured in RPM*Clock_ticks)

	adc_val = calc_one_adc_val( gen_data_s ,ang_val );

	gen_data_s.adc_b = adc_val;
} // calc_pair_of_adc_vals
/*****************************************************************************/
static void print_gen_data( // Print ADC generation data
	GENERATE_TST_TYP &gen_data_s // Reference to structure of ADC test data
)
{
	acquire_lock(); // Acquire Display Mutex

	printint( gen_data_s.curr_mtim );
	printstr( " > " );
	printint( gen_data_s.gain );
	
	switch(gen_data_s.gain)
	{
		case MAX_GAIN :
			printstr( ":" ); printint(gen_data_s.adc_a >> GAIN_BITS); printstr( ":" ); printintln(gen_data_s.adc_b >> GAIN_BITS);
		break; // MAX_GAIN

		case MIN_GAIN :
			printstr( ":" ); printint(gen_data_s.adc_a); printstr( ":" ); printintln(gen_data_s.adc_b);
		break; // MIN_GAIN

		default:
			printstr( ":" ); printint(gen_data_s.adc_a/gen_data_s.gain); printstr( ":" ); printintln(gen_data_s.adc_b/gen_data_s.gain);
		break; // default
	} // switch(gen_data_s.gain)

	release_lock(); // Release Display Mutex
} // print_gen_data
/*****************************************************************************/
static void send_adc_values( // Calculates and send ADC values
	GENERATE_TST_TYP &gen_data_s, // Reference to structure of ADC test data
	streaming chanend c_chk, // Channel for communication with Checker core
	streaming chanend c_adc // Channel for communicating with ADC_Interface core
)
{
	calc_pair_of_adc_vals( gen_data_s ); // Compute ADC value

	// Check if verbose printing required
	if (gen_data_s.print)
	{
		print_gen_data( gen_data_s );
	} // if (gen_data_s.print)

	c_adc <: gen_data_s.adc_a; // return ADC value for Phase_A
	c_adc <: gen_data_s.adc_b; // return ADC value for Phase_B
	c_chk <: gen_data_s.curr_mtim; // Pass on mock time to Checker core
	
	gen_data_s.curr_mtim += ADC_PERIOD; // Update mock time 
} // send_adc_values
/*****************************************************************************/
static void assign_test_vector_sum( // Assign Zero-sum state of test vector
	GENERATE_TST_TYP &gen_data_s, // Reference to structure of ADC test data
	SUM_ADC_ENUM inp_sum // Input Zero-sum state
)
{
	gen_data_s.curr_vect.comp_state[SUM] = inp_sum; // Update Zero-sum state of test vector
} // assign_test_vector_sum
/*****************************************************************************/
static void assign_test_vector_mean( // Assign Zero-mean state of test vector
	GENERATE_TST_TYP &gen_data_s, // Reference to structure of ADC test data
	MEAN_ADC_ENUM inp_mean // Input Zero-mean state
)
{
	gen_data_s.curr_vect.comp_state[MEAN] = inp_mean; // Update Zero-mean state of test vector
} // assign_test_vector_mean
/*****************************************************************************/
static void assign_test_vector_spin( // Assign Spin-state of test vector
	GENERATE_TST_TYP &gen_data_s, // Reference to structure of ADC test data
	SPIN_ADC_ENUM inp_spin // Input Spin-state
)
{
	switch( inp_spin )
	{
		case CLOCK: // Clock-wise
			gen_data_s.sign = 1; 
		break; // case CLOCK:

		case ANTI: // Anti-clockwise
			gen_data_s.sign = -1; 
		break; // case ANTI:

		default:
			acquire_lock(); // Acquire Display Mutex
			printstrln("ERROR: Unknown ADC Spin-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( inp_spin )

	gen_data_s.curr_vect.comp_state[SPIN] = inp_spin; // Update Spin-state of test vector
} // assign_test_vector_spin
/*****************************************************************************/
static void assign_test_vector_gain( // Assign Gain-state of test vector
	GENERATE_TST_TYP &gen_data_s, // Reference to structure of ADC test data
	GAIN_ADC_ENUM inp_gain // Input gain-state
)
{
	switch( inp_gain )
	{
		case SMALL: // Small amplitude
			gen_data_s.gain = MIN_GAIN; 
		break; // case SMALL

		case LARGE: // large amplitude
			gen_data_s.gain = MAX_GAIN; 
		break; // case LARGE

		default:
			acquire_lock(); // Acquire Display Mutex
			printstrln("ERROR: Unknown ADC Gain-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( inp_gain )

	gen_data_s.curr_vect.comp_state[GAIN] = inp_gain; // Update gain-state of test vector
} // assign_test_vector_gain
/*****************************************************************************/
static void assign_test_vector_speed( // Assign Speed-state of test vector
	GENERATE_TST_TYP &gen_data_s, // Reference to structure of ADC test data
	SPEED_ADC_ENUM inp_speed // Input speed-state
)
{
	switch( inp_speed )
	{
		case FAST: // Fast Speed
			gen_data_s.speed = HI_SPEED; // Set fast speed (in RPM)
		break; // case FAST:

		case SLOW: // Slow Speed
			gen_data_s.speed = LO_SPEED; // Set slow speed (in RPM)
		break; // case SLOW:

		default:
			acquire_lock(); // Acquire Display Mutex
			printstrln("ERROR: Unknown ADC Speed-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( inp_speed )

	gen_data_s.curr_vect.comp_state[SPEED] = inp_speed; // Update speed-state of test vector
} // assign_test_vector_speed
/*****************************************************************************/
static int vector_compare( // Check if 2 sets of test vector are different
	TEST_VECT_TYP &vect_a, // Structure of containing 1st set of vectore components
	TEST_VECT_TYP &vect_b  // Structure of containing 2nd set of vectore components
) // return TRUE (1) if vectors are different, FALSE(0) if equal
{
	VECT_COMP_ENUM comp_cnt; // vector component counter


	for (comp_cnt=0; comp_cnt<NUM_VECT_COMPS; comp_cnt++)
	{
		if (vect_a.comp_state[comp_cnt] != vect_b.comp_state[comp_cnt]) return 1;
	} // for comp_cnt=0

	return 0; // No differences found
} // vector_compare
/*****************************************************************************/
static void gen_adc_test_data( // Generate all ADC values for one ADC test vector
	GENERATE_TST_TYP &gen_data_s, // Reference to structure of ADC test data
	streaming chanend c_chk, // Channel for communication with Checker core
	streaming chanend c_adc // Channel for communication with ADC_Interface core
)
{
	timer chronometer; // local big-timer
	int chk_cmd; // Command sent from Checker


	gen_data_s.veloc = gen_data_s.speed * gen_data_s.sign; // Update angular velocity

	// loop until End-of-test command received
	do // while (END_TST_CMD != chk_cmd)
	{
		c_chk :> chk_cmd; // Get next command from Checker

		switch(chk_cmd)
		{
			case TST_REQ_CMD : // Request for ADC data
				send_adc_values( gen_data_s ,c_chk ,c_adc );

				// Update time of ADC output
				if (PACE_ON == gen_data_s.curr_vect.comp_state[CNTRL])
				{ // Pacing ON
					chronometer when timerafter(gen_data_s.real_time + ADC_PERIOD) :> void; // Wait for 'ADC pacing' time
				} // if (PACE_ON == gen_data_s.curr_vect.comp_state[CNTRL])

				chronometer :> gen_data_s.real_time; // Update local time
			break; // TST_REQ_CMD

			case END_TST_CMD : // Terminate test
			break; // END_TST_CMD

			default:
				assert(0 == 1); // ERROR: Unexpected Test Command
			break; // default
		} // switch(chk_cmd)
	} while (END_TST_CMD != chk_cmd);

} // gen_adc_test_data
/*****************************************************************************/
static void do_adc_vector( // Do all tests for one ADC test vector
	GENERATE_TST_TYP &gen_data_s, // Reference to structure of ADC test data
	streaming chanend c_chk, // Channel for communication with Checker core
	streaming chanend c_adc // Channel for communication with ADC_Interface core
)
{
	int new_vect; // flag set if new test vector detected


	new_vect = vector_compare( gen_data_s.curr_vect ,gen_data_s.prev_vect );

	// Check for new test-vector
	if (new_vect)
	{
// acquire_lock(); printstr("Vec_S="); printintln(gen_data_s.curr_vect.comp_state[SPEED]); release_lock(); // MB~
		c_chk <: gen_data_s.curr_vect; // Send new test-vector

		// Check if verbose printing required
		if (gen_data_s.print)
		{
			print_test_vector( gen_data_s.common ,gen_data_s.curr_vect ,"" );
		} // if (gen_data_s.print)

		if (QUIT != gen_data_s.curr_vect.comp_state[CNTRL])
		{
			gen_adc_test_data( gen_data_s ,c_chk ,c_adc ); // Generate all ADC values for this test
		} // if (QUIT != gen_data_s.curr_vect.comp_state[CNTRL])

		gen_data_s.prev_vect = gen_data_s.curr_vect; // update previous vector
	} // if (new_vect)

} // do_adc_vector
/*****************************************************************************/
static void gen_motor_adc_test_data( // Generate ADC Test data for one motor
	GENERATE_TST_TYP &gen_data_s, // Reference to structure of ADC test data
	streaming chanend c_chk, // Channel for communication with Checker core
	streaming chanend c_adc // Channel for communication with ADC_Interface core
)
{
	if (gen_data_s.print)
	{
		acquire_lock(); // Acquire Display Mutex
		printstr( " Start Test Gen. For Motor_"); printintln( gen_data_s.common.options.flags[TST_MOTOR] );
		release_lock(); // Release Display Mutex
	} // if (gen_data_s.print)

	// NB These tests assume ADC_FILTER = 0

	assign_test_vector_sum( gen_data_s ,SUM_ON ); // Set test vector to test zero-sum
	assign_test_vector_mean( gen_data_s ,MEAN_ON ); // Set test vector to test zero-mean

	// Check if Small-Gain tests activated
	if (gen_data_s.common.options.flags[TST_SMALL])
	{
		assign_test_vector_speed( gen_data_s ,FAST ); // Set test vector to Fast Speed to shorten time.
		assign_test_vector_gain( gen_data_s ,SMALL ); // Set test vector to Small gain
		assign_test_vector_spin( gen_data_s ,CLOCK ); // Set test vector to Clock-wise spin
		gen_data_s.curr_vect.comp_state[CNTRL] = NO_PACE; // Set control to No Pacing (Fast Execution)

		do_adc_vector( gen_data_s ,c_chk ,c_adc );
	} // if (gen_data_s.common.options.flags[TST_SMALL] || etc.

	// Check if Pacing tests activated
	if (gen_data_s.common.options.flags[TST_PACE])
	{
		assign_test_vector_speed( gen_data_s ,FAST ); // Set test vector to Fast Speed to shorten time.
		assign_test_vector_gain( gen_data_s ,LARGE ); // Set test vector to Large gain
		assign_test_vector_spin( gen_data_s ,ANTI ); // Set test vector to Anti-clockwise spin
		gen_data_s.curr_vect.comp_state[CNTRL] = PACE_ON; // Set control to Paced (Slow Execution)

		do_adc_vector( gen_data_s ,c_chk ,c_adc );
	} // if (gen_data_s.common.options.flags[TST_PACE] || etc.

	// Check if Slow test activated
	if (gen_data_s.common.options.flags[TST_SLOW])
	{ // Do Slow speed test
		assign_test_vector_speed( gen_data_s ,SLOW ); // Set test vector to Slow Speed (very long).
		assign_test_vector_gain( gen_data_s ,LARGE ); // Set test vector to Large gain
		assign_test_vector_spin( gen_data_s ,CLOCK ); // Set test vector to Clock-wise spin
		gen_data_s.curr_vect.comp_state[CNTRL] = NO_PACE; // Set control to No Pacing (Fast Execution)

		do_adc_vector( gen_data_s ,c_chk ,c_adc );
	} // if (gen_data_s.common.options.flags[TST_SLOW])

	gen_data_s.curr_vect.comp_state[CNTRL] = QUIT; // Signal that testing has ended for current motor
	do_adc_vector( gen_data_s ,c_chk ,c_adc );

} // gen_motor_adc_test_data
/*****************************************************************************/
void gen_all_adc_test_data( // Generate ADC Test data
	streaming chanend c_chk, // Channel for communication with Checker core
	streaming chanend c_adc // Channel for communication with ADC_Interface core
)
{
	GENERATE_TST_TYP gen_data_s; // Structure of ADC test data


	init_test_data( gen_data_s ,c_chk ,c_adc );

	gen_motor_adc_test_data( gen_data_s ,c_chk ,c_adc );

	if (gen_data_s.print)
	{
		acquire_lock(); // Acquire Display Mutex
		printstrln( "Test Generation Ends " );
		release_lock(); // Release Display Mutex
	} // if (gen_data_s.print)
} // gen_all_adc_test_data
/*****************************************************************************/
