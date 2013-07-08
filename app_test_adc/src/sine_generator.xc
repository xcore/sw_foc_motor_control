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

#include "sine_generator.h"

/*****************************************************************************/
static void read_sine_data( // read in tabulated sine values from file and set up test options
	SINE_TST_TYP &sin_data_s // Reference to structure of generated Sine data
)
{
  int bytes_read; // No of bytes read
  int file_id; // File identifier
  int status = 0; // Error status


  file_id = _open( SIN_NAME ,O_RDONLY ,0 ); // Open control file for ADC tests

  assert(-1 != file_id); // ERROR: Open file failed (_open)

  bytes_read = _read( file_id ,(sin_data_s.table ,unsigned char[]) ,SIN_BUF_SIZE );  // Read file into buffer
  assert(bytes_read == SIN_BUF_SIZE);	// ERROR: File read failed (_read)

  status = _close(file_id);	// Close file
  assert(0 == status);	// ERROR: Close file failed (_close)

	return;
} // read_sine_data
/*****************************************************************************/
static void init_sine_data( // Initialise Sine data
	SINE_TST_TYP &sin_data_s // Reference to structure of Sine data
)
{
	S64_T num_64; // 64-bit numerator value
	S64_T div_64; // 64-bit divisor value


	read_sine_data( sin_data_s );

	sin_data_s.first = 1; // Set flag for 1st data
	sin_data_s.err = 0; // Clear diffusion error
	sin_data_s.veloc = MAX_SPEC_RPM; // Initialise to Maximum speed
	sin_data_s.gain = MAX_GAIN; // Initialise to Maximum Gain
	sin_data_s.curr_mtim = 0; // Clear mock time

	// Calculate scaling factor for bit-shift division ...

	num_64 = (S64_T)1 << (SIN_RES_BITS + DIV_BITS); // 40-bit
	div_64 = (S64_T)(SECS_PER_MIN >> 2) * (S64_T)PLATFORM_REFERENCE_HZ; // 31-bit

	sin_data_s.scale = (int)((num_64 + (div_64 >> 1)) / div_64); // 10-bit
	sin_data_s.scale *= NUM_POLE_PAIRS; // 12-bit
} // init_sine_data
/*****************************************************************************/
static ADC_GEN_TYP calc_one_adc_val( // Calculates one ADC value from angular position value
	SINE_TST_TYP &sin_data_s, // Reference to structure of generated Sine data
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


	tmp_val = (S64_T)sin_data_s.scale * ang_val; // 50-bits
	tmp_val = tmp_val >> DIV_BITS; 
	index = (int)tmp_val; // Convert to 32-bit


	index &= MAX_SINx4_ID;	// Wrap index into lowest Sine period range

	// Determine which quadrant to simulate
	if (index <= MAX_SINx2_ID)
	{ // 1st or 2nd Quadrant
		if (index <= MAX_SIN_INDEX)
		{ // 1st Quadrant
			sin_val = sin_data_s.table[index];
		} // if (index <= MAX_SIN_INDEX)
		else
		{ // 2nd Quadrant
			sin_val = sin_data_s.table[MAX_SINx2_ID - index]; // Mirror around PI/2
		} // else !(index <= MAX_SIN_INDEX)
	} // if (index <= MAX_SINx2_ID)
	else
	{ // 3rd or 4th Quadrant
		if (index <= MAX_SINx3_ID)
		{ // 3rd Quadrant
			sin_val = -sin_data_s.table[index - NUM_SINx2_VALS]; // Mirror around X-axis
		} // if (index <= MAX_SINx3_ID)
		else
		{ // 4th Quadrant
			sin_val = -sin_data_s.table[MAX_SINx4_ID - index]; // Mirror around X-axis * 3.PI/2
		} // else !(index <= MAX_SINx3_ID)
	} // else !(index <= MAX_SINx2_ID)

	// NB Adding sign bit makes sin_val a 17-bit number

	out_adc = (ADC_GEN_TYP)(sin_data_s.gain * sin_val);	// Adjust amplitude
	// NB This value is down-scaled in the ADC_interface to simulate the ADC chip behaviour

	sin_data_s.prev_mtim = sin_data_s.curr_mtim; // Store mock time for next iteration
/* MB~
acquire_lock(); 
printstr(" A="); printllong(ang_val);
printstr(" S="); printint(sin_data_s.scale);
printstr(" M="); printllong(tmp_val);
printstr(" I="); printint(index); 
printstr(" V="); printint(sin_val); 
printstr(" A="); printintln(out_adc);
release_lock(); // MB~
*/
	return out_adc;
} // calc_one_adc_val
/*****************************************************************************/
static void calc_all_adc_vals( // Calculates all ADC values from Gain and time data
	SINE_TST_TYP &sin_data_s // Reference to structure of generated Sine data
)
{
	unsigned diff_time;
	ADC_GEN_TYP adc_val; // ADC value
	S64_T ang_val; // 64-bit value


	// Compute angular distance rotated ...

	if (sin_data_s.first)
	{ // Special case of 1st data
		sin_data_s.sum_time = 0;
		sin_data_s.first = 0; // Switch off first data flag
	} // if (sin_data_s.first)
	else
	{ // NOT 1st data

		// Calculate time difference from previous request
		diff_time = sin_data_s.curr_mtim - sin_data_s.prev_mtim; // NB handles clock wrap

		sin_data_s.sum_time += (unsigned)diff_time; // Convert to 32-bit time 
	} // else !(sin_data_s.first)

	// Check if time for clock wrap
	if (CLOCK_WRAP < sin_data_s.sum_time) sin_data_s.sum_time -= CLOCK_WRAP;

	ang_val = (S64_T)sin_data_s.veloc * (S64_T)sin_data_s.sum_time; // 'Angular position' 40-bits

	adc_val = calc_one_adc_val( sin_data_s ,ang_val );
	sin_data_s.adc_a = adc_val;

	ang_val += (S64_T)FORTY_SECS; // Step on angular value by 2/3 of a rev (NB measured in RPM*Clock_ticks)

	adc_val = calc_one_adc_val( sin_data_s ,ang_val );

	sin_data_s.adc_b = adc_val;
} // calc_all_adc_vals
/*****************************************************************************/
static void update_ang_velocity( // Updates angular velocity
	SINE_TST_TYP &sin_data_s // Reference to structure of generated Sine data
)
{
	switch( sin_data_s.curr_vect.comp_state[SPIN] )
	{
		case CLOCK: // Clock-wise
			sin_data_s.sign = 1; 
		break; // case CLOCK:

		case ANTI: // Anti-clockwise
			sin_data_s.sign = -1; 
		break; // case ANTI:

		default:
			acquire_lock(); // Acquire Display Mutex
			printstrln("ERROR: Unknown ADC Spin-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( inp_spin )

	sin_data_s.veloc = MAX_SPEC_RPM * sin_data_s.sign; // NB run at max speed to shorten time time.

acquire_lock(); printstr("VEL="); printintln(sin_data_s.veloc); release_lock(); // MB~
} // update_ang_velocity
/*****************************************************************************/
static void process_new_test_vector( // Process new test vector
	SINE_TST_TYP &sin_data_s // Reference to structure of generated Sine data
)
{
	int change = 0; // Clear flag indicating change in test vector detected


	// Check for change in speed test
	if (sin_data_s.curr_vect.comp_state[SPIN] != sin_data_s.prev_vect.comp_state[SPIN])
	{
		update_ang_velocity( sin_data_s );

		change = 1; // Set flag indicating change in test vector detected
	} // if (sin_data_s.curr_vect.comp_state[SPIN] != sin_data_s.prev_vect.comp_state[SPIN])

	// Check if test vector changed
	if (change)
	{
		sin_data_s.prev_vect = sin_data_s.curr_vect; // Update previous test-vector
	} // if (change)
} // process_new_test_vector
/*****************************************************************************/
static void send_adc_values( // Calculates and send ADC values
	SINE_TST_TYP &sin_data_s, // Reference to structure of generated Sine data
	streaming chanend c_chk, // Channel for communication with Checker core
	streaming chanend c_adc // Channel for communicating with ADC_Interface core
)
{
	calc_all_adc_vals( sin_data_s ); // Compute ADC value
acquire_lock(); printstr("G_Ang="); printint(sin_data_s.adc_a); printstr(":"); printintln(sin_data_s.adc_b); release_lock(); // MB~
	c_adc <: sin_data_s.adc_a; // return ADC value for Phase_A
	c_adc <: sin_data_s.adc_b; // return ADC value for Phase_B
	c_chk <: sin_data_s.curr_mtim; // Pass on mock time to Checker core
	
	sin_data_s.curr_mtim += ADC_PERIOD; // Update mock time 
} // send_adc_values
/*****************************************************************************/
void get_sine_data( // Transmits a Sine value having received a velocity and time value
	streaming chanend c_tst, // Channel for communication with Test_Generator
	streaming chanend c_chk, // Channel for communication with Checker core
	streaming chanend c_adc // Channel for communicating with ADC_Interface core
)
{
	SINE_TST_TYP sin_data_s; // Structure of generated Sine data
	timer chronometer; // local big-timer
	unsigned real_time; // real time-stamp (from local timer)
	int chk_cmd; // Command sent from Checker


	init_sine_data( sin_data_s );

	c_tst :> sin_data_s.curr_vect; // Wait for 1st test vector
	update_ang_velocity( sin_data_s );

	chronometer :> real_time; // Initialise real-time

	// loop until QUIT Test Vector found
	while(QUIT != sin_data_s.curr_vect.comp_state[CNTRL])
	{
		select 
		{
			case c_tst :> sin_data_s.curr_vect : // Service any change in Test Vector
				process_new_test_vector( sin_data_s ); // Process new test vector
			break; // c_tst

			case c_chk :> chk_cmd : // Service command from Checker
				if (TST_REQ_CMD == chk_cmd)
				{ 
					send_adc_values( sin_data_s ,c_chk ,c_adc );
	
					// Update time of ADC output
					if (PACE_ON == sin_data_s.curr_vect.comp_state[PACE])
					{ // Pacing ON
						chronometer when timerafter(real_time + ADC_PERIOD) :> void; // Wait for 'ADC pacing' time
					} // if (PACE_ON == sin_data_s.curr_vect.comp_state[PACE])
	
					chronometer :> real_time; // Update local time
				} // if (TST_REQ_CMD == chk_cmd)
			break; // c_chk
		} // select
	} // while(QUIT != sin_data_s.curr_vect.comp_state[CNTRL])
} // get_sine_data
/*****************************************************************************/
