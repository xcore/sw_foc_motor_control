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
static void init_adc_data( // Initialise ADC Test data
	TEST_ADC_TYP &tst_data_s, // Reference to structure of ADC test data
	TEST_WAVE_TYP &tst_wave_s // Reference to structure containing all wave data
)
{
	int samp_cnt; // sample counter


	acquire_lock(); // Acquire Display Mutex
	printstrln("Initialising Data ...");

	for (samp_cnt=0; samp_cnt<NUM_WAVE_SAMPS; samp_cnt++)
	{
		printchar('.');
		tst_wave_s.sine.samps[samp_cnt] = get_sine_value( samp_cnt ,NUM_WAVE_SAMPS ,MAX_ADC_VAL );
	} // for samp_cnt

	printcharln('|');
	release_lock(); // Release Display Mutex

} // init_adc_data
/*****************************************************************************/
static void transmit_adc_data( // transmits data for one ADC test
	TEST_ADC_TYP &tst_data_s, // Reference to structure of ADC test data
	buffered out port:32 pb32_tst[], // Array of current port on which to transmit test data
	chanend c_pwm2adc_trig // Array of channels outputting ADC trigger 
)
{
	int phase_cnt; // counter for ADC phases


	acquire_lock(); // Acquire Display Mutex
	printint( tst_data_s.id );

	// Transmit ADC sample for used ADC phases 
	for (phase_cnt=0; phase_cnt<USED_ADC_PHASES; phase_cnt++) 
	{
		pb32_tst[phase_cnt] <: tst_data_s.vals[phase_cnt];	// Transmit ADC sample for current ADC phase 
		printstr( " : " ); printint( tst_data_s.vals[phase_cnt] );
	} // for phase_cnt

	/* Send synchronisation token to ADC. 
	 * NB In full FOC motor control loop, this is usually done by PWM server
   */
	outct( c_pwm2adc_trig ,XS1_CT_END );

	printstrln( " :ADC" );
	release_lock(); // Release Display Mutex
} // transmit_adc_data
/*****************************************************************************/
static void do_adc_test( // Performs one ADC test
	TEST_ADC_TYP &tst_data_s, // Reference to structure of ADC test data
	ADC_WAVE_TYP &wave_data_s, // Reference to structure containing data for one wave
	buffered out port:32 pb32_tst[], // Array of current port on which to transmit test data
	chanend c_pwm2adc_trig // Array of channels outputting ADC trigger 
)
{
	int curr_off; // current offset into array of wave samples
	int phase_cnt; // counter for ADC phases


	tst_data_s.cnt = (tst_data_s.cnt + 1) & MAX_WAVE_ID; // Increment (and wrap) ADC position

	// Transmit ADC sample for each ADC phase 
	for (phase_cnt=0; phase_cnt<NUM_ADC_PHASES; phase_cnt++) 
	{
		curr_off = (tst_data_s.cnt + tst_data_s.offsets[phase_cnt]) & MAX_WAVE_ID; // Calculate (and wrap) Wave offset
		tst_data_s.vals[phase_cnt] = wave_data_s.samps[curr_off]; // store current adc value
	} // for phase_cnt

	transmit_adc_data( tst_data_s ,pb32_tst ,c_pwm2adc_trig ); // Transmit test data
} // do_adc_test
/*****************************************************************************/
static void steady_speed( // Holds steady speed for a number of tests
	TEST_ADC_TYP &tst_data_s, // Reference to structure of ADC test data
	ADC_WAVE_TYP &wave_data_s, // Reference to structure containing data for one wave
	buffered out port:32 pb32_tst[], // Array of current port on which to transmit test data
	chanend c_pwm2adc_trig, // Array of channels outputting ADC trigger 
	int test_cnt, // count-down test counter
	const char str_val[] // String containing title for tests
)
{
	timer chronometer; // XMOS timer


	// Initial start-up, spin at full speed
	acquire_lock(); // Acquire Display Mutex
	printstrln( str_val ); // Print test title
	release_lock(); // Release Display Mutex

	// Loop until all tests done
	while(test_cnt)
	{
		// Wait till test period elapsed
		chronometer when timerafter(tst_data_s.time + tst_data_s.period) :> tst_data_s.time;

		do_adc_test( tst_data_s ,wave_data_s ,pb32_tst ,c_pwm2adc_trig ); // Performs one ADC test

		test_cnt--; // Decrement test counter
	} // while(test_cnt)

} // steady_speed
/*****************************************************************************/
static void gen_motor_adc_test_data( // Generate ADC Test data for one motor
	TEST_ADC_TYP &tst_data_s, // Reference to structure of ADC test data
	TEST_WAVE_TYP &tst_wave_s, // Reference to structure containing all wave data
	buffered out port:32 pb32_tst[], // Array of current port on which to transmit test data
	chanend c_pwm2adc_trig // Array of channels outputting ADC trigger 
)
{
	int phase_cnt; // counter for ADC phases
	timer chronometer; // XMOS timer


	chronometer :> tst_data_s.time; // Get start time

	// NB These tests assume ADC_FILTER = 0

	tst_data_s.period = TEST_TIME; // Time period between tests
	steady_speed( tst_data_s ,tst_wave_s.sine ,pb32_tst ,c_pwm2adc_trig ,MAX_TESTS ," Max_Speed Clockwise ");

	// Signal end-of-tests (all phases max. value) ...

	for (phase_cnt=0; phase_cnt<NUM_ADC_PHASES; phase_cnt++) 
	{
		tst_data_s.vals[phase_cnt] = MAX_ADC_VAL; // store max. adc value
	} // for phase_cnt

	transmit_adc_data( tst_data_s ,pb32_tst ,c_pwm2adc_trig ); // Transmit 'end-of-tests' data

} // gen_motor_adc_test_data
/*****************************************************************************/
void gen_all_adc_test_data( // Generate ADC Test data
	buffered out port:32 pb32_tst[][NUM_ADC_PHASES], // Array of 32-bit buffered ports outputing test ADC values 
	chanend c_pwm2adc_trig[] // Array of channels outputting ADC trigger 
)
{
	TEST_ADC_TYP tst_data_s; // Structure of ADC test data
	TEST_WAVE_TYP tst_wave_s; // Structure containing all wave data
	int motor_cnt; // counts Motors 
	int phase_cnt; // counter for ADC phases
	int offset_inc = 0; // counter for ADC phases


	init_adc_data( tst_data_s ,tst_wave_s );

	// Loop through motors, so we can print results serially
	for (motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++)
	{
		acquire_lock(); // Acquire Display Mutex
		printstr( " Start Test Gen. For Motor_"); printintln( motor_cnt );
		release_lock(); // Release Display Mutex

		tst_data_s.cnt = 0; // Set ADC position counter to arbitary number
		tst_data_s.id = motor_cnt; // Assign current motor identifier

		// Initialise  ADC phase offset
		for (phase_cnt=0; phase_cnt<NUM_ADC_PHASES; phase_cnt++) 
		{
			tst_data_s.offsets[phase_cnt] = (offset_inc + 1) / NUM_ADC_PHASES;
			offset_inc += NUM_WAVE_SAMPS;
		} // for phase_cnt

		gen_motor_adc_test_data( tst_data_s ,tst_wave_s ,pb32_tst[motor_cnt] ,c_pwm2adc_trig[motor_cnt] );
	} // for motor_cnt

	acquire_lock(); // Acquire Display Mutex
	printstrln( "Test Generation Ends " );
	release_lock(); // Release Display Mutex
} // gen_all_adc_test_data
/*****************************************************************************/
