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
static void init_adc_params( // Initialise ADC parameter structure
	ADC_PARAM_TYP &adc_param_s,	// Reference to structure containing ADC data
	int motor_id // Motors Identifier
)
{
	int phase_cnt; // counter for ADC phases

	// Loop through used ADC phases
	for (phase_cnt=0; phase_cnt<NUM_ADC_PHASES; ++phase_cnt) 
	{
		adc_param_s.vals[phase_cnt] = 0;
	} // for phase_cnt

} // print_adc_data
/*****************************************************************************/
static void print_adc_data( // Print elements of ADC structure
	ADC_PARAM_TYP &adc_param_s,	// Reference to structure containing ADC data
	int test_id, // Test Identifier
	int motor_id // Motors Identifier
)
{
	int phase_cnt; // counter for ADC phases


	acquire_lock(); // Acquire Display Mutex

	printstr( "                             " );
	printstr( "P=" );

	// Loop through used ADC phases
	for (phase_cnt=0; phase_cnt<NUM_ADC_PHASES; ++phase_cnt) 
	{
		printint( adc_param_s.vals[phase_cnt] );
		printstr( "  " );
	} // for phase_cnt

	printchar(':');
	printintln( motor_id );

	release_lock(); // Release Display Mutex
} // print_adc_data
/*****************************************************************************/
static void disp_motor_adc_client_data( // Display ADC results for one motor
	int motor_id, // Motors Identifier
	streaming chanend c_adc // ADC channel between Client and Server
)
{
	ADC_PARAM_TYP adc_param_s;	// Structure containing ADC parameters
	ADC_PARAM_TYP prev_param_s;	// Structure containing previous ADC parameters
	int test_cnt;
	timer chronometer; // XMOS timer
	unsigned time_val;   // time value
	unsigned time_inc;   // time increment
	int diff_flg;   // Flag set if difference in parameters found
	int do_loop = 1;   // Flag set until loop-end condition found 
	int phase_cnt; // counter for ADC phases


	init_adc_params( adc_param_s ,motor_id ); // Initialise ADC parameter structure
	
	chronometer :> time_val; // Get start time
	chronometer when timerafter(time_val + (MICRO_SEC << 1)) :> time_val; // Wait for Test Generation to Start

	acquire_lock(); // Acquire Display Mutex
	printstr( "                             " );
	printstr("Start Checks For Motor_"); printintln( motor_id ); 
	release_lock(); // Release Display Mutex

	time_inc = TEST_TIME; // time increment between ADC Client requests. (Typical value when using real motor)

	// Loop until end condition found
	while( do_loop )
	{
		foc_adc_get_parameters( adc_param_s ,c_adc ); // Client function under test

		// Check for difference in parameters supplied by Client

		// Initialise difference flag with first phase
		diff_flg = (adc_param_s.vals[ADC_PHASE_A] != prev_param_s.vals[ADC_PHASE_A]);

		phase_cnt=ADC_PHASE_B; // Initialise phase count to second phase
 
		// Loop while no differences
		while ((!diff_flg) && (phase_cnt<NUM_ADC_PHASES)) 
		{
			if (adc_param_s.vals[phase_cnt] != prev_param_s.vals[phase_cnt])
			{
				diff_flg = 1; // Signal differences found
			} // if (adc_param_s.vals[phase_cnt] != prev_param_s.vals[phase_cnt])

			phase_cnt++; // Increment phase count
		} // while ((!diff_flg) ...

/* XXXXXXXX Currently, there are never any differences!-(
 * To resolve this, foc_adc_7265_triggered() needs splitting into H/W and S/w components
 */

		// Check for differences
		if (diff_flg)
		{ // Data different
			print_adc_data( adc_param_s ,test_cnt ,motor_id ); // Print new ADC data
			prev_param_s = adc_param_s; // Store ADC parameters

			// Check for end-of-test condition ( all phases max-value )
			if (MAX_ADC_VAL == adc_param_s.vals[ADC_PHASE_A])
			{
				if (MAX_ADC_VAL == adc_param_s.vals[ADC_PHASE_B])
				{
					if (MAX_ADC_VAL == adc_param_s.vals[ADC_PHASE_C])
					{
						do_loop = 0; // Error flag signals end-of-loop
					} // if (MAX_ADC_VAL == adc_param_s.vals[ADC_PHASE_C])
				} // if (MAX_ADC_VAL == adc_param_s.vals[ADC_PHASE_B])
			} // if (MAX_ADC_VAL == adc_param_s.vals[ADC_PHASE_A])

		} // if (prev_ang != adc_param_s.theta)

		// Pace ADC Client requests, so as NOT to overload ADC server
		chronometer when timerafter(time_val + time_inc) :> time_val;
	} // while( loop )

	chronometer when timerafter(time_val + MILLI_SEC) :> time_val; // Wait for Test Generation to End
} // disp_motor_adc_client_data
/*****************************************************************************/
void disp_all_adc_client_data( // Display ADC results for all motors
	streaming chanend c_adc[] // Array of ADC channel between Client and Server
)
{
	int motor_cnt; // counts Motors 


	// Loop through motors, so we can print results serially
	for (motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++)
	{
		disp_motor_adc_client_data( motor_cnt ,c_adc[motor_cnt] );
	} // for motor_cnt

	acquire_lock(); // Acquire Display Mutex
	printstr( "                             " );
	printstrln( "Test Check Ends " );
	release_lock(); // Release Display Mutex
} // disp_all_adc_client_data
/*****************************************************************************/
