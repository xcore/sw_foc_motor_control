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

#include "test_adc_common.h"

/*****************************************************************************/
static void init_sum_component( // Initialise ADC Test data for zero-sum test vector component
	VECT_COMP_TYP &vect_comp_s, // Reference to structure of common data for one test vector component
	int inp_states, // No. of states for this test vector component
	const char inp_name[] // input name for current test vector component
)
{
	// Check enough room for all states
	if (MAX_COMP_STATES < inp_states)
	{
		acquire_lock(); // Acquire Display Mutex
		printstr( "ERROR on line "); printint( __LINE__ ); printstr( " of "); printstr( __FILE__ );
		printstrln( ": MAX_COMP_STATES < inp_states, Update value for MAX_COMP_STATES in test_adc_common.h" );
		release_lock(); // Release Display Mutex
		assert(0 == 1);
	} // if (MAX_COMP_STATES < inp_states)

	vect_comp_s.num_states = inp_states; // Assign number of states for current component
	safestrcpy( vect_comp_s.comp_name.str ,inp_name );

	safestrcpy( vect_comp_s.state_names[SUM_ON].str ," Sum_On" );
	safestrcpy( vect_comp_s.state_names[NO_SUM].str ," No_Sum" );

	// Add any new component states here
} // init_sum_component
/*****************************************************************************/
static void init_mean_component( // Initialise ADC Test data for zero-mean test vector component
	VECT_COMP_TYP &vect_comp_s, // Reference to structure of common data for one test vector component
	int inp_states, // No. of states for this test vector component
	const char inp_name[] // input name for current test vector component
)
{
	// Check enough room for all states
	if (MAX_COMP_STATES < inp_states)
	{
		acquire_lock(); // Acquire Display Mutex
		printstr( "ERROR on line "); printint( __LINE__ ); printstr( " of "); printstr( __FILE__ );
		printstrln( ": MAX_COMP_STATES < inp_states, Update value for MAX_COMP_STATES in test_adc_common.h" );
		release_lock(); // Release Display Mutex
		assert(0 == 1);
	} // if (MAX_COMP_STATES < inp_states)

	vect_comp_s.num_states = inp_states; // Assign number of states for current component
	safestrcpy( vect_comp_s.comp_name.str ,inp_name );

	safestrcpy( vect_comp_s.state_names[MEAN_ON].str ," Mean_On" );
	safestrcpy( vect_comp_s.state_names[NO_MEAN].str ," No_Mean" );

	// Add any new component states here
} // init_mean_component
/*****************************************************************************/
static void init_spin_component( // Initialise ADC Test data for spin-direction test vector component
	VECT_COMP_TYP &vect_comp_s, // Reference to structure of common data for one test vector component
	int inp_states, // No. of states for this test vector component
	const char inp_name[] // input name for current test vector component
)
{
	// Check enough room for all states
	if (MAX_COMP_STATES < inp_states)
	{
		acquire_lock(); // Acquire Display Mutex
		printstr( "ERROR on line "); printint( __LINE__ ); printstr( " of "); printstr( __FILE__ );
		printstrln( ": MAX_COMP_STATES < inp_states, Update value for MAX_COMP_STATES in test_adc_common.h" );
		release_lock(); // Release Display Mutex
		assert(0 == 1);
	} // if (MAX_COMP_STATES < inp_states)

	vect_comp_s.num_states = inp_states; // Assign number of states for current component
	safestrcpy( vect_comp_s.comp_name.str ,inp_name );

	safestrcpy( vect_comp_s.state_names[ANTI].str ," Anti-Clock" );
	safestrcpy( vect_comp_s.state_names[CLOCK].str ," Clock-Wise" );

	// Add any new component states here
} // init_spin_component
/*****************************************************************************/
static void init_gain_component( // Initialise ADC Test data for Gain test vector component
	VECT_COMP_TYP &vect_comp_s, // Reference to structure of common data for one test vector component
	int inp_states, // No. of states for this test vector component
	const char inp_name[] // input name for current test vector component
)
{
	// Check enough room for all states
	if (MAX_COMP_STATES < inp_states)
	{
		acquire_lock(); // Acquire Display Mutex
		printstr( "ERROR on line "); printint( __LINE__ ); printstr( " of "); printstr( __FILE__ );
		printstrln( ": MAX_COMP_STATES < inp_states, Update value for MAX_COMP_STATES in test_adc_common.h" );
		release_lock(); // Release Display Mutex
		assert(0 == 1);
	} // if (MAX_COMP_STATES < inp_states)

	vect_comp_s.num_states = inp_states; // Assign number of states for current component
	safestrcpy( vect_comp_s.comp_name.str ,inp_name );

	safestrcpy( vect_comp_s.state_names[SMALL].str	," Small" );
	safestrcpy( vect_comp_s.state_names[LARGE].str	," Large" );

	// Add any new component states here
} // init_gain_component
/*****************************************************************************/
static void init_speed_component( // Initialise ADC Test data for speed test vector component
	VECT_COMP_TYP &vect_comp_s, // Reference to structure of common data for one test vector component
	int inp_states, // No. of states for this test vector component
	const char inp_name[] // input name for current test vector component
)
{
	// Check enough room for all states
	if (MAX_COMP_STATES < inp_states)
	{
		acquire_lock(); // Acquire Display Mutex
		printstr( "ERROR on line "); printint( __LINE__ ); printstr( " of "); printstr( __FILE__ );
		printstrln( ": MAX_COMP_STATES < inp_states, Update value for MAX_COMP_STATES in test_adc_common.h" );
		release_lock(); // Release Display Mutex
		assert(0 == 1);
	} // if (MAX_COMP_STATES < inp_states)

	vect_comp_s.num_states = inp_states; // Assign number of states for current component
	safestrcpy( vect_comp_s.comp_name.str ,inp_name );

	safestrcpy( vect_comp_s.state_names[FAST].str ," Fast" );
	safestrcpy( vect_comp_s.state_names[SLOW].str ," Slow" );

	// Add any new component states here
} // init_speed_component
/*****************************************************************************/
static void init_control_component( // Initialise ADC Test data for Control/Communications test vector component
	VECT_COMP_TYP &vect_comp_s, // Reference to structure of common data for one test vector component
	int inp_states, // No. of states for this test vector component
	const char inp_name[] // input name for current test vector component
)
{
	// Check enough room for all states
	if (MAX_COMP_STATES < inp_states)
	{
		acquire_lock(); // Acquire Display Mutex
		printstr( "ERROR on line "); printint( __LINE__ ); printstr( " of "); printstr( __FILE__ );
		printstrln( ": MAX_COMP_STATES < inp_states, Update value for MAX_COMP_STATES in test_adc_common.h" );
		release_lock(); // Release Display Mutex
		assert(0 == 1);
	} // if (MAX_COMP_STATES < inp_states)

	vect_comp_s.num_states = inp_states; // Assign number of states for current component
	safestrcpy( vect_comp_s.comp_name.str ,inp_name );

	safestrcpy( vect_comp_s.state_names[QUIT].str	," QUIT  " );
	safestrcpy( vect_comp_s.state_names[NO_PACE].str ,"NO_PACE" );
	safestrcpy( vect_comp_s.state_names[PACE_ON].str ,"PACE_ON" );

	// Add any new component states here
} // init_speed_component
/*****************************************************************************/
void print_test_vector( // print test vector details
	COMMON_ADC_TYP &comm_adc_s, // Reference to structure of common ADC data
	TEST_VECT_TYP inp_vect, // Structure containing current ADC test vector to be printed
	const char prefix_str[] // prefix string
)
{
	int comp_cnt; // Counter for Test Vector components
	int comp_state; // state of current component of input test vector


	acquire_lock(); // Acquire Display Mutex
	printstr( prefix_str ); // Print prefix string

	// Loop through NON-control test vector components
	for (comp_cnt=1; comp_cnt<NUM_VECT_COMPS; comp_cnt++)
	{
		comp_state = inp_vect.comp_state[comp_cnt];  // Get state of current component

		if (comp_state < comm_adc_s.comp_data[comp_cnt].num_states)
		{
			printstr( comm_adc_s.comp_data[comp_cnt].state_names[comp_state].str ); // Print component status
		} //if (comp_state < comm_adc_s.comp_data[comp_cnt].num_states)
		else
		{
			printcharln(' ');
			printstr( "ERROR: Invalid state. Found ");
			printint( comp_state );
			printstr( " for component ");
			printstrln( comm_adc_s.comp_data[comp_cnt].comp_name.str );
			assert(0 == 1); // Force abort
		} //if (comp_state < comm_adc_s.comp_data[comp_cnt].num_states)
	} // for comp_cnt

	printchar( ':' );
	comp_state = inp_vect.comp_state[CNTRL];  // Get state of Control/Comms. status
	printstrln( comm_adc_s.comp_data[CNTRL].state_names[comp_state].str ); // Print Control/Comms. status

	release_lock(); // Release Display Mutex
} // print_test_vector
/*****************************************************************************/
void init_common_data( // Initialise ADC Test data
	COMMON_ADC_TYP &comm_adc_s // Reference to structure of common ADC data
)
{
	init_sum_component(			comm_adc_s.comp_data[SUM]		,NUM_ADC_SUMS		,"  Sum  " );
	init_mean_component(		comm_adc_s.comp_data[MEAN]	,NUM_ADC_MEANS	,"  Mean " );
	init_spin_component(		comm_adc_s.comp_data[SPIN]	,NUM_ADC_SPINS	,"  Spin " );
	init_gain_component(		comm_adc_s.comp_data[GAIN]	,NUM_ADC_GAINS	,"  Gain " );
	init_speed_component(		comm_adc_s.comp_data[SPEED]	,NUM_ADC_SPEEDS	," Speed " );
	init_control_component(	comm_adc_s.comp_data[CNTRL]	,NUM_ADC_CNTRLS	," Comms." );

	// Add any new test vector components here
} // init_common_data
/*****************************************************************************/
