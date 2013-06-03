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

#include "test_pwm_common.h"

/*****************************************************************************/
static void init_width_component( // Initialise PWM Test data for PWM width test vector component
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
		printstrln( ": MAX_COMP_STATES < inp_states, Update value for MAX_COMP_STATES in test_pwm_common.h" );
		release_lock(); // Release Display Mutex
		assert(0 == 1);
	} // if (MAX_COMP_STATES < inp_states)

	vect_comp_s.num_states = inp_states; // Assign number of states for current component
	safestrcpy( vect_comp_s.comp_name.str ,inp_name );

	safestrcpy( vect_comp_s.state_names[FAST].str		," Fast-width " );
	safestrcpy( vect_comp_s.state_names[SLOW].str		," Slow-width " );

	// Add any new component states here 
} // init_width_component
/*****************************************************************************/
static void init_phase_component( // Initialise PWM Test data for phase test vector component
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
		printstrln( ": MAX_COMP_STATES < inp_states, Update value for MAX_COMP_STATES in test_pwm_common.h" );
		release_lock(); // Release Display Mutex
		assert(0 == 1);
	} // if (MAX_COMP_STATES < inp_states)

	vect_comp_s.num_states = inp_states; // Assign number of states for current component
	safestrcpy( vect_comp_s.comp_name.str ,inp_name );

	safestrcpy( vect_comp_s.state_names[PWM_PHASE_A].str ," Phase_A " );
	safestrcpy( vect_comp_s.state_names[PWM_PHASE_B].str ," Phase_B " );
	safestrcpy( vect_comp_s.state_names[PWM_PHASE_C].str ," Phase_C " );

	// Add any new component states here 
} // init_phase_component
/*****************************************************************************/
static void init_control_component( // Initialise PWM Test data for Control/Communications test vector component
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
		printstrln( ": MAX_COMP_STATES < inp_states, Update value for MAX_COMP_STATES in test_pwm_common.h" );
		release_lock(); // Release Display Mutex
		assert(0 == 1);
	} // if (MAX_COMP_STATES < inp_states)

	vect_comp_s.num_states = inp_states; // Assign number of states for current component
	safestrcpy( vect_comp_s.comp_name.str ,inp_name );

	safestrcpy( vect_comp_s.state_names[QUIT].str	,"QUIT " );
	safestrcpy( vect_comp_s.state_names[VALID].str	,"VALID" );
	safestrcpy( vect_comp_s.state_names[SKIP].str		,"SKIP " );

	// Add any new component states here 
} // init_control_component
/*****************************************************************************/
void print_test_vector( // print test vector details
	COMMON_PWM_TYP &comm_pwm_s, // Reference to structure of common PWM data
	TEST_VECT_TYP inp_vect, // Structure containing current PWM test vector to be printed
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

		if (comp_state < comm_pwm_s.comp_data[comp_cnt].num_states)
		{
			printstr( comm_pwm_s.comp_data[comp_cnt].state_names[comp_state].str ); // Print component status
		} //if (comp_state < comm_pwm_s.comp_data[comp_cnt].num_states)
		else
		{
			printcharln(' ');
			printstr( "ERROR: Invalid state. Found ");
			printint( comp_state );
			printstr( " for component ");
			printstrln( comm_pwm_s.comp_data[comp_cnt].comp_name.str );
			assert(0 == 1); // Force abort
		} //if (comp_state < comm_pwm_s.comp_data[comp_cnt].num_states)
	} // for comp_cnt

	printchar( ':' );
	comp_state = inp_vect.comp_state[CNTRL];  // Get state of Control/Comms. status
	printstrln( comm_pwm_s.comp_data[CNTRL].state_names[comp_state].str ); // Print Control/Comms. status

	release_lock(); // Release Display Mutex
} // print_test_vector
/*****************************************************************************/
void init_common_data( // Initialise PWM Test data
	COMMON_PWM_TYP &comm_pwm_s // Reference to structure of common PWM data
)
{
	init_width_component(		comm_pwm_s.comp_data[WIDTH]		,NUM_PWM_WIDTHS	," Width " );
	init_phase_component(		comm_pwm_s.comp_data[PHASE]		,NUM_PWM_PHASES	," Phase " );
	init_control_component(		comm_pwm_s.comp_data[CNTRL]	,NUM_PWM_CNTRLS	," Comms." );

	// Add any new test vector components here
} // init_common_data
/*****************************************************************************/
