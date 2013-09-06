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

#include "test_qei_common.h"

/*****************************************************************************/
static void init_error_component( // Initialise QEI Test data for error_status test vector component
	VECT_COMP_TYP &vect_comp_s, // Reference to structure of common data for one test vector component
	int inp_states, // No. of states for this test vector component
	const char inp_name[] // input name for current test vector component
)
{
	// Check enough room for all states
	if (MAX_COMP_STATES < inp_states)
	{
		acquire_lock(); // Acquire Display Mutex
		printstrln( "ERROR: MAX_COMP_STATES < inp_states, Update value for MAX_COMP_STATES in test_qei_common.h" );
		release_lock(); // Release Display Mutex
		assert(0 == 1); // Force Abort
	} // if (MAX_COMP_STATES < inp_states)

	vect_comp_s.num_states = inp_states; // Assign number of states for current component
	safestrcpy( vect_comp_s.comp_name.str ,inp_name );

	safestrcpy( vect_comp_s.state_names[QEI_ERR_OFF].str ," No_Err" );
	safestrcpy( vect_comp_s.state_names[QEI_ERR_ON].str ," Err_On" );

	// Add any new component states here 
} // init_error_component
/*****************************************************************************/
static void init_origin_component( // Initialise QEI Test data for origin-bit test vector component
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
		printstrln( ": MAX_COMP_STATES < inp_states, Update value for MAX_COMP_STATES in test_qei_common.h" );
		release_lock(); // Release Display Mutex
		assert(0 == 1);
	} // if (MAX_COMP_STATES < inp_states)

	vect_comp_s.num_states = inp_states; // Assign number of states for current component
	safestrcpy( vect_comp_s.comp_name.str ,inp_name );

	safestrcpy( vect_comp_s.state_names[ORIG_OFF].str ," No_Orig" );
	safestrcpy( vect_comp_s.state_names[ORIG_ON].str ," Orig_On" );

	// Add any new component states here 
} // init_origin_component
/*****************************************************************************/
static void init_spin_component( // Initialise QEI Test data for spin-direction test vector component
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
		printstrln( ": MAX_COMP_STATES < inp_states, Update value for MAX_COMP_STATES in test_qei_common.h" );
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
static void init_speed_component( // Initialise QEI Test data for speed test vector component
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
		printstrln( ": MAX_COMP_STATES < inp_states, Update value for MAX_COMP_STATES in test_qei_common.h" );
		release_lock(); // Release Display Mutex
		assert(0 == 1);
	} // if (MAX_COMP_STATES < inp_states)

	vect_comp_s.num_states = inp_states; // Assign number of states for current component
	safestrcpy( vect_comp_s.comp_name.str ,inp_name );

	safestrcpy( vect_comp_s.state_names[ACCEL].str	," Accelerating" );
	safestrcpy( vect_comp_s.state_names[FAST].str		," Fast-steady " );
	safestrcpy( vect_comp_s.state_names[DECEL].str	," Decelerating" );
	safestrcpy( vect_comp_s.state_names[SLOW].str		," Slow-steady " );

	// Add any new component states here 
} // init_speed_component
/*****************************************************************************/
static void init_control_component( // Initialise QEI Test data for Control/Communications test vector component
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
		printstrln( ": MAX_COMP_STATES < inp_states, Update value for MAX_COMP_STATES in test_qei_common.h" );
		release_lock(); // Release Display Mutex
		assert(0 == 1);
	} // if (MAX_COMP_STATES < inp_states)

	vect_comp_s.num_states = inp_states; // Assign number of states for current component
	safestrcpy( vect_comp_s.comp_name.str ,inp_name );

	safestrcpy( vect_comp_s.state_names[QUIT].str	,"QUIT " );
	safestrcpy( vect_comp_s.state_names[VALID].str	,"VALID" );
	safestrcpy( vect_comp_s.state_names[SKIP].str		,"SKIP " );

	// Add any new component states here 
} // init_speed_component
/*****************************************************************************/
void init_common_data( // Initialise Test data
	COMMON_TST_TYP &comm_data_s // Structure containing common test data
)
{
	init_error_component(		comm_data_s.comp_data[ERROR]		,NUM_QEI_ERRS		," Status" );
	init_origin_component(	comm_data_s.comp_data[ORIGIN]	,NUM_QEI_ORIGS	," Origin" );
	init_spin_component(		comm_data_s.comp_data[SPIN]		,NUM_QEI_SPINS	,"  Spin " );
	init_speed_component(		comm_data_s.comp_data[SPEED]		,NUM_QEI_SPEEDS	," Speed " );
	init_control_component(		comm_data_s.comp_data[CNTRL]	,NUM_QEI_CNTRLS	," Comms." );

	// Add any new test vector components here

	safestrcpy( comm_data_s.prefix[DISP_INP_GEN].str ,"" );
	safestrcpy( comm_data_s.prefix[DISP_INP_CHK].str ,"                                             " );

	// Add any new display input prefixes here 

} // init_common_data
/*****************************************************************************/
