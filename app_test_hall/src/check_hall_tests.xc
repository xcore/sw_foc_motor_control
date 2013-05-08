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

#include "check_hall_tests.h"

/*****************************************************************************/
static void init_hall_params( // Initialise HALL parameter structure
	HALL_PARAM_TYP &hall_param_s,	// Reference to structure containing HALL data
	int motor_id // Motors Identifier
)
{
	hall_param_s.hall_val = 0;
	hall_param_s.err = 0; // No Errors
} // print_hall_data
/*****************************************************************************/
static void print_hall_data( // Print elements of HALL structure
	HALL_PARAM_TYP &hall_param_s,	// Reference to structure containing HALL data
	int test_id, // Test Identifier
	int motor_id // Motors Identifier
)
{
	acquire_lock(); // Acquire Display Mutex
	printstr( "                             " );
	printint( motor_id );
	printstr( ":  E=" );
	printint( hall_param_s.err );
	printstr( "  H=" );
	printint( hall_param_s.hall_val );
	printcharln(' ');
	release_lock(); // Release Display Mutex
} // print_hall_data
/*****************************************************************************/
static void disp_motor_hall_client_data( // Display HALL results for one motor
	int motor_id, // Motors Identifier
	streaming chanend c_hall // HALL channel between Client and Server
)
{
	HALL_PARAM_TYP hall_param_s;	// Structure containing HALL parameters
	int test_cnt;
	int prev_hall = 0; // Set to Illegal value
	timer chronometer; // XMOS timer
	unsigned time_val;   // time value
	unsigned time_inc;   // time increment
	int do_loop = 1;   // Flag set until loop-end condition found 


	init_hall_params( hall_param_s ,motor_id ); // Initialise HALL parameter structure
	
	chronometer :> time_val; // Get start time
	chronometer when timerafter(time_val + (MICRO_SEC << 1)) :> time_val; // Wait for Test Generation to Start

	acquire_lock(); // Acquire Display Mutex
	printstr( "                             " );
	printstr("Start Checks For Motor_"); printintln( motor_id ); 
	release_lock(); // Release Display Mutex

	time_inc = (40 * MICRO_SEC); // time increment between HALL Client requests. (Typical value when using real motor)

	// Loop until end condition found
	while( do_loop )
	{
		foc_hall_get_parameters( hall_param_s ,c_hall ); // Client function under test

		// Check for difference in Angle supplied by Client
		if (prev_hall != hall_param_s.hall_val)
		{
			print_hall_data( hall_param_s ,test_cnt ,motor_id ); // Print new HALL data
			prev_hall = hall_param_s.hall_val; // Store angle

			if (hall_param_s.err)
			{
				do_loop = 0; // Error flag signals end-of-loop
			} // if (hall_param_s.err)
		} // if (prev_hall != hall_param_s.hall_val)

		// Pace HALL Client requests, so as NOT to overload HALL server
		chronometer when timerafter(time_val + time_inc) :> time_val;
	} // while( loop )

	chronometer when timerafter(time_val + MILLI_SEC) :> time_val; // Wait for Test Generation to End
} // disp_motor_hall_client_data
/*****************************************************************************/
void disp_all_hall_client_data( // Display HALL results for all motors
	streaming chanend c_hall[] // Array of HALL channel between Client and Server
)
{
	int motor_cnt; // counts Motors 


	// Loop through motors, so we can print results serially
	for (motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++)
	{
		disp_motor_hall_client_data( motor_cnt ,c_hall[motor_cnt] );
	} // for motor_cnt

	acquire_lock(); // Acquire Display Mutex
	printstr( "                             " );
	printstrln( "Test Check Ends " );
	release_lock(); // Release Display Mutex
} // disp_all_hall_client_data
/*****************************************************************************/
