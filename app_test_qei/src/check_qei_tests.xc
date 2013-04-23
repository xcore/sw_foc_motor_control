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

#include "check_qei_tests.h"

/*****************************************************************************/
static void print_qei_data( // Print elements of QEI structure
	QEI_PARAM_TYP &qei_data_s,	// Reference to structure containing QEI data
	int test_id, // Test Identifier
	int motor_id // Motors Identifier
)
{
	acquire_lock(); // Acquire Display Mutex
	printstr( "                             " );
	printint( motor_id );
	printstr( ": R=" );
	printint( qei_data_s.rev_cnt);
	printstr( "  A=" );
	printint( qei_data_s.theta );
	printstr( "  V=" );
	printint( qei_data_s.veloc );
	printcharln(' ');
	release_lock(); // Release Display Mutex
} // print_qei_data
/*****************************************************************************/
static void disp_motor_qei_client_data( // Display QEI results for one motor
	int motor_id, // Motors Identifier
	streaming chanend c_qei // QEI channel between Client and Server
)
{
	QEI_PARAM_TYP qei_data_s;	// Structure containing QEI data
	int test_cnt;
	int prev_ang = (QEI_REV_MASK >> 1); // Set to Non-Zero value
	timer chronometer; // XMOS timer
	unsigned time_val;   // time value
	unsigned time_inc;   // time increment
	int end_cnt = 0;   // Count No of end conditions found
	int do_loop = 1;   // Flag set until loop-end condition found 


	chronometer :> time_val; // Get start time
	chronometer when timerafter(time_val + (MICRO_SEC << 1)) :> time_val; // Wait for Test Generation to Start

	acquire_lock(); // Acquire Display Mutex
	printstr( "                             " );
	printstr("Start Checks For Motor_"); printintln( motor_id ); 
	release_lock(); // Release Display Mutex

	time_inc = (40 * MICRO_SEC); // time increment between QEI Client requests. (Typical value when using real motor)
	qei_data_s.veloc = 9999; // Preset to large value 

	// Loop until end condition found
	while( do_loop )
	{
		foc_qei_get_data( qei_data_s ,c_qei ); // Client function under test

		// Check for difference in Angle supplied by Client
		if (prev_ang != qei_data_s.theta)
		{
			print_qei_data( qei_data_s ,test_cnt ,motor_id ); // Print new QEI data
			prev_ang = qei_data_s.theta; // Store angle

			// Check for loop termination. (speeds between 45 and 55) 
			if (45 < abs(qei_data_s.veloc))
			{
				if (55 > abs(qei_data_s.veloc))
				{
					end_cnt++; // Increment No of end conditions found

					// Check for 3 end conditions
					if (2 < end_cnt)
					{
						do_loop = 0; // Signal end-of-loop
					} // if (2 < end_cnt)
				} // if (57 > abs(qei_data_s.veloc))
				else
				{
					end_cnt = 0; // Reset end condition counter
				} // else !(57 > abs(qei_data_s.veloc))
			} // if (40 < abs(qei_data_s.veloc))
			else
			{
				end_cnt = 0; // Reset end condition counter
			} // else !(40 < abs(qei_data_s.veloc))
		} // if (prev_ang != qei_data_s.theta)

		// Pace QEI Client requests, so as NOT to overload QEI server
		chronometer when timerafter(time_val + time_inc) :> time_val;
	} // while( loop )

	chronometer when timerafter(time_val + MILLI_SEC) :> time_val; // Wait for Test Generation to End
} // disp_motor_qei_client_data
/*****************************************************************************/
void disp_all_qei_client_data( // Display QEI results for all motors
	streaming chanend c_qei[] // Array of QEI channel between Client and Server
)
{
	int motor_cnt; // counts Motors 


	// Loop through motors, so we can print results serially
	for (motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++)
	{
		disp_motor_qei_client_data( motor_cnt ,c_qei[motor_cnt] );
	} // for motor_cnt

	acquire_lock(); // Acquire Display Mutex
	printstr( "                             " );
	printstrln( "Test Check Ends " );
	release_lock(); // Release Display Mutex
} // disp_all_qei_client_data
/*****************************************************************************/
