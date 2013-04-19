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

/*	Order is 00 -> 01 -> 11 -> 10  Clockwise direction
 *	Order is 00 -> 10 -> 11 -> 01  Anti-Clockwise direction
 */
/*****************************************************************************/
static void print_qei_data( // Print elements of QEI structure
	QEI_PARAM_TYP &qei_data_s,	// Reference to structure containing QEI data
	int test_id, // Test Identifier
	int motor_id // Motors Identifier
)
{
		printint( motor_id );
		printstr( ":" );
		printint( test_id );
		printstr( ": A=" );
		printint( qei_data_s.theta );
		printstr( "  V=" );
		printint( qei_data_s.veloc );
		printstr( "  R=" );
		printint( qei_data_s.rev_cnt);
		printcharln(' ');
} // print_qei_data
/*****************************************************************************/
static void disp_motor_qei_client_data( // Display QEI results for one motor
	int motor_id, // Motors Identifier
	streaming chanend c_qei // QEI channel between Client and Server
)
{
	QEI_PARAM_TYP qei_data_s;	// Structure containing QEI data
	int test_cnt;


	for (test_cnt=0; test_cnt<NUM_TESTS; test_cnt++)
	{
		foc_qei_get_data( qei_data_s ,c_qei ); // Client function under test

		print_qei_data( qei_data_s ,test_cnt ,motor_id );
	} // for test_cnt

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

} // disp_all_qei_client_data
/*****************************************************************************/
