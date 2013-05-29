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

#include "main.h"

// HALL ports
on tile[MOTOR_TILE]: port in p4_hall[NUMBER_OF_MOTORS] = { PORT_M1_HALLSENSOR ,PORT_M2_HALLSENSOR };

// Test ports (Borrowed from QEI Sensor)
on tile[MOTOR_TILE]: port out p4_tst[NUMBER_OF_MOTORS] = { PORT_M1_ENCODER ,PORT_M2_ENCODER };

/*****************************************************************************/
int main ( void ) // Program Entry Point
{
	streaming chan c_hall[NUMBER_OF_MOTORS]; // Channel connecting Client and Server
	streaming chan c_tst; // Channel for sending test vectors from Generator to Checker core


	par
	{	// NB All cores are run on one tile so that all cores use the same clock frequency (100 MHz)
		on tile[MOTOR_TILE] : 
		{
		  init_locks(); // Initialise Mutex for display

			par
			{
				gen_all_hall_test_data( c_tst ,p4_tst ); // Generate test data
		
				foc_hall_do_multiple( c_hall, p4_hall ); // Server function under test
		
				check_all_hall_client_data( c_tst ,c_hall ); // Check results using Hall Client
			} // par
		
		  free_locks(); // Free Mutex for display
		} // on tile[MOTOR_TILE] : 
	} // par 

	return 0;
} // main
/*****************************************************************************/
// main.xc
