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

// QEI ports
on tile[MOTOR_TILE]: port in p4_qei[NUMBER_OF_MOTORS] = { PORT_M1_ENCODER ,PORT_M2_ENCODER };

// Test ports (Borrowed from Hall Sensor)
on tile[MOTOR_TILE]: port out p4_tst[NUMBER_OF_MOTORS] = { PORT_M1_HALLSENSOR ,PORT_M2_HALLSENSOR };

/*****************************************************************************/
int main ( void ) // Program Entry Point
{
	streaming chan c_qei[NUMBER_OF_MOTORS];


	par
	{	// NB All cores are run on one tile so that all cores use the same clock frequency (250 MHz)
		on tile[MOTOR_TILE] : 
		{
		  init_locks(); // Initialise Mutex for display

			par
			{
				gen_all_qei_test_data( p4_tst ); // Generate test data
		
				foc_qei_do_multiple( c_qei, p4_qei ); // Server function under test
		
				disp_all_qei_client_data( c_qei ); // Check results using QEI Client
			} // par
		
		  free_locks(); // Free Mutex for display
		} // on tile[MOTOR_TILE] : 
	} // par 

	return 0;
} // main
/*****************************************************************************/
// main.xc
