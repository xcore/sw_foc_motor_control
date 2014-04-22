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
on tile[MOTOR_TILE]: buffered port:4 in pb4_qei[NUMBER_OF_MOTORS] = { PORT_M1_ENCODER ,PORT_M2_ENCODER };

// Test ports (Borrowed from Hall Sensor)
on tile[MOTOR_TILE]: buffered port:4 out pb4_tst[NUMBER_OF_MOTORS] = { PORT_M1_HALLSENSOR ,PORT_M2_HALLSENSOR };

#if (USE_XSCOPE)
/*****************************************************************************/
void xscope_user_init()
{
	xscope_register( 4
		,XSCOPE_CONTINUOUS, "RevCounter", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "AngularPos", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "Velocity", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "Err_Status", XSCOPE_INT , "n"
	); // xscope_register
} // xscope_user_init
/*****************************************************************************/
#endif // (USE_XSCOPE)

/*****************************************************************************/
int main ( void ) // Program Entry Point
{
	streaming chan c_qei_chk[NUMBER_OF_MOTORS]; // For each motor, a channel connecting QEI Server and Checker cores
	streaming chan c_master[MB_FIXME]; // Array of channel for sending display data to Master_Print core
	streaming chan c_slave[NUM_DISP_SLAVES]; // Array of channels for sending data from Master to Slave print cores
	streaming chan c_gen_chk; // Channel for sending test vectors from Generator to Checker core


	par
	{	// NB All cores are run on one tile so that all cores use the same clock frequency (100 MHz)
		on tile[MOTOR_TILE] :
		{
			COMMON_TST_TYP comm_data_s; // Reference to structure of common test data


			init_common_data( comm_data_s ); // Initialise common test data
  
		  init_locks(); // Initialise Mutex for display

			par
			{
				// Generate test data
				gen_all_qei_test_data( comm_data_s ,c_gen_chk ,c_master[DISP_INP_GEN] ,pb4_tst );

				foc_qei_do_multiple( c_qei_chk, pb4_qei ); // Server function under test

				// Check results using QEI Client
				check_all_qei_client_data( comm_data_s ,c_gen_chk ,c_qei_chk ,c_master[DISP_INP_CHK] );

				master_print( comm_data_s ,c_slave ,c_master ); // Collect and schedule display data

				// Loop through slave Display cores
				par (int disp_cnt=0; disp_cnt<NUM_DISP_SLAVES; disp_cnt++)
				{
					slave_print( comm_data_s ,c_slave[disp_cnt] ,disp_cnt ); // Print display data
				} // par disp_cnt
			} // par

		  free_locks(); // Free Mutex for display

		} // on tile[MOTOR_TILE] :
	} // par

	return 0;
} // main
/*****************************************************************************/
// main.xc
