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

// Test ports
on tile[INTERFACE_TILE]: port out p4_tst[NUMBER_OF_MOTORS] = { PORT_ETH_TXD ,PORT_WATCHDOG };

#if (USE_XSCOPE)
/*****************************************************************************/
void xscope_user_init()
{
	xscope_register( 6
		,XSCOPE_CONTINUOUS, "m_theta", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "m_veloc", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "rev_cnt", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "qei_A", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "qei_B", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "qei_I", XSCOPE_INT , "n"
	); // xscope_register 

	xscope_config_io( XSCOPE_IO_BASIC ); // Enable XScope printing
} // xscope_user_init
/*****************************************************************************/
#endif // (USE_XSCOPE)

/*****************************************************************************/
int main ( void ) // Program Entry Point
{
	streaming chan c_qei[NUMBER_OF_MOTORS];


	par
	{
		on tile[INTERFACE_TILE] : gen_all_qei_test_data( p4_tst ); // Generate test data

		on tile[MOTOR_TILE] : foc_qei_do_multiple( c_qei, p4_qei ); // Server function under test

		on tile[MOTOR_TILE] : disp_all_qei_client_data( c_qei ); // Check results
	} // par
	return 0;
} // main
/*****************************************************************************/
// main.xc
