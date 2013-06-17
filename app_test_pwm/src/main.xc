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

// PWM Output ports
on tile[MOTOR_TILE]: buffered out port:32 pb32_pwm_hi[NUM_PWM_PHASES] = {PORT_M1_HI_A, PORT_M1_HI_B, PORT_M1_HI_C};
on tile[MOTOR_TILE]: buffered out port:32 pb32_pwm_lo[NUM_PWM_PHASES] = {PORT_M1_LO_A, PORT_M1_LO_B, PORT_M1_LO_C};
on tile[MOTOR_TILE]: clock pwm_clk= XS1_CLKBLK_5;
on tile[MOTOR_TILE]: in port p16_adc_sync = XS1_PORT_16A; // NB Dummy input port

// Test/Check Input ports
on tile[MOTOR_TILE]: buffered in port:32 pb32_tst_hi[NUM_PWM_PHASES]	= {PORT_M2_HI_A, PORT_M2_HI_B, PORT_M2_HI_C};
on tile[MOTOR_TILE]: buffered in port:32 pb32_tst_lo[NUM_PWM_PHASES]	= {PORT_M2_LO_A, PORT_M2_LO_B, PORT_M2_LO_C};
on tile[MOTOR_TILE]: out port p8_tst_sync = XS1_PORT_8C; // NB Dummy output port

// Test/Check Clocks
on tile[MOTOR_TILE]: clock comm_clk = XS1_CLKBLK_1; // Common clock for all test ports

/*****************************************************************************/
int main ( void ) // Program Entry Point
{
	chan c_pwm2adc_trig;
	chan c_pwm; // Channel connecting Client and Server
	streaming chan c_tst; // Channel for sending test vectors from Generator to Checker core
	streaming chan c_chk[NUM_CHANS]; // Channel for sending PWM data from Capture to Checker core


	par
	{	// NB All cores are run on one tile so that all cores use the same clock frequency (100 MHz)
		on tile[MOTOR_TILE] : 
		{
		  init_locks(); // Initialise Mutex for display


			par
			{
				gen_all_pwm_test_data( c_tst ,c_pwm ); // Generate test data using PWM Client
		
				// Server function under test
				foc_pwm_do_triggered( MOTOR_ID, c_pwm ,pb32_pwm_hi ,pb32_pwm_lo ,c_pwm2adc_trig ,p16_adc_sync ,pwm_clk );
		
				// Capture results
				capture_pwm_client_data( pb32_tst_hi ,pb32_tst_lo ,p8_tst_sync ,comm_clk ,c_chk ,c_pwm2adc_trig );

				check_pwm_client_data( c_chk ,c_tst ); // Check results
			} // par
		
		  free_locks(); // Free Mutex for display
		} // on tile[MOTOR_TILE] : 
	} // par 

	return 0;
} // main
/*****************************************************************************/
// main.xc
