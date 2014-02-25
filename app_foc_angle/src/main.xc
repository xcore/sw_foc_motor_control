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

// LCD & Button Ports
on tile[INTERFACE_TILE]: LCD_INTERFACE_TYP lcd_ports = { PORT_SPI_CLK, PORT_SPI_MOSI, PORT_SPI_SS_DISPLAY, PORT_SPI_DSA };
on tile[INTERFACE_TILE]: in port p_btns = PORT_BUTTONS;
on tile[INTERFACE_TILE]: out port p_leds = PORT_LEDS;

// Clock Blocks
on tile[MOTOR_TILE]: clock pwm_clk = XS1_CLKBLK_3;
on tile[MOTOR_TILE]: clock qei_clks[NUMBER_OF_MOTORS] = { XS1_CLKBLK_4 ,XS1_CLKBLK_5 };

// PWM ports
on tile[MOTOR_TILE]: in port p16_adc_sync[NUMBER_OF_MOTORS] = { XS1_PORT_16A ,XS1_PORT_16B }; // NB Dummy port
on tile[MOTOR_TILE]: buffered out port:32 pb32_pwm_hi[NUMBER_OF_MOTORS][NUM_PWM_PHASES] 
	= {	{PORT_M1_HI_A, PORT_M1_HI_B, PORT_M1_HI_C} ,{PORT_M2_HI_A, PORT_M2_HI_B, PORT_M2_HI_C} };
on tile[MOTOR_TILE]: buffered out port:32 pb32_pwm_lo[NUMBER_OF_MOTORS][NUM_PWM_PHASES] 
	= {	{PORT_M1_LO_A, PORT_M1_LO_B, PORT_M1_LO_C} ,{PORT_M2_LO_A, PORT_M2_LO_B, PORT_M2_LO_C} };

// ADC ports
on tile[MOTOR_TILE]: buffered in port:32 pb32_adc_data[NUM_ADC_DATA_PORTS] = { PORT_ADC_MISOA ,PORT_ADC_MISOB };
on tile[MOTOR_TILE]: out port p1_adc_sclk = PORT_ADC_CLK; // 1-bit port connecting to external ADC serial clock
on tile[MOTOR_TILE]: port p1_ready = PORT_ADC_CONV; // 1-bit port used to as ready signal for pb32_adc_data ports and ADC chip
on tile[MOTOR_TILE]: out port p4_adc_mux = PORT_ADC_MUX; // 4-bit port used to control multiplexor on ADC chip
on tile[MOTOR_TILE]: clock adc_xclk = XS1_CLKBLK_2; // Internal XMOS clock

// Hall ports
on tile[MOTOR_TILE]: port in p4_hall[NUMBER_OF_MOTORS] = { PORT_M1_HALLSENSOR ,PORT_M2_HALLSENSOR };

// QEI ports
on tile[MOTOR_TILE]: buffered QEI_PORT in pb4_qei[NUMBER_OF_MOTORS] = { PORT_M1_ENCODER, PORT_M2_ENCODER };

// Watchdog port
on tile[INTERFACE_TILE]: out port p2_i2c_wd = PORT_WATCHDOG; // 2-bit port used to control WatchDog chip

#if (USE_XSCOPE)
/*****************************************************************************/
void xscope_user_init()
{
	xscope_register( 16
		,XSCOPE_CONTINUOUS, "mId_0", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "mId_1", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "mIq_0", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "mIq_1", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "mVel0", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "mVel1", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "thet0", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "thet1", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "nVel0", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "nVel1", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "xVel0", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "xVel1", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "seVd0", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "seVd1", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "seVq0", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "seVq1", XSCOPE_INT , "n"
/*
		,XSCOPE_CONTINUOUS, "sErr0", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "sErr1", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "pVel0", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "pVel1", XSCOPE_INT , "n"

		,XSCOPE_CONTINUOUS, "pidVE", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "tarIq", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "mId_0", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "sumVE", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "per_Q", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "per_F", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "Eficny", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "set_Vd", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "est_Ih", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "thetaT", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "set_Vq", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "sum_Id", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "sum_Iq", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "set_Vq", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "set_Vd", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "targ_Iq", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "pwm_alf", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "pwm_bet", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "Iq_est", XSCOPE_INT , "n"

		,XSCOPE_CONTINUOUS, "pid_vel", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "req_vel", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "pid_Iq", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "targ_Iq", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "hall_0", XSCOPE_INT , "n"

		,XSCOPE_CONTINUOUS, "rev_cnt", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "p_err", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "s_err", XSCOPE_INT , "n"
*/
	); // xscope_register 

	xscope_config_io( XSCOPE_IO_BASIC ); // Enable XScope printing
} // xscope_user_init
#endif // (USE_XSCOPE)

/*****************************************************************************/
int main ( void ) // Program Entry Point
{
	chan c_speed[NUMBER_OF_MOTORS];
	chan c_commands; // chan c_commands[NUMBER_OF_MOTORS]; //MB~ Hack till 'XTAG commands' implemented
	chan c_wd[NUMBER_OF_MOTORS]; // WatchDog channels
	chan c_pwm2adc_trig[NUMBER_OF_MOTORS];
	chan c_pwm[NUMBER_OF_MOTORS];
	streaming chan c_hall[NUMBER_OF_MOTORS];
	streaming chan c_qei[NUMBER_OF_MOTORS];
	streaming chan c_adc_cntrl[NUMBER_OF_MOTORS];


	par
	{
		on tile[INTERFACE_TILE] :
		{
		  init_locks(); // Initialise Mutex for display

			par {
				foc_display_shared_io_manager( c_speed ,lcd_ports ,p_btns, p_leds );
		
				/* NB Ideally WatchDog Server Core should be on Tile most likely to Hang,
				 * However, unfortunately p2_i2c_wd port is on INTERFACE_TILE
				 */
				foc_loop_do_wd( c_wd, p2_i2c_wd );
			} // par

		  free_locks(); // Free Mutex for display
		} // on tile[INTERFACE_TILE]

		on tile[MOTOR_TILE] : 
		{
		  init_locks(); // Initialise Mutex for display

			// Configure PWM ports to run from common clock
			foc_pwm_config( pb32_pwm_hi ,pb32_pwm_lo ,p16_adc_sync ,pwm_clk );

			// Configure QEI ports to run from common clock
			foc_qei_config( pb4_qei ,qei_clks );

			par {
				// Loop through all motors
				par (int motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++)
				{
					run_motor( motor_cnt ,c_wd[motor_cnt]  ,c_pwm[motor_cnt] ,c_hall[motor_cnt] ,c_qei[motor_cnt] 
						,c_adc_cntrl[motor_cnt] ,c_speed[motor_cnt] ,c_commands ); //MB~ Was ,c_commands[motor_cnt] );
		
					foc_pwm_do_triggered( motor_cnt ,c_pwm[motor_cnt] ,pb32_pwm_hi[motor_cnt] ,pb32_pwm_lo[motor_cnt] 
						,c_pwm2adc_trig[motor_cnt] ,p16_adc_sync[motor_cnt] );
				} // par motor_cnt
	
				foc_qei_do_multiple( c_qei ,pb4_qei );
		
				foc_hall_do_multiple( c_hall ,p4_hall );
		
				foc_adc_7265_triggered( c_adc_cntrl ,c_pwm2adc_trig ,pb32_adc_data ,adc_xclk ,p1_adc_sclk ,p1_ready ,p4_adc_mux );
			} // par

		  free_locks(); // Free Mutex for display
		} // on tile[MOTOR_TILE]
	} // par

	return 0;
} // main
/*****************************************************************************/
// main.xc
