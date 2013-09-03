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

//CAN and ETH reset port
on tile[INTERFACE_TILE] : out port p_shared_rs = PORT_SHARED_RS;

// PWM ports
on tile[MOTOR_TILE]: buffered out port:32 pb32_pwm_hi[NUMBER_OF_MOTORS][NUM_PWM_PHASES] 
	= {	{PORT_M1_HI_A, PORT_M1_HI_B, PORT_M1_HI_C} ,{PORT_M2_HI_A, PORT_M2_HI_B, PORT_M2_HI_C} };
on tile[MOTOR_TILE]: buffered out port:32 pb32_pwm_lo[NUMBER_OF_MOTORS][NUM_PWM_PHASES] 
	= {	{PORT_M1_LO_A, PORT_M1_LO_B, PORT_M1_LO_C} ,{PORT_M2_LO_A, PORT_M2_LO_B, PORT_M2_LO_C} };
on tile[MOTOR_TILE]: clock pwm_clk[NUMBER_OF_MOTORS] = { XS1_CLKBLK_5 ,XS1_CLKBLK_4 };
on tile[MOTOR_TILE]: in port p16_adc_sync[NUMBER_OF_MOTORS] = { XS1_PORT_16A ,XS1_PORT_16B }; // NB Dummy port

// ADC ports
on tile[MOTOR_TILE]: buffered in port:32 pb32_adc_data[NUM_ADC_DATA_PORTS] = { PORT_ADC_MISOA ,PORT_ADC_MISOB }; 
on tile[MOTOR_TILE]: out port p1_adc_sclk = PORT_ADC_CLK; // 1-bit port connecting to external ADC serial clock
on tile[MOTOR_TILE]: port p1_ready = PORT_ADC_CONV; // 1-bit port used to as ready signal for pb32_adc_data ports and ADC chip
on tile[MOTOR_TILE]: out port p4_adc_mux = PORT_ADC_MUX; // 4-bit port used to control multiplexor on ADC chip
on tile[MOTOR_TILE]: clock adc_xclk = XS1_CLKBLK_2; // Internal XMOS clock

// Hall ports
on tile[MOTOR_TILE]: port in p4_hall[NUMBER_OF_MOTORS] = { PORT_M1_HALLSENSOR ,PORT_M2_HALLSENSOR };

// QEI ports
on tile[MOTOR_TILE]: buffered in port:4 pb4_qei[NUMBER_OF_MOTORS] = { PORT_M1_ENCODER, PORT_M2_ENCODER };

// Watchdog port
on tile[INTERFACE_TILE]: out port p2_i2c_wd = PORT_WATCHDOG; // 2-bit port used to control WatchDog chip

#if (USE_ETH)
	// These intializers are taken from the ethernet_board_support.h header for
	// XMOS dev boards. If you are using a different board you will need to
	// supply explicit port structure intializers for these values
	ethernet_xtcp_ports_t xtcp_ports =
  {	on ETHERNET_DEFAULT_TILE: OTP_PORTS_INITIALIZER,
   														ETHERNET_DEFAULT_SMI_INIT,
   														ETHERNET_DEFAULT_MII_INIT_lite,
   														ETHERNET_DEFAULT_RESET_INTERFACE_INIT
	};

	// IP Config - change this to suit your network.  Leave with all 0 values to use DHCP/AutoIP
	xtcp_ipconfig_t ipconfig = 
	{	{ 0, 0, 0, 0 }, // ip address (eg 192,168,0,2)
		{ 0, 0, 0, 0 }, // netmask (eg 255,255,255,0)
		{ 0, 0, 0, 0 } // gateway (eg 192,168,0,1)
	};
#endif // (USE_ETH)

#if (USE_CAN)
	on tile[INTERFACE_TILE] : clock p_can_clk = XS1_CLKBLK_4;
	on tile[INTERFACE_TILE] : buffered in port:32 pb32_can_rx = PORT_CAN_RX;
	on tile[INTERFACE_TILE] : port p_can_tx = PORT_CAN_TX;
#endif // (USE_CAN)

#if (USE_XSCOPE)
/*****************************************************************************/
void xscope_user_init()
{
	xscope_register( 9
		,XSCOPE_CONTINUOUS, "SERVER", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "CLIENT", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "adc_c", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "set_Vq", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "pid_vel", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "req_vel", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "est_Iq", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "pid_Iq", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "targ_Iq", XSCOPE_INT , "n"
/*
		,XSCOPE_CONTINUOUS, "hall_0", XSCOPE_INT , "n"

		,XSCOPE_CONTINUOUS, "rev_cnt", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "p_err", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "s_err", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "pwm_A", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "pwm_B", XSCOPE_INT , "n"
		,XSCOPE_CONTINUOUS, "pwm_C", XSCOPE_INT , "n"
*/
	); // xscope_register 

	xscope_config_io( XSCOPE_IO_BASIC ); // Enable XScope printing
} // xscope_user_init
/*****************************************************************************/
#endif // (USE_XSCOPE)

/*****************************************************************************/
int main ( void ) // Program Entry Point
{
	chan c_wd; // WatchDog Channel connecting Client & Server 
	chan c_speed[NUMBER_OF_MOTORS];
	chan c_commands[NUMBER_OF_MOTORS];
	chan c_pwm2adc_trig[NUMBER_OF_MOTORS];
	chan c_pwm[NUMBER_OF_MOTORS];
	streaming chan c_hall[NUMBER_OF_MOTORS];
	streaming chan c_qei[NUMBER_OF_MOTORS];
	streaming chan c_adc_cntrl[NUMBER_OF_MOTORS];

#if (USE_ETH)
	chan c_ethernet[1]; // NB Need to declare an array of 1 element, because ethernet_xtcp_server() expects array reference 
#endif // (USE_ETH)

#if (USE_CAN)
	chan c_rxChan, c_txChan;
#endif // (USE_CAN)

	par
	{
#if (USE_ETH)
		// MB~ WARNING: Ethernet not yet tested
		on ETHERNET_DEFAULT_TILE: foc_comms_init_eth( xtcp_ports ,ipconfig ,c_ethernet ); // Ethernet & TCP/IP server core
		on tile[INTERFACE_TILE] : foc_comms_do_eth( c_commands, c_ethernet[0] ); // Core to extract Motor commands from ethernet commands
#endif // (USE_ETH)

#if (USE_CAN)
		// MB~ WARNING: CAN not yet tested
		on tile[INTERFACE_TILE] : foc_comms_init_can( c_rxChan, c_txChan, p_can_clk, pb32_can_rx, p_can_tx, p_shared_rs ); // Can server core
		on tile[INTERFACE_TILE] : foc_comms_do_can( c_commands, c_rxChan, c_txChan ); // Core to extract Motor commands from CAN commands
#endif // (USE_CAN)

		on tile[INTERFACE_TILE] : foc_display_shared_io_manager( c_speed ,lcd_ports ,p_btns, p_leds );

		/* NB Ideally WatchDog Server Core should be on Tile most likely to Hang,
		 * However, unfortunately p2_i2c_wd port is on INTERFACE_TILE
		 */
		on tile[INTERFACE_TILE] : foc_loop_do_wd( c_wd, p2_i2c_wd );

		on tile[MOTOR_TILE] : 
		{
			run_motor( 0 ,null ,c_pwm[0] ,c_hall[0] ,c_qei[0] ,c_adc_cntrl[0] ,c_speed[0] ,c_commands[0] ); // Special case of 1st Motor
		} // on tile[MOTOR_TILE]

		// Loop through remaining motors
		par (int motor_cnt=1; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++)
			on tile[MOTOR_TILE] : run_motor( motor_cnt ,c_wd ,c_pwm[motor_cnt] ,c_hall[motor_cnt] ,c_qei[motor_cnt] 
				,c_adc_cntrl[motor_cnt] ,c_speed[motor_cnt] ,c_commands[motor_cnt] );

		// Loop through all motors
		par (int motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++)
		{
			on tile[MOTOR_TILE] : foc_pwm_do_triggered( motor_cnt ,c_pwm[motor_cnt] ,pb32_pwm_hi[motor_cnt] ,pb32_pwm_lo[motor_cnt] ,c_pwm2adc_trig[motor_cnt] ,p16_adc_sync[motor_cnt] ,pwm_clk[motor_cnt] );
		}

		on tile[MOTOR_TILE] : foc_qei_do_multiple( c_qei, pb4_qei );

		on tile[MOTOR_TILE] : foc_hall_do_multiple( c_hall ,p4_hall );

		on tile[MOTOR_TILE] : foc_adc_7265_triggered( c_adc_cntrl ,c_pwm2adc_trig ,pb32_adc_data ,adc_xclk ,p1_adc_sclk ,p1_ready ,p4_adc_mux );
	} // par

	return 0;
} // main
/*****************************************************************************/
// main.xc
