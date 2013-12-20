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

#include "shared_io.h"

/*****************************************************************************/
static void update_speed_control( // Updates the speed control loop
	chanend c_speed[], // speed channel
	CMD_IO_ENUM cmd_id // Command identifier
)
{
	int motor_cnt; // motor counter

	// Loop through motors
	for (motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++) 
	{
		c_speed[motor_cnt] <: cmd_id;
	} // for motor_cnt
} // update_speed_control 
/*****************************************************************************/
void foc_display_shared_io_manager( // Manages the display, buttons and shared ports.
	chanend c_speed[], // Display channel between Client & Server
	LCD_INTERFACE_TYP &lcd_interface_s, // Reference to structure containing data for LCD display 
	in port btns, // Input port buttons
	out port leds // Output port LED's
)
{
	char my_string[50]; // array of display characters
	int new_meas_speed[NUMBER_OF_MOTORS]; // Array containing new measured speeds
	int old_meas_speed[NUMBER_OF_MOTORS]; // Array containing old measured speeds
	int fault[NUMBER_OF_MOTORS]; // Array containing motor fault ids
	int new_req_speed[NUMBER_OF_MOTORS]; // new requested speed
	int old_req_speed[NUMBER_OF_MOTORS] = { 1000 ,-1000 }; // old requested speed
	int speed_change; // flag set when new speed parameters differ from old
	int cur_speed; // current speed
	unsigned int btn_en = 0; // button debounce counter
	int motor_cnt; // motor counter
	int toggle = 1; // indicates motor spin direction 
	unsigned btns_val; // value that encodes state of buttons
	int err_cnt; // error counter

	timer timer_10Hz; // 10Hz timer
	timer timer_30ms; // 30ms timer
	unsigned int time_10Hz_val; // 10Hz timer value
	unsigned int time_30ms_val; // 30ms timer value


	// Initialise array of old measured speeds
	for (motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++)
	{
		old_meas_speed[motor_cnt]=0;
	} // for motor_cnt

	leds <: 0;

	/* Initiate the LCD ports */
	lcd_ports_init( lcd_interface_s );

	/* Output the default value to the port */
	lcd_interface_s.p_core1_shared <:0;

	/* Get the initial time value */
	timer_10Hz :> time_10Hz_val;

	/* Loop forever processing commands */
	while (1)
	{
		select
		{
		/* Timer event at 10Hz */
			case timer_10Hz when timerafter(time_10Hz_val + 10000000) :> time_10Hz_val:
			{
				speed_change = 0; // Preset to speed change

				/* Get the motor speeds from channels. NB Do this as quickly as possible */
				for (motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++) 
				{
					c_speed[motor_cnt] <: IO_CMD_GET_IQ;
					c_speed[motor_cnt] :> new_meas_speed[motor_cnt];
					c_speed[motor_cnt] :> new_req_speed[motor_cnt];

					c_speed[motor_cnt] <: IO_CMD_GET_FAULT;
					c_speed[motor_cnt] :> fault[motor_cnt];

					// Check for speed change
					if (new_req_speed[motor_cnt] != old_req_speed[motor_cnt])
					{ 
						speed_change = 1; // flag speed change
					} // if (new_req_speed[motor_cnt] != old_req_speed[motor_cnt])

					sprintf( my_string ," SetVeloc%1d:%5d RPM\n" ,motor_cnt ,new_req_speed[motor_cnt] );
					lcd_draw_text_row( my_string ,(2*motor_cnt) ,lcd_interface_s );

					// Filter estimated velocity 
					old_meas_speed[motor_cnt] = (old_meas_speed[motor_cnt] + new_meas_speed[motor_cnt]) >> 1;

					if (fault[motor_cnt]) 
					{
						sprintf(my_string, "  Motor%1d: FAULT = %02d\n" ,motor_cnt ,fault[motor_cnt] );
					} 
					else 
					{
						sprintf(my_string, " EstVeloc%1d:%5d RPM\n" ,motor_cnt ,old_meas_speed[motor_cnt] );
					}

					lcd_draw_text_row( my_string ,(2*motor_cnt + 1) ,lcd_interface_s );

					old_req_speed[motor_cnt] = new_req_speed[motor_cnt]; // Store for next iteration
				} // for motor_cnt

#ifdef DEPRECIATED
				if (speed_change) 
				{
					if (1 == USE_CAN)
					{
						lcd_draw_text_row( "  XMOS Demo 2013: CAN\n" ,0 ,lcd_interface_s );
					} // if (1 == USE_CAN)

					if (1 == USE_ETH)
					{
						lcd_draw_text_row( "  XMOS Demo 2013: ETH\n" ,0 ,lcd_interface_s );
					} // if (1 == USE_ETH)
				} // if (speed_change)
#endif // DEPRECIATED

				if ( btn_en > 0) btn_en--; // decrement switch debouncer
			}
			break; // case timer_10Hz when timerafter(time_10Hz_val + 10000000) :> time_10Hz_val:

			case !btn_en => btns when pinsneq(btns_val) :> btns_val:
				btns_val = (~btns_val & 0x0000000F); // Invert and mask out 4 most LS (active) bits

				// check for button change
				if (btns_val)
				{
					// Decode buttons value
					switch( btns_val )
					{
						case 1 : // Increase the speed, by the increment
							err_cnt = 0; // Valid button value so clear error count
							leds <: 1;

							update_speed_control( c_speed ,IO_CMD_INC_SPEED ); // Increase speed
						break; // case 1
	
						case 2 : // Decrease the speed, by the increment
							err_cnt = 0; // Valid button value so clear error count
							leds <: 2;

							update_speed_control( c_speed ,IO_CMD_DEC_SPEED ); // Decrease speed
						break; // case 2
	
						case 8 : // Change direction of spin
							err_cnt = 0; // Valid button value so clear error count
							leds <: 4;
			
							update_speed_control( c_speed ,IO_CMD_FLIP_SPIN ); // Reverse spin direction
						break; // case 8
	
				    default: // btns_val unsupported
							assert(err_cnt < ERR_LIM); // Check for persistant error

							err_cnt++; // Increment error count
				    break;
					} // switch( btns_val )

					btn_en = 2;	// Set the debouncer
				} // if (btns_val)
				else
				{	// No change
					err_cnt = 0; // Valid button value so clear error count
					leds <: 0;
				} // else !(btns_val)

			break; // case !btn_en => btns when pinsneq(btns_val) :> btns_val:

/* JMB 21-NOV-2012  NOT required: Also improves response to push buttons!-)
 *		default:
 *		break;
 */
		} // select
	} // while (1)
} // display_shared_io_manager
/*****************************************************************************/

