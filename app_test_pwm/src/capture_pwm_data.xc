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

#include "capture_pwm_data.h"

/*****************************************************************************/
void capture_pwm_client_data( // Display PWM results for all motors
	buffered in port:32 p32_tst_hi[], // array of PWM ports (High side)  
	buffered in port:32 p32_tst_lo[], // array of PWM ports (Low side)
	out port p8_tst_sync, // NB Dummy output port
	clock comm_clk, // Common clock for all test ports
	streaming chanend c_chk[], // Array of channel for transmitting PWM data to test checker
	chanend c_adc_trig // ADC trigger channel 
)
{
	PWM_PORT_TYP port_data; // Buffers for PWM data
	PORT_TIME_TYP port_time; // Time when port read
	unsigned char cntrl_token; // control token
  PWM_PHASE_ENUM phase_cnt; // PWM phase counter

	unsigned curr_pins; // current value on input pins
	unsigned prev_pins = 0x55555555; // Initialise previous input pins to impossible value
	unsigned chan_off = 0; // offset into channel array


	// Configure all ports to use a common clock. Then all port timers with be synchronised
	for (phase_cnt=0; phase_cnt<NUM_PWM_PHASES; phase_cnt++)
	{ 
	  configure_in_port( p32_tst_hi[phase_cnt] ,comm_clk );
	  configure_in_port( p32_tst_lo[phase_cnt] ,comm_clk );
	} // for phase_cnt

  configure_out_port( p8_tst_sync ,comm_clk ,0 );
  start_clock(comm_clk);


	// Loop forever
	while (1)
	{
		if (TEST_PHASE == PWM_HI_LEG)
		{ // Capture PWM data from High-leg
#pragma ordered // If multiple cases fire at same time, service top-most first
			select
			{
				case p32_tst_hi[TEST_PHASE] :> curr_pins @ port_time :
					// For PWM Phase/Leg under test: capture next 32-bits of data

					if (curr_pins != prev_pins)
					{
						port_data.time_off = port_time;
						port_data.pattern = curr_pins;
// acquire_lock(); printuint(chan_off); printstr("T="); printuint(port_time); printstr(" P="); printhexln(curr_pins); release_lock(); //MB~
			
						// NB We need an array of channels, as one channel does NOT get read quick enough (in checker)
						c_chk[chan_off] <: port_data; // Send PWM data to checker
			
						// Update circular channel offset
						chan_off++; // Increment channel counter
						chan_off = (((unsigned)chan_off) & CHAN_MASK); // Wrap offset into range [0..CHAN_MASK];
			
						prev_pins = curr_pins;
					} // if (curr_pins != prev_pins)
				break;
	
				case inct_byref( c_adc_trig, cntrl_token ):
					// Capture time of ADC trigger

					p8_tst_sync <: (unsigned char)chan_off @ port_data.time_off; // Ouput dummy data to get timestamp
					port_data.pattern = ADC_PATN; // NB This is a special pattern used to signal to the checker

					// NB We need an array of channels, as one channel does NOT get read quick enough (in checker)
					c_chk[chan_off] <: port_data; // Send PWM data to checker
			
					// Update circular channel offset
					chan_off++; // Increment channel counter
					chan_off = (((unsigned)chan_off) & CHAN_MASK); // Wrap offset into range [0..CHAN_MASK];
				break;
			} // select
		} // if (TEST_PHASE == PWM_HI_LEG)
#ifdef MB
		else
		{ // Capture PWM data from Low-leg
			case p32_tst_lo[TEST_PHASE] :> curr_pins @ port_time;
		} // else !(TEST_PHASE == PWM_HI_LEG)
#endif //MB~
		
	}	// while (1)

} // capture_pwm_client_data
/*****************************************************************************/
