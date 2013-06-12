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
	streaming chanend c_chk[] // Array of channel for transmitting PWM data to test checker
)
{
	PWM_PORT_TYP port_data; // Buffers for PWM data
	PORT_TIME_TYP port_time; // Time when port read

	unsigned curr_pins; // current value on input pins
	unsigned prev_pins = 0x55555555; // Initialise previous input pins to impossible value
	unsigned chan_off = 0; // offset into channel array


	// Loop forever
	while (1)
	{
		// For PWM Phase/Leg under test: capture next 32-bits of data
		if (TEST_PHASE == PWM_HI_LEG)
		{ // Capture PWM data from High-leg
			p32_tst_hi[TEST_PHASE] :> curr_pins @ port_time;
		} // if (TEST_PHASE == PWM_HI_LEG)
		else
		{ // Capture PWM data from Low-leg
			p32_tst_lo[TEST_PHASE] :> curr_pins @ port_time;
		} // else !(TEST_PHASE == PWM_HI_LEG)
		
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
	}	// while (1)

} // capture_pwm_client_data
/*****************************************************************************/
