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
	streaming chanend c_chk // Channel for transmitting PWM data to test checker
)
{
	PWM_PORT_TYP port_data_bufs[NUM_PORT_BUFS]; // Set of buffers for PWM data for each PWM-leg
	PORT_TIME_TYP port_time; // Time when port read

	unsigned curr_pins; // current value on input pins
	unsigned prev_pins = 0x55555555; // Initialise previous input pins to impossible value
	int write_cnt = 0; // No of buffer writes
	int read_cnt = 0; // No of buffer reads
	unsigned write_off = 0; // buffer write offset
	unsigned read_off = 0; // buffer read offset


	// Loop forever
//MB~	while (1)
	while (write_cnt < 20)
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
			port_data_bufs[write_off].time_off = port_time;
			port_data_bufs[write_off].pattern = curr_pins;
// acquire_lock(); printuint(write_off); printstr("T="); printuint(port_time); printstr(" P="); printhexln(curr_pins); release_lock(); //MB~

			// Update circular buffer offset
			write_cnt++; // Increment write counter
			write_off = (((unsigned)write_cnt) & PORT_BUF_MASK); // Wrap offset into range [0..PORT_BUF_MASK];

			prev_pins = curr_pins;
		} // if (curr_pins != prev_pins)

#ifdef MB
		else
		{
			assert(write_cnt <= (read_cnt + NUM_PORT_BUFS)); // Check we have enough input buffers
	
			// Check if any new data to read
			if (write_cnt > read_cnt)
			{
				c_chk <: port_data_bufs[read_off]; // Send PWM data to checker

				// Update circular buffer offset
				read_cnt++; // Increment read counter
				read_off = (((unsigned)read_cnt) & PORT_BUF_MASK); // Wrap offset into range [0..PORT_BUF_MASK];
			} // if (write_cnt > read_cnt)
		} // else !(curr_pins != prev_pins)
#endif //MB~
	}	// while (1)

	for(read_cnt=0; read_cnt<write_cnt; read_cnt++)
	{
		c_chk <: port_data_bufs[read_cnt]; // Send PWM data to checker
	} // for(read_cnt=0; read_cnt<write_cnt; read_cnt++)
} // capture_pwm_client_data
/*****************************************************************************/
