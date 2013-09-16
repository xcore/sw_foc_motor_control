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

#include "watchdog.h"

/*****************************************************************************/
static void init_wd( // Initialise WatchDog circuit (2 chips)
	WD_DATA_TYP &wd_data_s, // Reference to structure containing data for WatchDog circuit
	chanend c_wd[NUMBER_OF_MOTORS], // Array of WatchDog channels
	out port p2_wd // 2-bit port used to control WatchDog circuit (2 chips)
)
{
	unsigned cmd; // WatchDog command from Client
	unsigned pulse_width; // width of pulse in clock ticks
	timer chronometer; // Timer


	wd_data_s.state = WD_UNARMED; // Initialise WatchDog to Un-armed state
	wd_data_s.shared_out = 0; // Clear (Bit_0 and Bit_1 of) data for shared output port 

	// Wait for Initialisation command from Client
	c_wd[1] :> cmd;
	assert (WD_CMD_INIT == cmd); // Un-expected command Received

	// Initialise	WatchDog ...
	chronometer :> wd_data_s.time;	// Get 'enable' time (first tick)

	wd_data_s.shared_out &= (~ENABLE_MASK); // Clear Bit_0 (start of rising Edge for FlipFlop Chip)
	wd_data_s.shared_out |= TICK_MASK;			// Set Bit_1 to switch On WatchDog Chip

	p2_wd <: wd_data_s.shared_out; // NB Switches on WatchDog chip
	
	// FlipFlop pulse width minimum in 3 ns. Therefore, stay low for a micro-second
	chronometer :> pulse_width;
	chronometer when timerafter(pulse_width + MICRO_SEC) :> void;
	
	wd_data_s.shared_out |= ENABLE_MASK; ; // Set Bit_0 (end of rising Edge for FlipFlop Chip)
	p2_wd <: wd_data_s.shared_out; // NB Switches on FlipFlop chip

} // init_wd
/*****************************************************************************/
static void run_unarmed( // Runs WatchDog in UN-armed mode until Motors have warmed-up
	WD_DATA_TYP &wd_data_s, // Reference to structure containing data for WatchDog circuit
	chanend c_wd[NUMBER_OF_MOTORS], // Array of WatchDog channels
	out port p2_wd // 2-bit port used to control WatchDog circuit (2 chips)
)
{
	unsigned cmd; // WatchDog command from Client
	timer chronometer; // Timer


	// Loop while WatchDog in Un-armed state
	while (WD_UNARMED == wd_data_s.state)
	{
		// Wait for one of following events
		select
		{
			// Check for a command from the Client
			case c_wd[1] :> cmd:
				switch(cmd)
				{
					case WD_CMD_DISABLE : // Actively disable the WatchDog circuit
						wd_data_s.state = WD_STOP; // Switch to state where WatchDog stops Motors
					break; // WD_CMD_DISABLE 

					case WD_CMD_TICK : // Check for end of Start-up mode
						wd_data_s.shared_out ^= TICK_MASK; // Toggle Bit_1 (for WatchDog Chip)
						wd_data_s.state = WD_ARMED; // Switch to fully armed WatchDog state
					break; // WD_CMD_TICK

					default :
						assert( 0 == 1 ); // Unexpected Command Received
					break; // default
				} // switch(cmd)
			break; // case c_wd[1] :> cmd:

			// Check if periodic tick required
			case chronometer when timerafter(wd_data_s.time + HALF_TICK_PERIOD) :> wd_data_s.time:
				// Force tick every 0.5 milli-seconds, to keep WatchDog alive until Client supplies ticks
				wd_data_s.shared_out ^= TICK_MASK; // Toggle Bit_1 (for WatchDog Chip)
			break; // case chronometer
		} // select

		p2_wd <: wd_data_s.shared_out; // Send out the new value to the shared port
	} // while(WD_UNARMED == wd_data_s.state)

} // run_unarmed 
/*****************************************************************************/
static void run_wd_armed( // Run WatchDog fully armed until STOP request received
	WD_DATA_TYP &wd_data_s, // Reference to structure containing data for WatchDog circuit
	chanend c_wd[NUMBER_OF_MOTORS], // Array of WatchDog channels
	out port p2_wd // 2-bit port used to control WatchDog circuit (2 chips)
)
{
	unsigned cmd; // WatchDog command from Client


	// Enter active WatchDog mode ...

	// Loop until disabled
	while (WD_STOP != wd_data_s.state)
	{
		// Wait for a command from the Client
		c_wd[1] :> cmd;

		switch(cmd)
		{
			case WD_CMD_DISABLE : // Actively disable the WatchDog circuit
				wd_data_s.state = WD_STOP; // Switch to state where WatchDog stops Motors
			break; // WD_CMD_DISABLE 

			case WD_CMD_TICK : // Toggle WD_TICK pin to keep WatchDog 'alive'
				wd_data_s.shared_out ^= TICK_MASK; // Toggle Bit_1 (for WatchDog Chip)
			break; // WD_CMD_TICK 

			default :
				assert( 1 == 2 ); // Unexpected Command Received
			break; // default
		} // switch(cmd)

		p2_wd <: wd_data_s.shared_out; // Send out the new value to the shared port
	} // while (WD_STOP != wd_data_s.state)

} // run_wd_armed 
/*****************************************************************************/
void foc_loop_do_wd( // Controls WatchDog circuit (2 chips)
	chanend c_wd[NUMBER_OF_MOTORS], // Array of WatchDog channels
	out port p2_wd // 2-bit port used to control WatchDog circuit (2 chips)
)
{
	WD_DATA_TYP wd_data_s; // Structure containing data for WatchDog circuit

//MB~ Once both motors are running, need to update watchdog to monitor both motors. 

	// Initialise WatchDog circuit
	init_wd( wd_data_s ,c_wd ,p2_wd );

	// Keep WatchDog alive but Un-armed until Client supplies ticks
	run_unarmed( wd_data_s ,c_wd ,p2_wd );

	// Client now supplying ticks, so switch to Armed state
	run_wd_armed( wd_data_s ,c_wd ,p2_wd );

	/* NB If this line is reached, then in less than 3.4ms, 
	 * the WatchDog will switch off motor power, 
	 * because this Server is no longer supplying ticks
	 */

} // foc_loop_do_wd
/*****************************************************************************/
