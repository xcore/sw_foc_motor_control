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
	/* The following structure determines which motors are protected, Edit the array initialiser and checksum to suit
	 * E.g. { { 1 ,1 ,0 } ,2 }; signifies, Motor_0 and Motor_1 protected, Motor_2 UN-protected, 
	 * and a total of 2 motors prrotected
	 */
WD_GUARD_TYP guard_prot_s = { { 1 ,1 } ,2 }; // Structure of guard data, contains motor protection data
//	WD_GUARD_TYP guard_prot_s = { { 0 ,1 } ,1 }; // Structure of guard data, contains motor protection data
	WD_GUARD_TYP guard_init_s; // Structure of guard data, used to check all motors are initialised
	unsigned cmd; // WatchDog command from Client
	unsigned pulse_width; // width of pulse in clock ticks
	unsigned ts1; // Time-Stamp
	int checksum; // motor counter
	int motor_cnt; // motor counter
	timer chronometer; // Timer


	assert (NUMBER_OF_MOTORS < 32); // ERROR: Only supports upto 31 motors

	wd_data_s.protect_s = guard_prot_s; // Load motor protection data

	// Initialise and check protection motor data 
	checksum = 0;
	guard_init_s.num = NUMBER_OF_MOTORS; // Set number of UN-initialised motors
 
 	for (motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++)
	{
		if (wd_data_s.protect_s.guards[motor_cnt]) checksum++;
		guard_init_s.guards[motor_cnt] = 1; // Set guards for UN-initialised motors
	} // for motor_cnt

	assert(checksum == wd_data_s.protect_s.num); // ERROR; Protection checksum failed. Update guard_prot_s initialiser 

	wd_data_s.state = WD_UNARMED; // Initialise WatchDog to Un-armed state
	wd_data_s.shared_out = 0; // Clear (Bit_0 and Bit_1 of) data for shared output port 

	// Loop until all motors have sent initialisation command
	while(0 < guard_init_s.num)
	{
		select {
			// Service any change on input port pins of UN-initialised motors
			case (int motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++) 
						guard_init_s.guards[motor_cnt] => c_wd[motor_cnt] :> cmd :
			{
				assert (WD_CMD_INIT == cmd); // Un-expected command Received

				guard_init_s.guards[motor_cnt] = 0; // Switch off guard for initialised motor 
				guard_init_s.num--; // Decrement No. of UN-initialised motors
			} // case
			break;

			default :
				chronometer :> ts1;
				chronometer when timerafter(ts1 + MICRO_SEC) :> void;
			break; // default
		} // select
	} // while(0 < run_cnt)

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

	// Signal Initialisation complete
 	for (motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++)
	{
		c_wd[motor_cnt] <: WD_CMD_ACK;
	} // for motor_cnt
} // init_wd
/*****************************************************************************/
static void run_unarmed( // Runs WatchDog in UN-armed mode until Motors have warmed-up
	WD_DATA_TYP &wd_data_s, // Reference to structure containing data for WatchDog circuit
	chanend c_wd[NUMBER_OF_MOTORS], // Array of WatchDog channels
	out port p2_wd // 2-bit port used to control WatchDog circuit (2 chips)
)
{
	WD_GUARD_TYP unarmed_s = wd_data_s.protect_s; // Initialise data structure used to check all motors are armed
	unsigned cmd; // WatchDog command from Client
	timer chronometer; // Timer


	// Loop until all motors have been armed
	while(0 < unarmed_s.num)
	{
		// Wait for one of following events
		select
		{
			// Service any change on the input port pins of any protected motors
			case (int motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++) 
						wd_data_s.protect_s.guards[motor_cnt] => c_wd[motor_cnt] :> cmd :
			{
				switch(cmd)
				{
					case WD_CMD_DISABLE : // Actively disable the WatchDog circuit
						wd_data_s.state = WD_STOP; // Switch to state where WatchDog stops Motors
					break; // WD_CMD_DISABLE 

					case WD_CMD_TICK : // Check for end of Start-up mode
						wd_data_s.shared_out ^= TICK_MASK; // Toggle Bit_1 (for WatchDog Chip)

						// Check if this motor was unarmed
						if (unarmed_s.guards[motor_cnt])
						{
							unarmed_s.guards[motor_cnt] = 0; // Clear unarmed flag
							unarmed_s.num--; // Decrement No. of UN-initialised motors
						} // if (unarmed_s.guards[motor_cnt])
					break; // WD_CMD_TICK

					default :
						assert( 0 == 1 ); // Unexpected Command Received
					break; // default
				} // switch(cmd)
			}
			break; // case motor_cnt

			// Check if periodic tick required
			case chronometer when timerafter(wd_data_s.time + HALF_TICK_PERIOD) :> wd_data_s.time:
				// Force tick every 0.5 milli-seconds, to keep WatchDog alive until Client supplies ticks
				wd_data_s.shared_out ^= TICK_MASK; // Toggle Bit_1 (for WatchDog Chip)
			break; // case chronometer
		} // select

		p2_wd <: wd_data_s.shared_out; // Send out the new value to the shared port
	} // while

	wd_data_s.state = WD_ARMED; // Switch to fully armed WatchDog state
} // run_unarmed 
/*****************************************************************************/
static void run_wd_armed( // Run WatchDog fully armed until STOP request received
	WD_DATA_TYP &wd_data_s, // Reference to structure containing data for WatchDog circuit
	chanend c_wd[NUMBER_OF_MOTORS], // Array of WatchDog channels
	out port p2_wd // 2-bit port used to control WatchDog circuit (2 chips)
)
{
	unsigned cmd; // WatchDog command from Client
	int tick_cnts[NUMBER_OF_MOTORS];
	int motor_id;


	// Clear counter array
	for (motor_id=0; motor_id<NUMBER_OF_MOTORS; motor_id++) tick_cnts[motor_id] = 0;

	// Enter active WatchDog mode ...

	// Loop until disabled
	while (WD_STOP != wd_data_s.state)
	{
		// Wait for one of following events
		select
		{
			// Service any change on the input port pins of any protected motors
			case (int motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++) 
						wd_data_s.protect_s.guards[motor_cnt] => c_wd[motor_cnt] :> cmd :
			{
				switch(cmd)
				{
					case WD_CMD_DISABLE : // Actively disable the WatchDog circuit
						wd_data_s.state = WD_STOP; // Switch to state where WatchDog stops Motors
					break; // WD_CMD_DISABLE 
		
					case WD_CMD_TICK : // Toggle WD_TICK pin to keep WatchDog 'alive'
						wd_data_s.shared_out ^= TICK_MASK; // Toggle Bit_1 (for WatchDog Chip)
						tick_cnts[motor_cnt]++; // Increment No. of ticks for this motor

						// Check if tick-limit reached
						if (tick_cnts[motor_cnt] > MAX_WD_TICKS)
						{ // Check all motors have supplied ticks
							for (motor_id=0; motor_id<NUMBER_OF_MOTORS; motor_id++)
							{
								// Check if this motor is protected
                if (wd_data_s.protect_s.guards[motor_id])
								{ // This motor protected
									if (0 == tick_cnts[motor_id])
									{ // ERROR: This motor is NOT supplying enough ticks
										assert(0 == 1); // ERROR: One of motors is NOT suppliing enough ticks
										wd_data_s.state = WD_STOP; // Switch to state where WatchDog stops Motors
										break;
									} // if (0 == tick_cnts[motor_id])
	
									tick_cnts[motor_id] = 0; // Clear this motors tick count
                } // if (wd_data_s.protect_s.guards[motor_id])
							} // for motor_id
						} // if (tick_cnts[motor_cnt] > MAX_WD_TICKS)
					break; // WD_CMD_TICK 
		
					default :
						assert( 1 == 2 ); // Unexpected Command Received
					break; // default
				} // switch(cmd)
			}
			break; // case motor_cnt
		} // select

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
