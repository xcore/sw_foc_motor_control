/*
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
 *
 **/

#include "pwm_server.h"

/*****************************************************************************/
static void init_pwm_data( // Initialise structure containing PWM data
	PWM_SERV_TYP &pwm_serv_s, // Reference to structure containing PWM server control data
	PWM_COMMS_TYP &pwm_comms_s, // Reference to structure containing PWM communication data
	PWM_ARRAY_TYP &pwm_ctrl_s, // Reference to structure containing double-buffered PWM output data
	chanend c_pwm // PWM channel between Client and Server
)
{
	// Initialise the address of PWM Control structure, in case shared memory is used
	pwm_comms_s.mem_addr = get_pwm_struct_address( pwm_ctrl_s );

	// Send address to Client, in case shared memory is used
	c_pwm <: pwm_comms_s.mem_addr;

	// Wait for initial buffer id
	c_pwm :> pwm_comms_s.buf;
} // init_pwm_data
/*****************************************************************************/
static void do_pwm_port_config( // Configure ports for one motor
	buffered out port:32 p32_pwm_hi[],
	buffered out port:32 p32_pwm_lo[],
	in port? p16_adc_sync,
	clock pwm_clk
)
{
	unsigned i;


	// Loop through PWM phases
	for (i = 0; i < NUM_PWM_PHASES; i++)
	{	// Configure ports for this phase

		configure_out_port( p32_pwm_hi[i] ,pwm_clk ,0 ); // Set initial value of port to 0 (Switched Off)
		configure_out_port( p32_pwm_lo[i] ,pwm_clk ,0 ); // Set initial value of port to 0 (Switched Off)
		set_port_inv( p32_pwm_lo[i] );
	}

	// Check of ADC synchronisation is being used
	if (1 == LOCK_ADC_TO_PWM)
	{	// ADC synchronisation activated

		// Configure dummy input port used to send ADC synchronisation pulse
		configure_in_port( p16_adc_sync ,pwm_clk );
	} // if (1 == LOCK_ADC_TO_PWM)
} // do_pwm_port_config
/*****************************************************************************/
void foc_pwm_config(  // Configure ports
	buffered out port:32 p32_pwm_hi[NUMBER_OF_MOTORS][NUM_PWM_PHASES], // array of PWM ports (High side)
	buffered out port:32 p32_pwm_lo[NUMBER_OF_MOTORS][NUM_PWM_PHASES], // array of PWM ports (Low side)
	in port p16_adc_sync[NUMBER_OF_MOTORS], // Dummy port used with ADC trigger
	clock pwm_clk // clock for generating accurate PWM timing
)
{
	int motor_cnt; // motor counter


	// Configure clock rate to PLATFORM_REFERENCE_MHZ/1 (100 MHz)
 	configure_clock_rate( pwm_clk ,PLATFORM_REFERENCE_MHZ ,1 );

	// Loop through motors
	for (motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++)
	{ // Configure all ports for this motor
		do_pwm_port_config( p32_pwm_hi[motor_cnt] ,p32_pwm_lo[motor_cnt]  ,p16_adc_sync[motor_cnt] ,pwm_clk );
	} // for (motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++)

	start_clock( pwm_clk ); // Start common PWM clock, once all ports configured
} // foc_pwm_config
/*****************************************************************************/
void foc_pwm_do_triggered( // Implementation of the Centre-aligned, High-Low pair, PWM server, with ADC sync
	unsigned motor_id, // Motor identifier
	chanend c_pwm, // PWM channel between Client and Server
	buffered out port:32 p32_pwm_hi[NUM_PWM_PHASES], // array of PWM ports (High side)
	buffered out port:32 p32_pwm_lo[NUM_PWM_PHASES], // array of PWM ports (Low side)
	chanend c_adc_trig, // ADC trigger channel
	in port p16_adc_sync // Dummy port used with ADC trigger
)
{
	PWM_ARRAY_TYP pwm_ctrl_s; // Structure containing double-buffered PWM output data
	PWM_SERV_TYP pwm_serv_s; // Structure containing PWM server control data
	PWM_COMMS_TYP pwm_comms_s; // Structure containing PWM communication data
	int cmd; // PWM command
	int do_loop = 1; // Set 'while loop' flag
	unsigned pattern; // Bit-pattern on port
	int pwm_time_off; // Used to evaluate PWM time period offset for each motor


	acquire_lock();
	printstr("PWM Server_");
	printint(motor_id);
	printstrln(" Starts");
	release_lock();

	pwm_serv_s.id = motor_id; // Assign motor identifier

	// Find out value of time clock on an output port, WITHOUT changing port value
	pattern = peek( p32_pwm_hi[0] ); // Find out value on 1-bit port. NB Only LS-bit is relevant
	pwm_serv_s.ref_time = partout_timestamped( p32_pwm_hi[0] ,1 ,pattern ); // Re-load output port with same bit-value

	// This section ensures PWM workload is interleaved in time ...

	pwm_serv_s.ref_time += INIT_SYNC_INCREMENT; // NB Ensure we have a time in the future

	// Align base time reference with PWM_PERIOD boundary
	pwm_serv_s.ref_time = pwm_serv_s.ref_time & ~(INIT_SYNC_INCREMENT - 1);

	pwm_time_off = motor_id * PWM_STAGGER; // Calculate PWM time period stagger offset for this motor

	pwm_serv_s.ref_time += pwm_time_off; // Add offset to base reference

	init_pwm_data( pwm_serv_s ,pwm_comms_s ,pwm_ctrl_s ,c_pwm ); // Initialise PWM parameters (from Client)

	pwm_serv_s.data_ready = 1; // Signal new data ready. NB this happened in init_pwm_data()

	/* This loop requires at least ~280 cycles, which means the PWM period must be at least 512 cycles (as needs to be 2^n)
	 * If convert_all_pulse_widths was optimised for speed, maybe a PWM period of 256 cycles would be possible
	 */

	// Loop until termination command received
	while (do_loop)
	{
#pragma xta endpoint "pwm_main_loop"
		// Do processing for one PWM period, using PWM data in current buffer

		// Check if new data ready
		if (pwm_serv_s.data_ready)
		{ // Data ready

			// If shared memory was used for data transfer, port data is already in pwm_ctrl_s.buf_data[pwm_comms_s.buf]
			if (0 == PWM_SHARED_MEM)
			{ // Shared Memory NOT used, so receive pulse widths from channel and calculate port data on server side.
				// This branch requires 216..224 cycles

				c_pwm :> pwm_comms_s.params; // Receive PWM parameters from Client

				// Convert all PWM pulse widths to pattern/time_offset port data
				convert_all_pulse_widths( pwm_comms_s ,pwm_ctrl_s.buf_data[pwm_comms_s.buf] ); // Max 178 Cycles
			} // if (0 == PWM_SHARED_MEM)
		} // if (pwm_serv_s.data_ready)

		pwm_serv_s.ref_time += INIT_SYNC_INCREMENT; // Update reference time to next PWM period

		// Load ports in correct time order. Rising-edges --> ADC_trigger --> Falling-edges ...

		/* These port-load commands have been unwrapped and expanded to improve timing.
		 * WARNING: If timing is not met pulse stays low for whole timer period (2^16 cycles)
		 */
    // Rising edges - these have negative time offsets - 44 Cycles
		p32_pwm_hi[PWM_PHASE_A] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[PWM_PHASE_A].hi.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[PWM_PHASE_A].hi.pattern;
		p32_pwm_lo[PWM_PHASE_A] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[PWM_PHASE_A].lo.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[PWM_PHASE_A].lo.pattern;

		p32_pwm_hi[PWM_PHASE_B] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[PWM_PHASE_B].hi.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[PWM_PHASE_B].hi.pattern;
		p32_pwm_lo[PWM_PHASE_B] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[PWM_PHASE_B].lo.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[PWM_PHASE_B].lo.pattern;

		p32_pwm_hi[PWM_PHASE_C] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[PWM_PHASE_C].hi.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[PWM_PHASE_C].hi.pattern;
		p32_pwm_lo[PWM_PHASE_C] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[PWM_PHASE_C].lo.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].rise_edg.phase_data[PWM_PHASE_C].lo.pattern;

		// Check of ADC synchronisation is being used
		if (1 == LOCK_ADC_TO_PWM)
		{ // ADC synchronisation active

			/* This trigger is used to signal to the ADC block the location of the PWM High-pulse mid-point.
			 * As a blocking wait is required, we send the trigger early by 1/4 of a PWM pulse.
			 * This then allows time to set up the falling edges before they are required.
			 * WARNING: The ADC module (module_foc_adc) must compensate for the early trigger.
			 */
			p16_adc_sync @ (PORT_TIME_TYP)(pwm_serv_s.ref_time - QUART_PWM_MAX) :> void; // NB Blocking wait for 1750..1920 cycles
			outct( c_adc_trig ,XS1_CT_END ); // Send synchronisation token to ADC
		} // if (1 ==LOCK_ADC_TO_PWM)

		/* These port-load commands have been unwrapped and expanded to improve timing.
		 * DANGER: If a short pulse (Low voltage) does NOT meet timing, then the pulse stays high for a whole timer period
		 * (2^16 cycles) This is a HIGH voltage and could damage the motor.
		 */
    // Falling edges - these have positive time offsets - 44 Cycles
		p32_pwm_hi[PWM_PHASE_A] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[PWM_PHASE_A].hi.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[PWM_PHASE_A].hi.pattern;
		p32_pwm_lo[PWM_PHASE_A] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[PWM_PHASE_A].lo.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[PWM_PHASE_A].lo.pattern;

		p32_pwm_hi[PWM_PHASE_B] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[PWM_PHASE_B].hi.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[PWM_PHASE_B].hi.pattern;
		p32_pwm_lo[PWM_PHASE_B] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[PWM_PHASE_B].lo.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[PWM_PHASE_B].lo.pattern;

		p32_pwm_hi[PWM_PHASE_C] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[PWM_PHASE_C].hi.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[PWM_PHASE_C].hi.pattern;
		p32_pwm_lo[PWM_PHASE_C] @ (PORT_TIME_TYP)(pwm_serv_s.ref_time + pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[PWM_PHASE_C].lo.time_off) <: pwm_ctrl_s.buf_data[pwm_comms_s.buf].fall_edg.phase_data[PWM_PHASE_C].lo.pattern;

		// Check if new data is ready  - ~8 cycles
		select
		{
			case c_pwm :> cmd : // Get next PWM command

				// Check for termination command
				if (PWM_CMD_LOOP_STOP == cmd)
				{ // Termination command
					do_loop = 0; // Clear flag to terminate loop

					pwm_serv_s.data_ready = 0; // signal data NOT ready
				} // if (PWM_CMD_LOOP_STOP == cmd)
				else
				{ // cmd must be buffer index
					pwm_comms_s.buf = cmd; // Assign new buffer index (0 or 1)

				pwm_serv_s.data_ready = 1; // signal new data ready
				} // else !(PWM_CMD_LOOP_STOP == cmd)
			break; // c_pwm :> pwm_comms_s.buf;

			default :
				pwm_serv_s.data_ready = 0; // signal data NOT ready
			break; // default
		} // select
	} // while(do_loop)

	acquire_lock();
	printstr("PWM Server_");
	printint(motor_id);
	printstrln(" Ends");
	release_lock();

	c_pwm <: PWM_CMD_ACK; // Acknowledge command to terminate PWM Server
} // foc_pwm_do_triggered
/*****************************************************************************/
// pwm_service_inv
