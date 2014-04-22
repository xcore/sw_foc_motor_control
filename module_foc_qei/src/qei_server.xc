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

#include "qei_server.h"

/*****************************************************************************/
static void do_qei_port_config( // Configure one QEI input port
	buffered QEI_PORT in pb4_QEI, // buffered 4-bit input ports (carries raw QEI motor data)
	clock qei_clk 
)
{
	// Configure port to be a clocked input port with no ready signals.
	configure_in_port( pb4_QEI ,qei_clk ); // XS1 Library call
} // do_qei_port_config
/*****************************************************************************/
static void init_phase_data( // Initialise structure of QEI phase data
	QEI_PHASE_TYP &phase_s, // Reference to structure containing data for one QEI phase
	QEI_PHASE_ETYP phase_index, // Unique QEI Phase Identifier
	int motor_index // Unique Motor Identifier
)
{
	phase_s.up_filt = 0; // Filterd phase value
	phase_s.scale_err = 0; // Error diffusion value for scaling
	phase_s.filt_err = 0; // Error diffusion value for filtering
	phase_s.prev = 0; // Previous phase value
	phase_s.phase_id = phase_index; // Unique QEI phase identifier
	phase_s.motor_id = motor_index; // Unique motor identifier
} // init_phase_data
/*****************************************************************************/
static void init_qei_data( // Initialise  QEI data for one motor
	QEI_DATA_TYP &inp_qei_s, // Reference to structure containing QEI parameters for one motor
	int inp_id  // Input unique motor identifier
)
{
  /* Look-up table for converting phase changes to angle increments, 
	 * inner(fastest) changing index is NEW phase combination
	 * WARNING: Using a 1-D table is slower, because index is calculated in XC.
	 */
	QEI_LUT_TYP ang_incs = {{{ 0 , 1 , -1 ,  0},
													 {-1 , 0 ,  0 ,  1},
													 { 1 , 0 ,  0 , -1},
													 { 0 , -1 , 1 ,  0}}};


	assert( QEI_PER_REV == (1 << QEI_RES_BITS) ); // Check consistency of pre-defined QEI values

	inp_qei_s.ang_lut = ang_incs; // Assign table converting phase changes to angle increments

	inp_qei_s.params.tot_ang_this = 0;  // Clear total angle for this motor
	inp_qei_s.params.tot_ang_master = 0;  // Clear total angle for master motor
	inp_qei_s.params.phase_period = 0; // Clear Time taken to traverse previous QEI phase
	inp_qei_s.params.corr_ang = 0; // Clear correction angle
	inp_qei_s.params.orig_corr = 0; // Preset flag to QEI correction NOT available
	inp_qei_s.params.err = QEI_ERR_OFF; // Clear error status flag returned to client

	inp_qei_s.pins_idle = 1; // NB Flag set until first pin-change is detected
	inp_qei_s.id = inp_id; // Assign unique motor identifier
	inp_qei_s.rev_period = QEI_PER_REV; // Preset No. of QEI phases changes between each origin detection
	inp_qei_s.ang_tot = 0; // Reset counter indicating total angular position of motor (since time=0)
	inp_qei_s.prev_ang = 0; // Reset previous total angle value
	inp_qei_s.ang_inc = 0; // Reset angular position increment
	inp_qei_s.status_errs = 0; // Initialise counter for QEI status errors

	inp_qei_s.prev_orig = 0; // Clear value of previous origin flag
	inp_qei_s.prev_phases = 0; // Clear value of previous phase combination

	// Initialise data for both QEI Phases 
	init_phase_data( inp_qei_s.phase_data[QEI_PHASE_A] ,QEI_PHASE_A ,inp_id );
	init_phase_data( inp_qei_s.phase_data[QEI_PHASE_B] ,QEI_PHASE_B ,inp_id );

	return;
} // init_qei_data
/*****************************************************************************/
static void estimate_error_status( // Update estimate of error status based on new data
	QEI_DATA_TYP &inp_qei_s, // Reference to structure containing QEI parameters for one motor
	ERROR_QEI_ENUM new_err // Newly acquired error flag
)
// NB We require MAX_QEI_STATUS_ERR consecutive new errors before error estimate is set
{
	// Check if status changed
	if (QEI_ERR_ON == new_err)
	{ // new error detected

		// Check previous error estimate
		if (QEI_ERR_OFF == inp_qei_s.params.err)
		{ // NO previous detected Error
			inp_qei_s.status_errs++; // Increment new error count

			// Check if too many errors occured
			if (MAX_QEI_STATUS_ERR <=  inp_qei_s.status_errs)
			{
				inp_qei_s.params.err = QEI_ERR_ON; // Switch ON Error Estimate
			} // if (MAX_QEI_STATUS_ERR <=  inp_qei_s.status_errs)
		} // if (QEI_ERR_OFF == inp_qei_s.params.err)
	} // if (QEI_ERR_ON == new_err)
	else
	{ // NO new error detected

		// Check previous error estimate
		if (QEI_ERR_ON == inp_qei_s.params.err)
		{ // Already detected Error
			inp_qei_s.status_errs--; // Decrement new error count

			// Check if all errors cleared
			if (0 >=  inp_qei_s.status_errs)
			{
				inp_qei_s.params.err = QEI_ERR_OFF; // Switch OFF Error Estimate
			} // if (0 >=  inp_qei_s.status_errs)
		} // if (QEI_ERR_ON == inp_qei_s.params.err)
	} // else !(QEI_ERR_ON == new_err)

} // estimate_error_status
/*****************************************************************************/
#pragma unsafe arrays
static void acknowledge_qei_command( // Acknowledge QEI command
	QEI_DATA_TYP &inp_qei_s, // Reference to structure containing QEI parameters for one motor
	streaming chanend c_qei // Data channel to client (carries processed QEI data)
)
{
	c_qei <: QEI_CMD_ACK; // Transmit QEI parameters to Client
} // acknowledge_qei_command
/*****************************************************************************/
static void process_new_origin( // Process new origin state
	QEI_DATA_TYP &inp_qei_s // Reference to structure containing QEI parameters for one motor
)
{
	int diff_ang = inp_qei_s.params.tot_ang_this - inp_qei_s.prev_ang; // Calculate change in angle since last origin
	int abs_diff = abs(diff_ang); // Magnitude of angle change since last origin
	int drift_ang = abs(abs_diff - inp_qei_s.rev_period);	// QEI drift from estimated origin position
	int uncorr_ang;	// Angular position before correction


	// Check for sensible drift value
	if (PERIOD_DELTA_LIM > drift_ang)
	{ // Valid drift value
		uncorr_ang = inp_qei_s.ang_tot; // Store angle before correction

		// Re-calibrate total-angle by rounding to multiple of QEI_PER_REV;
		inp_qei_s.ang_tot += HALF_QEI_CNT; // Add offset to get rounding
		inp_qei_s.ang_tot &= ~QEI_REV_MASK; // Clear least significant bits

		// Check that previous correction has been processed by client
		assert(0 == inp_qei_s.params.orig_corr); // ERROR: Unused correction

		// Update client data parameters
		inp_qei_s.params.corr_ang = uncorr_ang - inp_qei_s.ang_tot; // Evaluate correction
		inp_qei_s.params.orig_corr = 1; // Set flag to 'correction available'

		// Update estimate of QEI period
		if (inp_qei_s.rev_period < abs_diff)
		{ // Allow small increase in estimated QEI period
			inp_qei_s.rev_period++;

			assert( inp_qei_s.rev_period < (QEI_PER_REV << 1)); // MB~ Check for overflow
		} // if (inp_qei_s.rev_period < abs_diff)
		else
		{
			if (inp_qei_s.rev_period > abs_diff)
			{ // Allow small decrease in estimated QEI period
				inp_qei_s.rev_period--;

				assert( inp_qei_s.rev_period > (QEI_PER_REV >> 1)); // MB~ Check for underflow
			} // if (inp_qei_s.rev_period > abs_diff)
		} // else !(inp_qei_s.rev_period < abs_diff)
	} // if (PERIOD_DELTA_LIM > drift_ang)

	inp_qei_s.prev_ang = inp_qei_s.params.tot_ang_this; // Store origin angle for next iteration

	return;
} // process_new_origin
/*****************************************************************************/
#pragma unsafe arrays
static void service_client_data_request( // Send processed QEI data to client
	QEI_DATA_TYP &inp_qei_s, // Reference to structure containing QEI parameters for one motor
	streaming chanend c_qei, // Data channel to client (carries processed QEI data)
	int master_tot_ang // Total angle for master motor
)
{
	inp_qei_s.params.phase_period = (inp_qei_s.change_time - inp_qei_s.prev_change); // Time taken to traverse previous QEI phase
	inp_qei_s.params.tot_ang_this = inp_qei_s.ang_tot; // Transfer total-angle to client data structure
	inp_qei_s.params.tot_ang_master = master_tot_ang;

#if (LDO_MOTOR_SPIN)
	// NB The QEI sensor on the LDO motor is inverted with respect to the other sensors
	inp_qei_s.params.tot_ang_this = -inp_qei_s.params.tot_ang_this;
	inp_qei_s.params.corr_ang = -inp_qei_s.params.corr_ang;
#endif // (LDO_MOTOR_SPIN)

	c_qei <: inp_qei_s.params; // Transmit QEI parameters to Client

	inp_qei_s.params.corr_ang = 0; // Clear correction value
	inp_qei_s.params.orig_corr = 0; // Reset flag to correction NOT available
} // service_client_data_request
/*****************************************************************************/
static unsigned update_one_phase( // Returns filtered phase value NB Only works on Binary data
	QEI_PHASE_TYP &phase_s, // Reference to structure containing data for one QEI phase
	unsigned inp_phase // Input raw phase value (Zero or One)
)
{
	unsigned scaled_inp = (inp_phase << QEI_SCALE_BITS); // NB Scale by 65536 
	int inp_diff; // Difference between previous filtered value and new input value
	int filt_corr; // Correction to filtered value
	int out_phase; // Output (Down-sampled) filtered phase value (Zero or One)
	

	inp_diff = ((int)scaled_inp - (int)phase_s.up_filt); // Evaluate input change
	inp_diff += phase_s.filt_err; // Add diffusion error

 	filt_corr = (inp_diff + QEI_VELOC_HALF) >> QEI_VELOC_BITS; // Multiply by filter coeficient (1/(2^n))
	phase_s.filt_err = inp_diff - (filt_corr << QEI_VELOC_BITS); // Update filtering diffusion error

	phase_s.up_filt += filt_corr; // Update scaled filtered output

	if (QEI_SCALE_HALF > phase_s.up_filt)
	{
		out_phase = 0;
	} // if (QEI_SCALE_HALF > phase_s.up_filt)
	else
	{
		out_phase = 1;
	} // if (QEI_SCALE_HALF > phase_s.up_filt)

	return (unsigned)out_phase;
} // update_one_phase
/*****************************************************************************/
static void update_first_phase( // Special case filter for 1st phase value
	QEI_PHASE_TYP &phase_s, // Reference to structure containing data for one QEI phase
	int inp_phase // Input raw phase value (Zero or One)
)
{
	phase_s.up_filt = (inp_phase << QEI_SCALE_BITS);
	phase_s.prev =  phase_s.up_filt;
} // update_first_phase
/*****************************************************************************/
static void update_rs_phase_states( // Regular-Sampling: Update phase state
	QEI_DATA_TYP &inp_qei_s, // Reference to structure containing QEI parameters for one motor
	unsigned samp_time, // sample time-stamp (32-bit value)
	unsigned cur_phases // Current set of phase values
)
{
	int filt_a;  // Filtered Phase_A value (Zero or One)
	int filt_b;  // Filtered Phase_B value (Zero or One)
	unsigned filt_phases; // New set of phase values


	// Filter each phase seperately
	filt_a = update_one_phase( inp_qei_s.phase_data[QEI_PHASE_A] ,(cur_phases & 0b01) ); // Phase_A
	filt_b = update_one_phase( inp_qei_s.phase_data[QEI_PHASE_B] ,((cur_phases & 0b10) >> 1) ); // Phase_B

	filt_phases = (filt_b << 1) | filt_a; // Recombine phases

	// Check for change in filtered phase data
	if (inp_qei_s.prev_phases != filt_phases)
	{	// Phase data change detected

		inp_qei_s.ang_inc = inp_qei_s.ang_lut.incs[inp_qei_s.prev_phases][filt_phases]; // Decode new angular increment

		// Check for valid transition
		if (0 != inp_qei_s.ang_inc)
		{
			inp_qei_s.ang_tot += inp_qei_s.ang_inc; // Update new angle with angular increment

			inp_qei_s.prev_change = inp_qei_s.change_time; // Store previous phase change time
			inp_qei_s.change_time = samp_time; // Store time of filtered phase change

			inp_qei_s.prev_phases = filt_phases; // Store filtered phases for next iteration
		} //if (0 != inp_qei_s.ang_inc)
	} // if (inp_qei_s.prev_phases != filt_phases)

	return;
} // update_rs_phase_states
/*****************************************************************************/
static void service_input_pins( // Service detected change on input pins
	QEI_DATA_TYP &inp_qei_s, // Reference to structure containing QEI parameters for one motor
	unsigned time_stamp, // 32-bit time-stamp
	QEI_RAW_TYP inp_pins // Set of raw data values on input port pins
)
{
	unsigned cur_phases; // Current set of phase values
	unsigned orig_flg; // Flag set when motor at origin position 
	unsigned err_flg; // Flag set when Error condition detected


	// NB Due to noise corrupting bit-values, flags may change, even though phase does NOT appear to have changed
	cur_phases = inp_pins & QEI_PHASE_MASK; // Extract Phase bits from input pins
	orig_flg = inp_pins & QEI_ORIG_MASK; 		// Extract origin flag 
	err_flg = !(inp_pins & QEI_NERR_MASK); 	// Extract error flag. NB Bit_3=0, and err_flg=1, if error detected, 

	// Check for first data
	if (inp_qei_s.pins_idle)
	{ // Initialise 'previous data'
		inp_qei_s.pins_idle= 0; // Switch OFF idle flag

		inp_qei_s.prev_phases = cur_phases; // Store phase value
		inp_qei_s.prev_orig = orig_flg; // Store origin flag value
		inp_qei_s.params.err = err_flg; // Preset client error flag.

		update_first_phase( inp_qei_s.phase_data[QEI_PHASE_A] ,(cur_phases & 0b01) ); // Phase_A
		update_first_phase( inp_qei_s.phase_data[QEI_PHASE_B] ,((cur_phases & 0b10) >> 1) ); // Phase_B
	} // if (0 == inp_qei_s.pin_changes)
	else
	{ // NOT first data
		update_rs_phase_states( inp_qei_s ,time_stamp ,cur_phases ); // update phase state
	
		// Check for change in origin state
		if (orig_flg != inp_qei_s.prev_orig)
		{ // Process change of origin flag

			// Check if new origin flag found
			if (orig_flg)
			{ // New origin found
				process_new_origin( inp_qei_s ); // process new origin
			} // if (orig_flg)
	
			inp_qei_s.prev_orig = orig_flg; // Store origin flag value
		} // if (orig_flg != inp_qei_s.prev_orig)
	
		// Check for change in error state
		if (err_flg != inp_qei_s.params.err)
		{ // Change in error state detected

			// Update estimate of error status using new error flag
			estimate_error_status( inp_qei_s ,err_flg ); // NB Updates inp_qei_s.params.err
		} // if (err_flg != inp_qei_s.params.err)
	} // else !(0 == inp_qei_s.pin_changes)

	return;
} // service_input_pins
/*****************************************************************************/
void foc_qei_config(  // Configure all QEI ports
	buffered QEI_PORT in pb4_QEI[NUMBER_OF_MOTORS], // Array of buffered 4-bit input ports (carries raw QEI motor data)
	clock qei_clks[NUMBER_OF_MOTORS] // Array of clocks for generating accurate QEI timing (one per input port)
)
{
	timer chronometer; // H/W timer
	unsigned big_ticks; // ticks from 32-bit timer
	unsigned tst_ticks; // test 32-bit timer value
	int motor_cnt; // motor counter


	assert( HALF_PERIOD < 256 ); // Check for illegal clock configuration value

	// Once all ports configured, synchronise port-timers with reference clock
	chronometer :> big_ticks;	// Get current time
	big_ticks &= ~0xffff;			// Clear bottom 16 bits
	big_ticks += 0x20000;			// Step on time by 2 port-timer cycles, to allow some time configure ports

	// Loop through motors
	for (motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++)
	{	// Configure all ports for this motor

		// NB We want to sample the QEI data from any one of the motors every HALF_PERIOD ticks.
		configure_clock_ref( qei_clks[motor_cnt] ,HALF_PERIOD ); // Configure clock rate to PLATFORM_REFERENCE_MHZ/(2*HALF_PERIOD) (100/128 MHz)

		do_qei_port_config( pb4_QEI[motor_cnt] ,qei_clks[motor_cnt] ); // configure current port

		// Check timing has been met
		chronometer :> tst_ticks;
		assert( big_ticks > tst_ticks); // ERROR: Timing NOT met
 
		// Stagger when each port starts sampling, so buffer servicing is interleaved
		chronometer when timerafter(big_ticks) :> void;	// Wait until next synchronisation time
		start_clock( qei_clks[motor_cnt] ); // Start current QEI clock, 

		big_ticks += STAG_TICKS; // Update time to when next port should start
	} // for (motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++) 

} // foc_qei_config
/*****************************************************************************/
#pragma unsafe arrays
void foc_qei_do_multiple( // Get QEI data from motor and send to client
	streaming chanend c_qei[], // Array of data channel to client (carries processed QEI data)
	buffered port:32 in pb4_inp[NUMBER_OF_MOTORS] // Array of 32-bit buffered 4-bit input ports on which to receive test data
)
{
	QEI_DATA_TYP all_qei_s[NUMBER_OF_MOTORS]; // Array of structures containing QEI parameters for all motor
	timer chronometer; // H/W timer

	unsigned samp32_time; // sample time-stamp (32-bit value) (with fixed delay)
	PORT_TIME_TYP port16_cnt; // port sample counter (16-bit value)
	PORT_TIME_TYP prev16_cnt[NUMBER_OF_MOTORS]; // previous port sample cnt (16-bit value)
	PORT_TIME_TYP num_samps; // number of captured port samples
	QEI_RAW_TYP inp_pins; // raw data value from input port pins
	CMD_QEI_ENUM inp_cmd; // QEI command from Client
	unsigned buf_data = 0; // Clear 32-bit data buffer
	int time_err = 0; // Timing error count

	int motor_cnt; // Motor counter
	int samp_cnt; // Sample counter
	int do_loop = 1;   // Flag set until loop-end condition found


	acquire_lock(); 
	printstrln("                                          QEI RS_Server Starts");
	release_lock();

	// Check if we are running on the simulator
	if(0 == _is_simulation())
	{ // Running on real hardware
		// Wait 128ms for UV_FAULT pin to finish toggling
		chronometer :> samp32_time; 
		chronometer when timerafter(samp32_time + (MILLI_SEC << 7)) :> void;
	} // if (0 == _is_simulation())

	// Loop through all motors
	for (int motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; ++motor_cnt) 
	{ // Initialise current motor

		init_qei_data( all_qei_s[motor_cnt] ,motor_cnt ); // Initialise QEI data for current motor

		chronometer :> samp32_time;	// Get current time
		all_qei_s[motor_cnt].change_time = samp32_time; // Initialise time-stamp with sensible value
		all_qei_s[motor_cnt].prev_change = samp32_time; // Initialise time-stamp with sensible value
		all_qei_s[motor_cnt].prev_time = samp32_time; // Initialise time-stamp with sensible value

		// Use acknowledge command to signal to control-loop that initialisation is complete
		acknowledge_qei_command( all_qei_s[motor_cnt] ,c_qei[motor_cnt] );

		{buf_data ,samp32_time} = partin_timestamped( pb4_inp[motor_cnt] ,4 ); // Read 1st (dummy) value to get timestamp

		prev16_cnt[motor_cnt] = (PORT_TIME_TYP)samp32_time; // Initialise previous port sample counter
	} // for motor_cnt

	// Loop until terminate condition received
	while (do_loop) 
	{
#pragma xta endpoint "qei_main_loop"

		// Loop through all motors
		for (motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++)
		{
			// Read 8 4-bit data samples from QEI port buffer
			// NB There may be lost samples, but there are never repeated samples, i.e always a full buffer
			pb4_inp[motor_cnt] :> buf_data @ port16_cnt; // Get all buffer samples (8)

			chronometer :> samp32_time; // Get 32-bit timer value to use as sample time-stamp

			// Check Timing has been met ...

			// Calculate number of samples captured since previous iteration 
			num_samps = (PORT_TIME_TYP)(port16_cnt - prev16_cnt[motor_cnt]);

			// Check for lost samples
			if (num_samps > SAMPS_PER_LOOP)
			{ // Samples lost
				time_err++; // Increment error count
				assert(time_err <= MAX_TIME_ERR); // ERROR: Too many Input QEI port samples dropped (Increase HALF_PERIOD)
			} // if (num_samps > SAMPS_PER_LOOP)
			else
			{ // NO samples lost
				if (time_err > 0) time_err--; // Decrement error count
			} // if (num_samps > SAMPS_PER_LOOP)

			// Loop through all samples in buffer 
			for (samp_cnt=0; samp_cnt<SAMPS_PER_LOOP; samp_cnt++)
			{
				inp_pins = buf_data & QEI_SAMP_MASK; // mask out LS 4 bits

				// NB As only the difference between time-stamps is used, the fixed delay cancels out
				service_input_pins( all_qei_s[motor_cnt] ,samp32_time ,inp_pins );

				buf_data >>= QEI_SAMP_BITS; // Shift next sample to LS end of buffer
				samp32_time += (unsigned)TICKS_PER_SAMP; // Increment time-stamp for next sample
			} // for samp_cnt

			// Wait for ANY channel command
			select {
				case c_qei[motor_cnt] :> inp_cmd : // QEI Client command
				{
					switch(inp_cmd)
					{
						case QEI_CMD_DATA_REQ : // Data Request
							// Send processed QEI data to client
							service_client_data_request( all_qei_s[motor_cnt] ,c_qei[motor_cnt] ,all_qei_s[0].ang_tot );
						break; // case QEI_CMD_DATA_REQ
	
						case QEI_CMD_LOOP_STOP : // Termination Command
							acknowledge_qei_command( all_qei_s[motor_cnt] ,c_qei[motor_cnt] ); // Acknowledge QEI command
	
							do_loop = 0; // Terminate while loop
						break; // case QEI_CMD_DATA_REQ
	
						default : // Unknown Command
							assert(0 == 1); // ERROR: Should not happen
						break; // default
					} // switch(inp_cmd)
				} // case	c_qei[motor_cnt] :> inp_cmd :
				break;
	
				default :
					// Do nothing
				break; // default
			} // select

			prev16_cnt[motor_cnt] = port16_cnt; // Store sample count for next iteration
		} // for motor_cnt
	}	// while (do_loop)

	acquire_lock(); 
	printstrln("");
	printstrln("                                             QEI Server Ends");
	release_lock();

	return;
} // foc_qei_do_multiple
/*****************************************************************************/
// qei_server.xc
