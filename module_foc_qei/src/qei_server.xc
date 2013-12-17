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
	configure_in_port( pb4_QEI ,qei_clk );
} // do_qei_port_config
/*****************************************************************************/
static void init_phase_data( // Initialise structure of QEI phase data
	QEI_PHASE_TYP &phase_s, // Reference to structure containing data for one QEI phase
	QEI_PHASE_ETYP phase_index, // Unigue QEI Phase Identifier
	int motor_index // Unigue Motor Identifier
)
{
	phase_s.up_filt = 0; // Filterd phase value
	phase_s.scale_err = 0; // Error diffusion value for scaling
	phase_s.filt_err = 0; // Error diffusion value for filtering
	phase_s.prev = 0; // Previous phase value
	phase_s.phase_id = phase_index; // Unigue QEI phase identifier
	phase_s.motor_id = motor_index; // Unigue motor identifier
} // init_phase_data
/*****************************************************************************/
static void init_qei_data( // Initialise  QEI data for one motor
	QEI_DATA_TYP &inp_qei_s, // Reference to structure containing QEI parameters for one motor
	int inp_id  // Input unique motor identifier
)
{
  /* Look-up table for converting phase changes to angle increments, inner(fastest) changing index is NEW phase combination
	 * WARNING: Using a 1-D table is slower, because index is calculated in XC.
	 */
	QEI_LUT_TYP ang_incs = {{{ 0 , 1 , -1 ,  0},
													 {-1 , 0 ,  0 ,  1},
													 { 1 , 0 ,  0 , -1},
													 { 0 , -1 , 1 ,  0}}};

	int tmp_val; // temporary manipulation value

#if (QEI_DBG)
	inp_qei_s.dd.cnt = 0; // MB~ Dbg
#endif //(QEI_DBG)

	inp_qei_s.ang_lut = ang_incs; // Assign table converting phase changes to angle increments

	inp_qei_s.params.tot_ang = 0;
	inp_qei_s.params.period = 0;
	inp_qei_s.params.old_ang = 0; // Clear old angular position (at reset)
	inp_qei_s.params.calib = 0; // Clear calibration flag
	inp_qei_s.params.err = QEI_ERR_OFF; // Clear error status flag returned to client

	inp_qei_s.pin_changes = 0; // NB Initially this is used to count input-pin changes
	inp_qei_s.id = inp_id; // Assign unique motor identifier
	inp_qei_s.orig_cnt = 0; // Reset origin counter
	inp_qei_s.tot_ang = 0; // Reset counter indicating total angular position of motor (since time=0)
	inp_qei_s.ang_inc = 0; // Reset angular position increment
	inp_qei_s.status_errs = 0; // Initialise counter for QEI status errors

	inp_qei_s.prev_orig = 0;
	inp_qei_s.prev_phases = 0;


	inp_qei_s.dbg_str[2] = 0; // String Terminator
	inp_qei_s.dbg = 0;

	inp_qei_s.tmp_raw = 0;
	inp_qei_s.tmp_i[0] = 0; //MB~
	inp_qei_s.tmp_i[1] = 0; //MB~
	inp_qei_s.tmp_i[2] = 0; //MB~
	inp_qei_s.tmp_i[3] = 0; //MB~

	// Check consistency of pre-defined QEI values
	tmp_val = (1 << QEI_RES_BITS); // Build No. of QEI points from resolution bits
	assert( QEI_PER_REV == tmp_val );

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
// We require MAX_QEI_STATUS_ERR consecutive new errors before error estimate is set
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
#if (QEI_DBG)
/*****************************************************************************/
static void print_smp_dbg( // MB~ Print debug sample
	QEI_DATA_TYP &inp_qei_s, // Reference to structure containing QEI parameters for one motor
	DBG_SMP_TYP dbg_smp_s // Reference to structure containing debug sample info
)
{
	printstr( dbg_smp_s.dbg_str ); 

	printstr(" TD="); printint( dbg_smp_s.diff_time ); 
	printstr(" LI="); printint( dbg_smp_s.lo_inc ); 
	printstr(" HI="); printint( dbg_smp_s.hi_inc );
	printstr(" PI="); printint( dbg_smp_s.phase_inc ); 
	printstr(" AI="); printint( dbg_smp_s.ang_inc ); 
	printstr(" CS="); printint( dbg_smp_s.curr_state ); 
	printstr(" CFD="); printint( dbg_smp_s.confid ); 
	printstr(" VEL="); printint( dbg_smp_s.veloc ); 
	printstrln("");
} // print_smp_dbg
/*****************************************************************************/
static void print_all_dbg( // MB~ Print all debug info.
	QEI_DATA_TYP &inp_qei_s // Reference to structure containing QEI parameters for one motor
)
{
	int dbg_cnt;


	acquire_lock(); 

	for(dbg_cnt=0; dbg_cnt<inp_qei_s.dd.cnt; dbg_cnt++)
	{ 
		printint(dbg_cnt); printstr(": "); 
		print_smp_dbg( inp_qei_s ,inp_qei_s.dd.ss[dbg_cnt] );  
	} // for dbg_cnt

	release_lock();
} // print_all_dbg
#endif //(QEI_DBG)
/*****************************************************************************/
static void update_origin_state( // Update origin state
	QEI_DATA_TYP &inp_qei_s, // Reference to structure containing QEI parameters for one motor
	unsigned orig_flg // Flag set when motor at origin position 
)
{
	if (orig_flg)
	{ // Reset position ( 'orig_flg' transition  0 --> 1 )
		inp_qei_s.params.old_ang = inp_qei_s.tot_ang; // Store uncorrected total-angle
		inp_qei_s.params.calib = 1; // Set flag indicating angular position is calibrated

		// Update origin counter
		if (0 > inp_qei_s.ang_inc)
		{ // Negative Angle
			inp_qei_s.orig_cnt--; // Decrement
		} // if (0 > inp_qei_s.ang_inc)
		else
		{ // Positive Angle
			inp_qei_s.orig_cnt++; // Increment
		} // else !(0 > inp_qei_s.ang_inc)

		// Round total angle to multiple of QEI_PER_REV;
		inp_qei_s.tot_ang += HALF_QEI_CNT; // Add offset to get rounding
		inp_qei_s.tot_ang &= ~QEI_REV_MASK; // Clear least significant bits
	} // if (orig_flg)

	return;
} // update_origin_state
/*****************************************************************************/
#pragma unsafe arrays
static void service_client_data_request( // Send processed QEI data to client
	QEI_DATA_TYP &inp_qei_s, // Reference to structure containing QEI parameters for one motor
	streaming chanend c_qei // Data channel to client (carries processed QEI data)
)
{
	inp_qei_s.params.tot_ang = inp_qei_s.tot_ang;
	inp_qei_s.params.period = (inp_qei_s.change_time - inp_qei_s.prev_change); // Time taken to traverse previous QEI phase

// xscope_int( (8+inp_qei_s.id) ,inp_qei_s.tmp_raw ); // MB~

	c_qei <: inp_qei_s.params; // Transmit QEI parameters to Client

// xscope_int( (10+inp_qei_s.id) ,(inp_qei_s.tmp_raw - inp_qei_s.tmp_inc) ); // MB~
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

 	filt_corr = (inp_diff + QEI_HALF_VELOC) >> QEI_VELOC_BITS; // Multiply by filter coeficient (1/(2^n))
	phase_s.filt_err = inp_diff - (filt_corr << QEI_VELOC_BITS); // Update filtering diffusion error

	phase_s.up_filt = (int)phase_s.up_filt + filt_corr; // Update scaled filtered output

	if (QEI_HALF_SCALE > phase_s.up_filt)
	{
		out_phase = 0;
	} // if (QEI_HALF_SCALE > phase_s.up_filt)
	else
	{
		out_phase = 1;
	} // if (QEI_HALF_SCALE > phase_s.up_filt)

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



	filt_a = update_one_phase( inp_qei_s.phase_data[QEI_PHASE_A] ,(cur_phases & 0b01) ); // Phase_A
	filt_b = update_one_phase( inp_qei_s.phase_data[QEI_PHASE_B] ,((cur_phases & 0b10) >> 1) ); // Phase_B

	filt_phases = (filt_b << 1) | filt_a; // Recombine phases

	// Check for change in filtered phase data
	if (inp_qei_s.prev_phases != filt_phases)
	{ // Update angular position
		inp_qei_s.ang_inc = inp_qei_s.ang_lut.incs[inp_qei_s.prev_phases][filt_phases]; // Decode angular increment

		// Check for valid transition
		if (0 != inp_qei_s.ang_inc)
		{
			inp_qei_s.tot_ang += inp_qei_s.ang_inc; // Update new angle with angular increment

			inp_qei_s.prev_change = inp_qei_s.change_time; // Store previous phase change time
			inp_qei_s.change_time = samp_time; // Store time of filtered phase change

			inp_qei_s.prev_phases = filt_phases;
		} //if (0 != inp_qei_s.ang_inc)
	} // if (inp_qei_s.prev_phases != filt_phases)

	return;
} // update_rs_phase_states
/*****************************************************************************/
static void update_et_phase_states( // Regular-Sampling: Update phase state
	QEI_DATA_TYP &inp_qei_s, // Reference to structure containing QEI parameters for one motor
	unsigned edge_time, // Phase-Edge time-stamp (32-bit value)
	unsigned cur_phases // Current set of phase values
)
{
	int diff_time = (int)((unsigned)(edge_time - inp_qei_s.prev_time)); // Difference between 2 adjacent time-stamps.

	// Check for sensible time
	if (THR_TICKS_PER_QEI < diff_time)
	{ // Sensible time!
		// Check for change in phase data
		if (inp_qei_s.prev_phases != cur_phases)
		{ // Update angular position
			inp_qei_s.ang_inc = inp_qei_s.ang_lut.incs[inp_qei_s.prev_phases][cur_phases]; // Decode angular increment
	
			// Check for valid transition
			if (0 != inp_qei_s.ang_inc)
			{
				inp_qei_s.tot_ang += inp_qei_s.ang_inc; // Update new angle with angular increment
	
				inp_qei_s.prev_change = inp_qei_s.change_time; // Store previous phase change time
				inp_qei_s.change_time = edge_time; // Store time of filtered phase change
	
				inp_qei_s.prev_phases = cur_phases;
			} //if (0 != inp_qei_s.ang_inc)
	
			inp_qei_s.tmp_i[0]++; //MB~
		} // if (inp_qei_s.prev_phases != cur_phases)
		else inp_qei_s.tmp_i[1]++; //MB~
	} // if (THR_TICKS_PER_QEI < diff_time)
	else // MB~ Depreciated
	{
		if (inp_qei_s.prev_phases != cur_phases)
		{ // Update angular position
			inp_qei_s.tmp_i[2]++; //MB~
		} // if (inp_qei_s.prev_phases != cur_phases)
		else inp_qei_s.tmp_i[3]++; //MB~
	} // else !(THR_TICKS_PER_QEI < diff_time)

	inp_qei_s.prev_time = edge_time; // Store time-stamp

	return;
} // update_et_phase_states
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
	err_flg = !(inp_pins & QEI_NERR_MASK); 	// NB Bit_3=0, and err_flg=1, if error detected, 


	// Check for first data
	if (0 == inp_qei_s.pin_changes)
	{ // Initialise 'previous data'
		inp_qei_s.prev_phases = cur_phases; // Store phase value
		inp_qei_s.prev_orig = orig_flg; // Store origin flag value
		inp_qei_s.params.err = err_flg;

		inp_qei_s.pin_changes = 1; // Update number of pin-changes

#if (1 == QEI_RS_MODE)
		update_first_phase( inp_qei_s.phase_data[QEI_PHASE_A] ,(cur_phases & 0b01) ); // Phase_A
		update_first_phase( inp_qei_s.phase_data[QEI_PHASE_B] ,((cur_phases & 0b10) >> 1) ); // Phase_B
#endif // if (1 == QEI_RS_MODE)
	} // if (0 == inp_qei_s.pin_changes)
	else
	{ // NOT first data
#if (1 == QEI_RS_MODE)
		update_rs_phase_states( inp_qei_s ,time_stamp ,cur_phases ); // update phase state
#else // if (1 == QEI_RS_MODE)
		update_et_phase_states( inp_qei_s ,time_stamp ,cur_phases ); // update phase state
#endif // else !(1 == QEI_RS_MODE)
	
		// Check for change in origin state
		if (orig_flg != inp_qei_s.prev_orig)
		{
			update_origin_state( inp_qei_s ,orig_flg ); // update origin state
	
			inp_qei_s.prev_orig = orig_flg; // Store origin flag value
		} // if (orig_flg != inp_qei_s.prev_orig)
	
		// Check for change in error state
		if (err_flg != inp_qei_s.params.err)
		{
			// Update estimate of error status using new error flag
			estimate_error_status( inp_qei_s ,err_flg ); // NB Updates inp_qei_s.params.err
		} // if (err_flg != inp_qei_s.params.err)
	} // else !(0 == inp_qei_s.pin_changes)

	return;
} // service_input_pins
/*****************************************************************************/
#if (1 == QEI_RS_MODE)  // Following functions for Regular-Sampling Mode
/*****************************************************************************/
void foc_qei_config(  // Configure all QEI ports
	buffered QEI_PORT in pb4_QEI[NUMBER_OF_MOTORS], // Array of buffered 4-bit input ports (carries raw QEI motor data)
	clock qei_clks[NUMBER_OF_MOTORS] // Array of clocks for generating accurate QEI timing (one per input port)
)
{
	timer chronometer; // H/W timer
	unsigned big_ticks; // ticks from 32-bit timer
	unsigned tmp_ticks; // ticks from 32-bit timer
	int motor_cnt; // motor counter


	assert( HALF_PERIOD < 256 ); // Check for illegal value

	// Once all ports configured, synchronise port-timers with reference clock
	chronometer :> big_ticks;	// Get current time
	big_ticks &= ~0xffff;			// Clear bottom 16 bits
	big_ticks += 0x20000;			// Step on time by 2 port-timer cycles, too allow some time configure ports

	// Loop through all ports to be configured
	for (motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++)
	{
		// NB We want to sample the QEI data every 128 ticks (for both motors).
		configure_clock_ref( qei_clks[motor_cnt] ,HALF_PERIOD ); // Configure clock rate to PLATFORM_REFERENCE_MHZ/(2*HALF_PERIOD) (100/128 MHz)

		do_qei_port_config( pb4_QEI[motor_cnt] ,qei_clks[motor_cnt] ); // configure current port

		// Check timing has been met
		chronometer :> tmp_ticks;
		assert( big_ticks > tmp_ticks); // ERROR: Timing NOT met
 
		// Stagger when each port start sampling, so buffer servicing is interleaved
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
	unsigned stag_inc = STAG_TICKS; // Used to stagger servicing of port buffers. NB use a variable to trick the compiler!-(
	unsigned stag_off; // Initial value of staggered tick offset (for 1st motor)
	PORT_TIME_TYP port16_cnt; // port sample counter (16-bit value)
	PORT_TIME_TYP prev16_cnt[NUMBER_OF_MOTORS]; // previous port sample cnt (16-bit value)
	PORT_TIME_TYP diff16_ticks; // difference between port times (16-bit value)
	QEI_RAW_TYP inp_pins; // raw data value from input port pins
	CMD_QEI_ENUM inp_cmd; // QEI command from Client
	unsigned buf_data = 0;
	int time_err = 0; // Timing error count

	int motor_cnt; // Motor counter
	int samp_cnt; // Sample counter
	int do_loop = 1;   // Flag set until loop-end condition found

#ifdef MB
unsigned dbg_orig = 0; // MB~
unsigned dbg_strt = 0;
unsigned dbg_end = 0;
unsigned dbg_diff; // MB~
#endif //MB~


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

	for (int motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; ++motor_cnt) 
	{
		init_qei_data( all_qei_s[motor_cnt] ,motor_cnt ); // Initialise QEI data for current motor

		chronometer :> samp32_time;	// Get current time
		all_qei_s[motor_cnt].change_time = samp32_time; // Initialise time-stamp with sensible value
		all_qei_s[motor_cnt].prev_change = samp32_time; // Initialise time-stamp with sensible value
		all_qei_s[motor_cnt].prev_time = samp32_time; // Initialise time-stamp with sensible value

		// Use acknowledge command to signal to control-loop that initialisation is complete
		acknowledge_qei_command( all_qei_s[motor_cnt] ,c_qei[motor_cnt] );

		{buf_data ,samp32_time} = partin_timestamped( pb4_inp[motor_cnt] ,4 ); // Read 1st (dummy) value to get timestamp

		prev16_cnt[motor_cnt] = (PORT_TIME_TYP)samp32_time;
	} // for motor_cnt

// chronometer :> dbg_orig; // MB~

	while (do_loop) 
	{
#pragma xta endpoint "qei_main_loop"
		stag_off = 0; // Initial value of staggered tick offset (for 1st motor)
		for (motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++)
		{
			// Read 8 4-bit data samples from QEI port buffer
			pb4_inp[motor_cnt] :> buf_data @ port16_cnt; // Get all buffer samples (8)
			chronometer :> samp32_time; // Get 32-bit timer value to use as sample time-stamp
// xscope_int( (8+motor_cnt) ,port16_cnt ); //MB~

// chronometer :> dbg_strt; // MB~
			// Check Timing has been met ...

			diff16_ticks = (PORT_TIME_TYP)(port16_cnt - prev16_cnt[motor_cnt]);

			if (diff16_ticks > SAMPS_PER_LOOP)
			{
				time_err++; // Increment error count
// xscope_int( (8+motor_cnt) ,time_err ); //MB~
				assert(time_err <= MAX_TIME_ERR); // ERROR: Too many Input QEI port samples dropped (Increase HALF_PERIOD)
			} // if (diff16_ticks > SAMPS_PER_LOOP)
			else
			{
				if (time_err > 0) time_err--; // Decrement error count
			} // if (diff16_ticks > SAMPS_PER_LOOP)

			// Read individual samples from buffer 
			for (samp_cnt=0; samp_cnt<SAMPS_PER_LOOP; samp_cnt++)
			{
				inp_pins = buf_data & QEI_SAMP_MASK; // mask out LS 4 bits

				// NB As only the difference between time-stamps is used, the fixed delay cancels out
				service_input_pins( all_qei_s[motor_cnt] ,samp32_time ,inp_pins );

				buf_data >>= QEI_SAMP_BITS; // Shift next sample to LS end of buffer
				samp32_time += (unsigned)TICKS_PER_SAMP; // Increment time to end next sample
			} // for samp_cnt

// chronometer :> dbg_end; // MB~
			select {
				case c_qei[motor_cnt] :> inp_cmd :
				{
					switch(inp_cmd)
					{
						case QEI_CMD_DATA_REQ : // Data Request
							service_client_data_request( all_qei_s[motor_cnt] ,c_qei[motor_cnt] );
// xscope_int( (8+motor_cnt) ,(dbg_end - dbg_strt) ); //MB~
						break; // case QEI_CMD_DATA_REQ
	
						case QEI_CMD_LOOP_STOP : // Termination Command
							acknowledge_qei_command( all_qei_s[motor_cnt] ,c_qei[motor_cnt] );
	
							do_loop = 0; // Terminate while loop
						break; // case QEI_CMD_DATA_REQ
	
						default : // Unknown Command
							assert(0 == 1); // ERROR: Should not happen
						break; // default
					} // switch(inp_cmd)
				} // case	c_qei[motor_cnt] :> inp_cmd :
				break;
	
				default :
				break; // default
			} // select

			prev16_cnt[motor_cnt] = port16_cnt; // Store port time-stamp
			stag_off += stag_inc; // Update staggered offset ready for next motor
		} // for motor_cnt

	}	// while (do_loop)

#if (QEI_DBG)
	print_all_dbg( all_qei_s[0] ); // MB~ Dbg
#endif //(QEI_DBG)

	acquire_lock(); 
	printstrln("");
	printstrln("                                             QEI Server Ends");
	release_lock();

	return;
} // foc_qei_do_multiple
/*****************************************************************************/
#else // if (1 == QEI_RS_MODE)  // Following functions for Edge-Trigger Mode
/*****************************************************************************/
void foc_qei_config(  // Configure all QEI ports
	buffered QEI_PORT in pb4_QEI[NUMBER_OF_MOTORS], // Array of buffered 4-bit input ports (carries raw QEI motor data)
	clock qei_clks[NUMBER_OF_MOTORS] // Array of clocks for generating accurate QEI timing (one per input port)
)
{
	timer chronometer; // H/W timer
	unsigned big_ticks; // ticks from 32-bit timer
	int motor_cnt; // motor counter


 	configure_clock_rate( qei_clks[0] ,PLATFORM_REFERENCE_MHZ ,1 ); // Configure clock rate to PLATFORM_REFERENCE_MHZ/1 (100 MHz)

	// Loop through all ports to be configured
	for (motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++)
	{
		do_qei_port_config( pb4_QEI[motor_cnt] ,qei_clks[0] ); // configure current port
	} // for (motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++) 

	// Once all ports configured, synchronise port-timers with reference clock
	chronometer :> big_ticks;												// Get current time
	big_ticks &= ~0xffff;														// Clear bottom 16 bits
	big_ticks += 0x20000;														// Step on time by 2 port-timer cycles
	chronometer when timerafter(big_ticks) :> void;	// Wait until synchronisation time

	start_clock( qei_clks[0] ); // Start common QEI clock, 
} // foc_qei_config
/*****************************************************************************/
#pragma unsafe arrays
void foc_qei_do_multiple( // Get QEI data from motor and send to client
	streaming chanend c_qei[], // Array of data channel to client (carries processed QEI data)
	buffered QEI_PORT in pb4_QEI[NUMBER_OF_MOTORS] // Array of buffered 4-bit input ports (carries raw QEI motor data)
)
#define STAT_BITS 12
#define NUM_STATS (1 << STAT_BITS)
#define HALF_STATS (NUM_STATS >> 1)
{
	QEI_BUF_TYP buffer[QEI_BUF_SIZ]; // Buffers raw QEI values
	QEI_DATA_TYP all_qei_s[NUMBER_OF_MOTORS]; // Array of structures containing QEI parameters for all motor
	QEI_RAW_TYP inp_pins[NUMBER_OF_MOTORS]; // Array of raw data values on input port pins
	QEI_RAW_TYP tmp_pins; // temporary raw data value from input port pins
	CMD_QEI_ENUM inp_cmd; // QEI command from Client
	timer chronometer; // H/W timer
	unsigned approx32_ticks; // Approximate port sample time (32-bit value)
	unsigned exact32_ticks; // Exact port sample time (32-bit value)
	PORT_TIME_TYP port16_ticks; // Exact port sample time (16-bit value)
	PORT_TIME_TYP low16_ticks; // lowest 16-bits of 32-bit time (16-bit value)
	PORT_TIME_TYP corr16_ticks; // timer correction value (16-bit value)
	int motor_id; // Unique Motor identifier	
	int read_cnt = 0; // No of QEI values read from buffer
	int write_cnt = 0; // No of QEI values written to buffer
	unsigned read_off = 0; // read offset into buffer
	unsigned write_off = 0; // wtite offset into buffer
	int do_loop = 1;   // Flag set until loop-end condition found

	acquire_lock(); 
	printstrln("                                          QEI ET_Server Starts");
	release_lock();

{
	unsigned pre_ticks; // Timer value
	unsigned pst_ticks; // Timer value
	unsigned prt_ticks; // Timer value

	chronometer :> pre_ticks;
	pb4_QEI[0] :> tmp_pins @ prt_ticks; // Re-sample to test for glitch
	chronometer :> pst_ticks;

	acquire_lock(); 
	printstr(" PRE="); printhexln(pre_ticks ); 
	printstr(" PRT="); printhexln(prt_ticks ); 
	printstr(" PST="); printhexln(pst_ticks ); 
	release_lock();
}
	// Check if we are running on the simulator
	if(0 == _is_simulation())
	{ // Running on real hardware
		// Wait 128ms for UV_FAULT pin to finish toggling
		chronometer :> exact32_ticks;
		chronometer when timerafter(exact32_ticks + (MILLI_SEC << 7)) :> void;
	} // if (0 == _is_simulation())

	for (int motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; ++motor_cnt) 
	{
		inp_pins[motor_cnt] = 0xFF; // Set buffer for reading input pins to impossible value
		init_qei_data( all_qei_s[motor_cnt] ,motor_cnt ); // Initialise QEI data for current motor

		chronometer :> approx32_ticks;
		all_qei_s[motor_cnt].prev_time = approx32_ticks; // Initialise previous time-stamp with sensible value
		all_qei_s[motor_cnt].change_time = approx32_ticks; // Initialise time-stamp with sensible value
		all_qei_s[motor_cnt].prev_change = approx32_ticks; // Initialise time-stamp with sensible value

		// Use acknowledge command to signal to control-loop that initialisation is complete
		acknowledge_qei_command( all_qei_s[motor_cnt] ,c_qei[motor_cnt] );
	} // for motor_cnt

	while (do_loop) {
#pragma xta endpoint "qei_main_loop"
#pragma ordered // If multiple cases fire at same time, service top-most first
		select {
			// Service any change on input port pins
			case (int motor_id=0; motor_id<NUMBER_OF_MOTORS; motor_id++) pb4_QEI[motor_id] when pinsneq(inp_pins[motor_id]) :> inp_pins[motor_id] @ port16_ticks:
			{
all_qei_s[motor_id].tmp_raw++;
				pb4_QEI[motor_id] :> tmp_pins; // Re-sample to test for glitch

				// WARNING: H/W pin-change detector sometimes mis-fires, so also do check in S/W
				if (tmp_pins == inp_pins[motor_id])
				{
					// Build accurate 32-bit port time value ...
					chronometer :> approx32_ticks; // Get approximate 32-bit timer value

					buffer[write_off].time16 = port16_ticks; // Store exact 16-bit timer value
					buffer[write_off].time32 = approx32_ticks; // Store approximate 32-bit timer value
					buffer[write_off].inp_pins = tmp_pins;
					buffer[write_off].id = motor_id;
	
					// new QEI data written to buffer
					write_cnt++; // Increment write counter.  WARNING No overflow check
					write_off = write_cnt & QEI_BUF_MASK; // Wrap into buffer range
	
					assert( (write_cnt - read_cnt) < QEI_BUF_MASK); // Check for buffer overflow
				} // if (tmp_pins == inp_pins[motor_id])
			} // case
			break;

			// Service any client command
			case (int motor_id=0; motor_id<NUMBER_OF_MOTORS; motor_id++) c_qei[motor_id] :> inp_cmd :
			{
				switch(inp_cmd)
				{
					case QEI_CMD_DATA_REQ : // Data Request
						service_client_data_request( all_qei_s[motor_id] ,c_qei[motor_id] );
					break; // case QEI_CMD_DATA_REQ

					case QEI_CMD_LOOP_STOP : // Termination Command
						acknowledge_qei_command( all_qei_s[motor_id] ,c_qei[motor_id] );

						do_loop = 0; // Terminate while loop
					break; // case QEI_CMD_DATA_REQ

					default : // Unknown Command
						assert(0 == 1); // ERROR: Should not happen
					break; // default
				} // switch(inp_cmd)
			} // case	c_qei[motor_id] :> inp_cmd :
			break;

			default :
				// Check if any buffer data needs processing
				if (write_cnt > read_cnt)
				{
					motor_id = buffer[read_off].id;	

					low16_ticks = (PORT_TIME_TYP)(buffer[read_off].time32 & PORT_TIME_MASK); // Get lowest 16-bits of 32-bit timer

					// Calculate timer correction. NB correctly handles wrapped timer values
					corr16_ticks = (PORT_TIME_TYP)(low16_ticks - buffer[read_off].time16);
					buffer[read_off].time32 = buffer[read_off].time32 - (unsigned)corr16_ticks; // Correct 32-bit timer value

					service_input_pins( all_qei_s[motor_id] ,buffer[read_off].time32 ,buffer[read_off].inp_pins );

					read_cnt++; // Increment read counter. WARNING No overflow check
					read_off = read_cnt & QEI_BUF_MASK; // Wrap into buffer range
				} // if (write_off != read_off)
			break; // default
		} // select
	}	// while (do_loop)

#if (QEI_DBG)
	print_all_dbg( all_qei_s[0] ); // MB~ Dbg
#endif //(QEI_DBG)

{ //MB~
	int cc;
	acquire_lock(); 

	for (int motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; ++motor_cnt) 
	{
		printstr("M_");
		printintln(motor_cnt);

		for (cc=0; cc<5; cc++)
		{
			printstr(" ");
			printint(all_qei_s[motor_cnt].tmp_s[cc]);
		} // for cc
		printstrln(" <S");
	
		for (cc=0; cc<4; cc++)
		{
			printstr(" ");
			printint(all_qei_s[motor_cnt].tmp_i[cc]);
		} // for cc
		printstrln(" <I");
	} // for motor_cnt
} //MB~

	acquire_lock(); 
	printstrln("");
	printstrln("                                             QEI Server Ends");
	release_lock();

	return;
} // foc_qei_do_multiple
/*****************************************************************************/
#endif // else !(1 == QEI_RS_MODE)
/*****************************************************************************/
