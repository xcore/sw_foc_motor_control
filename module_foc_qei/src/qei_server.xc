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
static void init_qei_data( // Initialise  QEI data for one motor
	QEI_DATA_TYP &inp_qei_s, // Reference to structure containing QEI parameters for one motor
	QEI_RAW_TYP &inp_pins, // raw data value on input port pins
	int inp_id  // Input unique motor identifier
)
{
	inp_pins = 0xFF; // Set buffer for reading input pins to impossible value

	inp_qei_s.params.theta = 0; // Reset angular position returned to client
	inp_qei_s.params.rev_cnt = 0; // Reset revolution counter  returned to client
	inp_qei_s.params.veloc = 0; // Clear velocity returned to client
	inp_qei_s.params.err = QEI_ERR_OFF; // Clear error status flag returned to client

	inp_qei_s.diff_time = 0; // NB Initially this is used to count input-pin changes
	inp_qei_s.id = inp_id; // Clear Previous phase values
	inp_qei_s.ang_cnt = 0; // Reset counter indicating angular position of motor (from origin)
	inp_qei_s.spin_sign = 0; // Clear Sign of spin direction
	inp_qei_s.prev_state = QEI_STALL; // Initialise previous QEI state
	inp_qei_s.state_errs = 0; // Initialise counter for invalid QEI state transitions
	inp_qei_s.status_errs = 0; // Initialise counter for QEI status errors
	inp_qei_s.confid = 1; // Initialise confidence value

	inp_qei_s.prev_time = 0;
	inp_qei_s.prev_orig = 0;
	inp_qei_s.prev_phases = 0;

	inp_qei_s.filt_val = 0; // filtered value
	inp_qei_s.coef_err = 0; // Coefficient diffusion error
	inp_qei_s.scale_err = 0; // Scaling diffusion error
	inp_qei_s.speed_err = 0; // Speed diffusion error

	inp_qei_s.dbg = 0;

	return;
} // init_qei_data
/*****************************************************************************/
static ANG_INC_TYP estimate_angular_increment( // Estimate angular position increment from QEI states
	QEI_DATA_TYP &inp_qei_s, // Reference to structure containing QEI parameters for one motor
	unsigned cur_phases // Current set of phase values
) // Returns output angular increment value (one of [-2, -1, 0, 1, 2])
{
/* get_spin is a table for converting pins values to spin values
 *	[BA] order is 00 -> 01 -> 11 -> 10  Clockwise direction
 *	[BA] order is 00 -> 10 -> 11 -> 01  Anti-Clockwise direction
 *
 *	Spin-state is
 *		-1: ANTI Anti-Clocwise rotation,
 *		 0: STALL (The motor has probably stopped)
 *		 1: CLOCK Clockwise rotation,
 *		 2: JUMP (The motor has probably jumped 2 phases)
 *
 *	NB We are going to use a convention that a Clock-wise spin has +ve value.
 *	This does NOT mean the motor will spin clock-wise!
 *	This depends on the observers position relative to the motor.
 *	The following table satisfies the above convention when accessed as:-
 *		Spin = get_spin_state[Old_Phase][New_phase]
 */
static const QEI_ENUM_TYP get_spin_state[NUM_QEI_PHASES][NUM_QEI_PHASES] = {
		{ QEI_STALL,	QEI_ANTI,		QEI_CLOCK,	QEI_JUMP	},
		{ QEI_CLOCK,	QEI_STALL,	QEI_JUMP,		QEI_ANTI	},
		{ QEI_ANTI,		QEI_JUMP,		QEI_STALL,	QEI_CLOCK	},
		{ QEI_JUMP,		QEI_CLOCK,	QEI_ANTI,		QEI_STALL	}
};

	QEI_ENUM_TYP cur_state = get_spin_state[cur_phases][inp_qei_s.prev_phases]; // Estimated new spin-state fom phase info.
	QEI_ENUM_TYP prev_state = inp_qei_s.prev_state; // local copy of previous QEI-state
	ANG_INC_TYP out_ang_inc; // Output angular increment value


	// Update spin Finite-State-Machine ...

	// Check bit_0 of previous state
	if (prev_state & 0x1)
	{ // Valid previous spin state (CLOCK or ANTI)

		// Check bit_0 of current state
		if (cur_state & 0x1)
		{ // Valid current spin state (CLOCK or ANTI)
			out_ang_inc = prev_state; // Use previous state (For this case state-value is increment-value)

			// Check for change of valid state (CLOCK <--> ANTI)
			if (cur_state != prev_state)
			{
				// Check confidence levels
				if (0 < inp_qei_s.confid)
				{ // Still have some confidence
					cur_state = prev_state; // Revert to previous state
					inp_qei_s.confid--; // Decrease confidence level
				} // if (0 < inp_qei_s.confid)
				else
				{ // No confidence left. (Move to STALL state next time)
					cur_state = QEI_STALL; // Revert to previous state
					inp_qei_s.state_errs = 0; // Reset count of invalid states
				} // else !(0 < inp_qei_s.confid)
			} // if (cur_state != prev_state)
			else
			{ // Normal Case: Maintaining same valid spin direction
				if (MAX_CONFID > inp_qei_s.confid) inp_qei_s.confid++; // Increase confidence level
			} // else !(cur_state != prev_state)
		} // if (cur_state & 0x1)
		else
		{ // Invalid current spin state (STALL or JUMP)
			if (QEI_JUMP != cur_state)
			{ // cur_state == QEI_STALL
				out_ang_inc = prev_state; // Stay with previous state for possible STALL
			} // if (QEI_JUMP != cur_state)
			else
			{ // cur_state == QEI_JUMP
				out_ang_inc = (prev_state << 1); // Double increment as phase jump detected
			} // else !(QEI_JUMP != cur_state)

			// Check confidence levels
			if (0 < inp_qei_s.confid)
			{ // Still have some confidence
				cur_state = prev_state; // Revert to previous state
				inp_qei_s.confid--; // Decrease confidence level
			} // if (0 < inp_qei_s.confid)
			else
			{ // No confidence left. (Will move to current invalid state next time)
				inp_qei_s.state_errs = 0; // Reset count of invalid states
			} // else !(0 < inp_qei_s.confid)
		} // else !(cur_state & 0x1)
	} // if (prev_state & 0x1)
	else
	{ // Invalid previous spin state

		// Check bit_0 of current state
		if (cur_state & 0x1)
		{ // Valid current spin state (CLOCK or ANTI)
			out_ang_inc = cur_state; // Use new state value

			if (MAX_CONFID > inp_qei_s.confid) inp_qei_s.confid++; // Increase confidence level
		} // if (cur_state & 0x1)
		else
		{ // Invalid current spin state (STALL or JUMP)
			out_ang_inc = 0; // Zero Spin for invalid state

			assert(MAX_QEI_STATE_ERR > inp_qei_s.state_errs); // Too Many QEI errors

			inp_qei_s.state_errs++; // Increment invalid state cnt
		} // else !(cur_state & 0x1)
	} // else !(prev_state & 0x1)

	inp_qei_s.prev_state = cur_state; // Store old QEI state value

	return out_ang_inc; // Return output increment value
} // estimate_angular_increment
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
static void check_for_missed_origin( // Check for missed origin, and update angular position info. if necessary
	QEI_DATA_TYP &inp_qei_s // Reference to structure containing QEI parameters for one motor
)
/*
 *	This function checks for a 'missed origin' and ensures theta is in the correct range ...
 *	The local angular position (inp_qei_s.ang_cnt) should be in the range
 *	-540 Degrees <= ang_cnt <= 540 degrees
 *  If inp_qei_s.ang_cnt is outside the above range, an Origin signal has been missed.
 *
 * The if-then-else statements are arranged with the most likely first to reduce computation
 */
{
	if (0 > inp_qei_s.ang_cnt)
	{ // Negative angles
		if (inp_qei_s.ang_cnt < (-QEI_PER_REV))
		{ // A few mis-interpreted values
			if (inp_qei_s.ang_cnt < (-QEI_CNT_LIMIT))
			{ // Too many mis-interpreted values: Origin Missed --> Correct counters
				inp_qei_s.params.rev_cnt--; // Decrement origin counter
				inp_qei_s.ang_cnt += QEI_PER_REV; // 'Unwind' a whole (anti-clockwise) rotation
			} // (inp_qei_s.ang_cnt < -QEI_CNT_LIMIT)
		} // if (inp_qei_s.ang_cnt < -QEI_CNT_LIMIT)
	} // if (0 > inp_qei_s.ang_cnt)
	else
	{ // Positive angles
		if (inp_qei_s.ang_cnt > QEI_PER_REV)
		{ // A few mis-interpreted values
			if (inp_qei_s.ang_cnt > QEI_CNT_LIMIT)
			{ // Too many mis-interpreted values: Origin Missed --> Correct counters
				inp_qei_s.params.rev_cnt++; // Increment origin counter
				inp_qei_s.ang_cnt -= QEI_PER_REV; // 'Unwind' a whole (clock-wise) rotation
			} // (inp_qei_s.ang_cnt > QEI_CNT_LIMIT)
		} // if (inp_qei_s.ang_cnt > QEI_PER_REV)
	} // else !(0 > inp_qei_s.ang_cnt)

} // check_for_missed_origin
/*****************************************************************************/
static void update_qei_state( // Update QEI state
	QEI_DATA_TYP &inp_qei_s, // Reference to structure containing QEI parameters for one motor
	unsigned cur_phases, // Current set of phase values
	unsigned orig_flg, // Flag set when motor at origin position
	ERROR_QEI_ENUM err_flg // Flag set when Error condition detected
)
{
	ANG_INC_TYP ang_inc; // angular increment value


	// Update estimate of error status using new error flag
	estimate_error_status( inp_qei_s ,err_flg ); // NB Updates inp_qei_s.params.err

	// Update estimate of angular increment value using new spin state
	ang_inc = estimate_angular_increment( inp_qei_s ,cur_phases );

	inp_qei_s.ang_cnt += ang_inc; // Increment/Decrement angular position

	// Update estimate of spin direction based on angular increment
	if (ang_inc < 0)
	{
		inp_qei_s.spin_sign = -1; // -ve spin direction
	} // if (ang_inc < 0)
	else
	{
		inp_qei_s.spin_sign = 1;  // +ve spin direction
	} // else !(ang_inc < 0)

	// Check if motor at origin
	if (orig_flg != inp_qei_s.prev_orig)
	{
		if (orig_flg)
		{ // Reset position ( 'orig_flg' transition  0 --> 1 )
			inp_qei_s.params.rev_cnt += inp_qei_s.spin_sign; // Update origin counter
			inp_qei_s.ang_cnt = 0; // Reset position value
		} // if (orig_flg)

		inp_qei_s.prev_orig = orig_flg; // Store new orign flag value
	} // if (orig_flg != inp_qei_s.prev_orig)

	check_for_missed_origin( inp_qei_s ); // NB May update inp_qei_s.ang_cnt & inp_qei_s.params.rev_cnt

 	// The theta value returned to the client should be in the range:  0 <= ang_cnt < 360 degrees
	inp_qei_s.params.theta = (inp_qei_s.ang_cnt & QEI_REV_MASK); // force into range [0..QEI_REV_MASK]

	return;
} // update_qei_state
/*****************************************************************************/
static void update_speed( // Update speed estimate with from time period. (Angular_speed in RPM)
	QEI_DATA_TYP &inp_qei_s // Reference to structure containing QEI parameters for one motor
)
{
	unsigned ticks = inp_qei_s.diff_time; // ticks per QEI position (in Reference Frequency Cycles)
	int const_val = TICKS_PER_MIN_PER_QEI + inp_qei_s.speed_err; // Add diffusion error to constan


	inp_qei_s.ang_speed = (const_val + (ticks >> 1)) / ticks;  // Evaluate speed
	inp_qei_s.speed_err = const_val - (inp_qei_s.ang_speed * ticks); // Evaluate new remainder value
}	// update_speed
/*****************************************************************************/
static void service_input_pins( // Service detected change on input pins
	QEI_DATA_TYP &inp_qei_s, // Reference to structure containing QEI parameters for one motor
	QEI_RAW_TYP inp_pins // Set of raw data values on input port pins
)
{
	unsigned cur_phases; // Current set of phase values
	unsigned orig_flg; // Flag set when motor at origin position
	unsigned err_flg; // Flag set when Error condition detected
	int test_diff; // test time difference for sensible value


	cur_phases = inp_pins & QEI_PHASE_MASK; // Extract Phase bits from input pins

	// Check if phases have changed
	if (cur_phases != inp_qei_s.prev_phases)
	{
		// Get new time stamp ..
		test_diff = (int)(inp_qei_s.curr_time - inp_qei_s.prev_time);

		// Check for sensible time
		if (THR_TICKS_PER_QEI < test_diff)
		{ // Sensible time!
			orig_flg = inp_pins & QEI_ORIG_MASK; 		// Extract origin flag
			err_flg = !(inp_pins & QEI_NERR_MASK); 	// NB Bit_3=0, and err_flg=1, if error detected,

			// Determine new QEI state from new pin data
			update_qei_state( inp_qei_s ,cur_phases ,orig_flg ,err_flg );

			// Check for end of start-up phase
			if (START_UP_CHANGES <= inp_qei_s.diff_time)
			{
				inp_qei_s.diff_time = test_diff; // Store sensible time difference

				update_speed( inp_qei_s ); // Update speed value with new time difference
			} // if (START_UP_CHANGES <= inp_qei_s.diff_time)
			else
			{
				inp_qei_s.diff_time++; // Update number of input pin changes
				inp_qei_s.ang_speed = 1;	// Default speed value
			} // if (START_UP_CHANGES <= inp_qei_s.diff_time)

			inp_qei_s.prev_time = inp_qei_s.curr_time; // Store time stamp
			inp_qei_s.prev_phases = cur_phases; // Store phase value
		} // if (THR_TICKS_PER_QEI < test_diff)
	}	// if (cur_phases != inp_qei_s.prev_phases)

	return;
} // service_input_pins
/*****************************************************************************/
static int filter_velocity( // Smooths velocity estimate using low-pass filter
	QEI_DATA_TYP &qei_data_s, // Reference to structure containing QEI parameters for one motor
	int meas_veloc // Angular velocity of motor measured in Ticks/angle_position
) // Returns filtered output value
/* This is a 1st order IIR filter, it is configured as a low-pass filter,
 * The input velocity value is up-scaled, to allow integer arithmetic to be used.
 * The output mean value is down-scaled by the same amount.
 * Error diffusion is used to keep control of systematic quantisation errors.
 */
{
	int scaled_inp = (meas_veloc << QEI_SCALE_BITS); // Upscaled QEI input value
	int diff_val; // Difference between input and filtered output
	int increment; // new increment to filtered output value
	int out_val; // filtered output value


	// Form difference with previous filter output
	diff_val = scaled_inp - qei_data_s.filt_val;

	// Multiply difference by filter coefficient (alpha)
	diff_val += qei_data_s.coef_err; // Add in diffusion error;
	increment = (diff_val + QEI_HALF_COEF) >> QEI_COEF_BITS ; // Multiply by filter coef (with rounding)
	qei_data_s.coef_err = diff_val - (increment << QEI_COEF_BITS); // Evaluate new quantisation error value

	qei_data_s.filt_val += increment; // Update (up-scaled) filtered output value

	// Update mean value by down-scaling filtered output value
	qei_data_s.filt_val += qei_data_s.scale_err; // Add in diffusion error;
	out_val = (qei_data_s.filt_val + QEI_HALF_SCALE) >> QEI_SCALE_BITS; // Down-scale
	qei_data_s.scale_err = qei_data_s.filt_val - (out_val << QEI_SCALE_BITS); // Evaluate new remainder value

	return out_val; // return filtered output value
} // filter_velocity
/*****************************************************************************/
#pragma unsafe arrays
static void service_client_data_request( // Send processed QEI data to client
	QEI_DATA_TYP &inp_qei_s, // Reference to structure containing QEI parameters for one motor
	streaming chanend c_qei // Data channel to client (carries processed QEI data)
)
/*	The speed is calculated assuming the angular change is always 1 position.
 *	Experiment shows this to be more robust than using the actual position change, e.g. one of [0, 1, 2]
 *	This is because, the actual positions are estimates (and are sometimes incorrect)
 *	Whereas using the value 1, is effectively applying a lo-pass filter to the position change.
 *
 *	NB If angular position has NOT updated since last transmission, then the same data is re-transmitted
 */
{
	int meas_veloc = (inp_qei_s.spin_sign * inp_qei_s.ang_speed); // Angular_velocity of motor in RPM


	// Check if filter selected
	if (QEI_FILTER)
	{
		int tmp_veloc = filter_velocity( inp_qei_s ,meas_veloc );

		inp_qei_s.params.veloc = tmp_veloc; // NB XC ambiguous evaluation rule.
	} // if (QEI_FILTER)
	else
	{
		inp_qei_s.params.veloc = meas_veloc;
	} // else !(QEI_FILTER)

	c_qei <: inp_qei_s.params; // Transmit QEI parameters to Client

	return;
} // service_client_data_request
/*****************************************************************************/
#pragma unsafe arrays
static void service_client_stop_request( // Acknowledge termination request from QEI client
	QEI_DATA_TYP &inp_qei_s, // Reference to structure containing QEI parameters for one motor
	streaming chanend c_qei // Data channel to client (carries processed QEI data)
)
{
	inp_qei_s.params.err = QEI_TERMINATED; // Signal QEI Termination to Client

	c_qei <: inp_qei_s.params; // Transmit QEI parameters to Client
} // service_client_stop_request
/*****************************************************************************/
#pragma unsafe arrays
void foc_qei_do_multiple( // Get QEI data from motor and send to client
	streaming chanend c_qei[], // Array of data channel to client (carries processed QEI data)
	buffered port:4 in pb4_qei[] // Array of buffered 4-bit input ports (carries raw QEI motor data)
)
#define STAT_BITS 12
#define NUM_STATS (1 << STAT_BITS)
#define HALF_STATS (NUM_STATS >> 1)

{
	QEI_BUF_TYP buffer[QEI_BUF_SIZ]; // Buffers raw QEI values
	QEI_DATA_TYP all_qei_s[NUMBER_OF_MOTORS]; // Array of structures containing QEI parameters for all motor
	QEI_RAW_TYP inp_pins[NUMBER_OF_MOTORS]; // Set of raw data values on input port pins
	CMD_QEI_ENUM inp_cmd; // QEI command from Client
	timer chronometer; // H/W timer
	int read_cnt = 0; // No of QEI values read from buffer
	int write_cnt = 0; // No of QEI values written to buffer
	unsigned read_off = 0; // read offset into buffer
	unsigned write_off = 0; // wtite offset into buffer
	int do_loop = 1;   // Flag set until loop-end condition found


	acquire_lock();
	printstrln("                                             QEI Server Starts");
	release_lock();


	for (int motor_id=0; motor_id<NUMBER_OF_MOTORS; ++motor_id)
	{
		init_qei_data( all_qei_s[motor_id] ,inp_pins[motor_id] ,motor_id ); // Initialise QEI data for current motor
	} // for motor_id

	while (do_loop) {
#pragma xta endpoint "qei_main_loop"
#pragma ordered // If multiple cases fire at same time, service top-most first
		select {
			// Service any change on input port pins
			case (int motor_id=0; motor_id<NUMBER_OF_MOTORS; motor_id++) pb4_qei[motor_id] when pinsneq(inp_pins[motor_id]) :> inp_pins[motor_id] :
			{
				chronometer :> buffer[write_off].time;	// Get new time stamp as soon as possible
				buffer[write_off].inp_pins = inp_pins[motor_id];
				buffer[write_off].id = motor_id;

				// new QEI data written to buffer
				write_cnt++; // Increment write counter.  WARNING No overflow check
				write_off = write_cnt & QEI_BUF_MASK; // Wrap into buffer range

				assert( (write_cnt - read_cnt) < QEI_BUF_MASK); // Check for buffer overflow
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
						service_client_stop_request( all_qei_s[motor_id] ,c_qei[motor_id] );

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
					int motor_id = buffer[read_off].id;


					all_qei_s[motor_id].curr_time = buffer[read_off].time;
					service_input_pins( all_qei_s[motor_id] ,buffer[read_off].inp_pins );

					read_cnt++; // Increment read counter. WARNING No overflow check
					read_off = read_cnt & QEI_BUF_MASK; // Wrap into buffer range
				} // if (write_off != read_off)
			break; // default
		} // select
	}	// while (do_loop)

	acquire_lock();
	printstrln("");
	printstrln("                                             QEI Server Ends");
	release_lock();

	return;
} // foc_qei_do_multiple
/*****************************************************************************/
