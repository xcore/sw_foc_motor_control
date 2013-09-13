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
	QEI_PHASE_TYP clkwise = {{ 0 ,1 ,3 ,2 }};	// Table to convert QEI Phase value to circular index [BA] (NB Increment for clock-wise rotation)
	unsigned diff_bits; // No. of bits required to represent max. time-difference between QEI changes
	unsigned tmp_diff; // temporary time difference


	inp_qei_s.dd.cnt = 0; // MB~ Dbg

	inp_qei_s.inv_phase = clkwise; // Assign table converting QEI Phase value to circular index

	inp_pins = 0xFF; // Set buffer for reading input pins to impossible value

	inp_qei_s.params.theta = 0; // Reset angular position returned to client
	inp_qei_s.params.rev_cnt = 0; // Reset revolution counter  returned to client
	inp_qei_s.params.veloc = 0; // Clear velocity returned to client
	inp_qei_s.params.err = QEI_ERR_OFF; // Clear error status flag returned to client

	inp_qei_s.pin_changes = 0; // NB Initially this is used to count input-pin changes
	inp_qei_s.id = inp_id; // Clear Previous phase values
	inp_qei_s.ang_cnt = 0; // Reset counter indicating angular position of motor (from origin)
	inp_qei_s.ang_inc = 0; // Reset angular position increment
	inp_qei_s.prev_inc = 0;
	inp_qei_s.curr_state = QEI_BIT_ERR; // Initialise current QEI state
	inp_qei_s.state_errs = 0; // Initialise counter for invalid QEI state transitions
	inp_qei_s.status_errs = 0; // Initialise counter for QEI status errors
	inp_qei_s.confid = 1; // Initialise spin-direction confidence value (1: Marginal Clockwise probability)

	inp_qei_s.prev_diff = 0;
	inp_qei_s.prev_time = 0;
	inp_qei_s.prev_orig = 0;
	inp_qei_s.prev_phases = 0;
	inp_qei_s.phase_index = 0;
	inp_qei_s.prev_index = 3;

	inp_qei_s.filt_val = 0; // filtered value
	inp_qei_s.coef_err = 0; // Coefficient diffusion error
	inp_qei_s.scale_err = 0; // Scaling diffusion error 
	inp_qei_s.speed_err = 0; // Speed diffusion error 

	inp_qei_s.t_dif_old = 0; // Initialise to unassigned time difference
	inp_qei_s.t_dif_cur = 0; // Initialise to unassigned time difference

	inp_qei_s.dbg_str[2] = 0; // String Terminator
	inp_qei_s.dbg = 0;

	// Calculate how much down-scaling of time-differences is required to maintain precision ...

	// Evaluate how many bits required for maximum time difference
	diff_bits = 0; // Initialise number of time-diff bits to zero
	tmp_diff = MAX_TIME_DIFF; // Load maximum expected time difference
	while(0 < tmp_diff)
	{
		tmp_diff >>= 1;
		diff_bits++;
	} // while(0 < tmp_diff)

	return;
} // init_qei_data
/*****************************************************************************/
static ANG_INC_TYP estimate_increment_bound( // Estimate bound on angular increment given time-diff
	QEI_DATA_TYP &inp_qei_s, // Reference to structure containing QEI parameters for one motor
	int factor // time-diff scaling factor
) // Returns output angular increment value
{
	ANG_INC_TYP out_a_inc = 1; // Clear Output angular increment value
	int diff_est; // time difference estimate
	int tst_diff = (int)inp_qei_s.t_dif_new; // Initialise test-value time-difference
	int prev_tst; // Previous test-value


	diff_est = (factor * (int)inp_qei_s.t_dif_cur + HALF_QEI_SCALE) >> SCALE_QEI_BITS;
	prev_tst = tst_diff;
	tst_diff = prev_tst - diff_est;

	if (0 >= tst_diff) return out_a_inc; // Return if exit criterion already met

	while (( 0 < tst_diff) && (out_a_inc < NUM_QEI_PHASES)) 
	{
		diff_est = (factor * diff_est + HALF_QEI_SCALE) >> SCALE_QEI_BITS; // Scale time-diff estimate 
		prev_tst = tst_diff;
		tst_diff = prev_tst - diff_est;
		out_a_inc++; // increment lower bound for angular increment
	} // while (0 < tst_diff) etc

	prev_tst <<= 1; // NB Double prev_tst for closest estimate check

	// Decrement angle, if older estimate is closest, AND angle has been incremented
	if ((prev_tst <= diff_est) && (1 < out_a_inc))
	{ // new estimate was closest.
		out_a_inc--; // decrement change in angular position
	} // if ((tst_diff > diff_est) || (0 == out_a_inc))

	// Clip into QEI Phase range
	if (out_a_inc > QEI_PHASE_MASK) out_a_inc = QEI_PHASE_MASK;

	return out_a_inc;
} // estimate_increment_bound
/*****************************************************************************/
static ANG_INC_TYP estimate_angle_increment( // Estimate angle increment and qei-state
	QEI_DATA_TYP &inp_qei_s, // Reference to structure containing QEI data for one motor
	unsigned cur_phases // Current set of phase values
) // Return estimate of angle increment
{
	ANG_INC_TYP phase_inc; // Raw No. of angular increments from phase values
	ANG_INC_TYP new_ang_inc; // New estimated angular increment value


	// Determine probable range of angular increment values from time-differences ...

	// Find lower bound for angular increment, by assuming maximum deceleration
	inp_qei_s.lo_inc = estimate_increment_bound( inp_qei_s ,HI_QEI_SCALE );

	// Find higher bound for angular increment, by assuming maximum acceleration
	inp_qei_s.hi_inc = estimate_increment_bound( inp_qei_s ,LO_QEI_SCALE );

	// NB We have now isolated the increment estimate to the range [inp_qei_s.lo_inc .. inp_qei_s.hi_inc]

	inp_qei_s.prev_state = inp_qei_s.curr_state; // Store old QEI state value
	inp_qei_s.prev_index = inp_qei_s.phase_index; // Store old circular phase index
	inp_qei_s.phase_index = inp_qei_s.inv_phase.vals[cur_phases];	// Convert phase val into circ. index [0..QEI_PHASE_MASK]

	// Force phase increment into range [0..QEI_PHASE_MASK]
	phase_inc = (inp_qei_s.phase_index + NUM_QEI_PHASES - inp_qei_s.prev_index) & QEI_PHASE_MASK;

/* The Decision table for estimating angle increments and QEI-states looks like this ...
 *
 *	Numbers are angle increments (QEI steps)
 *	H/L = High/Low probability,   A/C = Anti-clockwise/Clock-wise spin,   be = Bit-Errors
 *              
 *	  Bounds	 |    1			2			3  <-- Phase_inc	
 *	-----------+-------------------
 *	Lo=1 Hi=1  |  HC_1  be_1  HA_1
 *	Lo=1 Hi=2  |  be_2  be_2  be_2
 *	Lo=2 Hi=2  |  be_2  be_2  be_2
 *	Lo=1 Hi=3  |  be_2  be_2  be_2
 *	Lo=2 Hi=3  |  be_2  be_2  be_2
 *	Lo=3 Hi=3  |  LA_3  be_3  LC_3
 */

	// Estimate angle increment and QEI-state ...

	// Check for be_2 states
	if ((1 < inp_qei_s.hi_inc) && (inp_qei_s.lo_inc < QEI_PHASE_MASK))
	{ // Most likely 1 or more bit errors occured
		inp_qei_s.curr_state = QEI_BIT_ERR; // 1 or 2 Bit-Errors occured
		new_ang_inc = 2; // Use '1-bit error' time estimate
	} // if ((1 < inp_qei_s.hi_inc) && (inp_qei_s.lo_inc < QEI_PHASE_MASK))
	else
	{ // lo_inc = hi_inc,  NOT be_2 state
		new_ang_inc = inp_qei_s.hi_inc; // Use more reliable time estimate

		switch(phase_inc)
		{
			case 1 : // phase_inc
				// Check for accurate data
				if (1 == inp_qei_s.hi_inc)
				{ // High accuracy (evaluated over 1 angle increment)
					inp_qei_s.curr_state = QEI_HI_CLOCK; // High accuracy Clock-wise increment
				} // if (1 == inp_qei_s.hi_inc)
				else
				{ // Low accuracy (evaluated over 3 angle increments - due to intermediate errors)
					inp_qei_s.curr_state = QEI_LO_ANTI; // Low accuracy Clock-wise increment
				} // else !(1 == inp_qei_s.hi_inc)
			break; // case phase_inc = 1

			case 2 : // phase_inc
				inp_qei_s.curr_state = QEI_BIT_ERR; // 1 or 2 Bit-Errors occured
			break; // case phase_inc = 2

			case 3 : // phase_inc
				if (1 == inp_qei_s.hi_inc)
				{ // High accuracy (evaulated over 1 angle increment)
					inp_qei_s.curr_state = QEI_HI_ANTI; // High accuracy Anti-clockwise increment
				} // if (1 == inp_qei_s.hi_inc)
				else
				{ // Low accuracy (evaluated over 3 angle increments - due to intermediate errors)
					inp_qei_s.curr_state = QEI_LO_CLOCK; // Low accuracy Anti-clockwise increment
				} // else !(1 == inp_qei_s.hi_inc)
			break; // case phase_inc = 3

			default:
				// NB we can NOT detect a change of 4 or zero increments
				assert(0 == 1); // ERROR: Should not happen
			break; // default
		} // switch(phase_inc)
	} // else !((1 < inp_qei_s.hi_inc) && (inp_qei_s.lo_inc < QEI_PHASE_MASK))

{ // MB~ Dbg
	inp_qei_s.dd.ss[inp_qei_s.dd.cnt].phase_inc = phase_inc;
	inp_qei_s.dd.ss[inp_qei_s.dd.cnt].new_ang_inc = new_ang_inc; 
}

	return new_ang_inc; // Return estimate of angle increment
} // estimate_angle_increment
/*****************************************************************************/
static void update_time_differences( // Update time differences between each QEI position traversed
	QEI_DATA_TYP &inp_qei_s // Reference to structure containing QEI parameters for one motor
)
{
	ANG_INC_TYP tmp_ang = inp_qei_s.ang_inc; // Make local copy of angular increment value
	unsigned est_diff; // Estimated time-difference per angular increment


	switch(tmp_ang)
	{
		case 1 : 
			est_diff = inp_qei_s.t_dif_new; // As is (Divide by 1)
		break; // case 1

		case 2 :
			est_diff = inp_qei_s.t_dif_new >> 1; // Divide by 2
		break; // case 2

		case 3 : // QEI_PHASE_MASK 
			est_diff = (THIRD * inp_qei_s.t_dif_new + HALF_QEI_SCALE) >> SCALE_QEI_BITS; // Divide by 3
		break; // case QEI_PHASE_MASK

		default :
			// NB we can NOT detect a change of 4 or zero increments
			assert(0 == 1); // ERROR: Should not happen
		break; // case 1
	} // switch(tmp_ang)

	// If necessary, update time-diffs ...

	// Loop until angular increment is one
	while (1 < tmp_ang)
	{
		inp_qei_s.t_dif_old = inp_qei_s.t_dif_cur;
		inp_qei_s.t_dif_cur = est_diff;
		inp_qei_s.t_dif_new -= est_diff;

		tmp_ang--; // Decrement 'angular increment'
	} // while (1 < tmp_ang)

} // update_time_differences
/*****************************************************************************/
static void update_spin_state( // Update spin-state from QEI-state and 'confidence' value
	QEI_DATA_TYP &inp_qei_s // Reference to structure containing QEI parameters for one motor
)
{
	int new_confid; // new unclamped spin-direction confidence level


	// Check current state
	if (QEI_BIT_ERR != inp_qei_s.curr_state)
	{ // Valid current spin state (CLOCK or ANTI)

		// Check previous state
		if (QEI_BIT_ERR != inp_qei_s.prev_state)
		{ // Valid previous spin state (CLOCK or ANTI)
			// Check for change of spin-direction (CLOCK <--> ANTI)
			if (0 > (inp_qei_s.curr_state * inp_qei_s.confid))
			{
				// NB This could be a genuine change of direction, so do confidence again to speed-up change-over
				new_confid = inp_qei_s.confid + inp_qei_s.curr_state; // new (unclamped) confidence level
				if (MAX_CONFID >= abs(new_confid)) inp_qei_s.confid = new_confid; // Clamp confidence level
			} // if (0 > (inp_qei_s.curr_state * inp_qei_s.confid))
		} // if (QEI_BIT_ERR != inp_qei_s.prev_state)

		new_confid = inp_qei_s.confid + inp_qei_s.curr_state; // new (unclamped) confidence level
		if (MAX_CONFID >= abs(new_confid)) inp_qei_s.confid = new_confid; // Clamp confidence level

		inp_qei_s.state_errs >>= 1; // Halve error count
	} // if (QEI_BIT_ERR != inp_qei_s.curr_state)
	else
	{ // Invalid current spin state (BIT_ERROR, JUMP, or STALL)
		inp_qei_s.state_errs++; // Increment error cnt
		assert(MAX_QEI_STATE_ERR > inp_qei_s.state_errs); // Too Many QEI errors
	} // else !(QEI_BIT_ERR != inp_qei_s.curr_state)

	// Assign spin-direction based on sign of 'confidence'
	if (0 > inp_qei_s.confid)
	{ // NON Clock-wise
		inp_qei_s.ang_inc = -inp_qei_s.ang_inc; // Decrement for Anti-clockwise spin
	} // if (0 >  inp_qei_s.confid)

} // update_spin_state
/*****************************************************************************/
static void update_qei_state( // Update QEI state	by estimating angular position increment from time increments
	QEI_DATA_TYP &inp_qei_s, // Reference to structure containing QEI parameters for one motor
	unsigned cur_phases // Current set of phase values
)
{
	inp_qei_s.ang_inc = estimate_angle_increment( inp_qei_s ,cur_phases ); // Estimate angle increment and spin direction

	// Update time differences for each QEI position traversed ...
	update_time_differences( inp_qei_s ); 

	update_spin_state( inp_qei_s ); // Update spin direction

	inp_qei_s.ang_cnt += inp_qei_s.ang_inc; // Increment/Decrement angular position

{ // MB~ Dbg
	assert(DBG_SIZ > inp_qei_s.dd.cnt); // ERROR: Debug array too small

	safestrcpy( inp_qei_s.dd.ss[inp_qei_s.dd.cnt].dbg_str ,inp_qei_s.dbg_str );
	inp_qei_s.dd.ss[inp_qei_s.dd.cnt].diff_time =  inp_qei_s.diff_time;
	inp_qei_s.dd.ss[inp_qei_s.dd.cnt].lo_inc = inp_qei_s.lo_inc;
	inp_qei_s.dd.ss[inp_qei_s.dd.cnt].hi_inc = inp_qei_s.hi_inc;
	inp_qei_s.dd.ss[inp_qei_s.dd.cnt].curr_state = inp_qei_s.curr_state; 
	inp_qei_s.dd.ss[inp_qei_s.dd.cnt].confid = inp_qei_s.confid; 
	inp_qei_s.dd.ss[inp_qei_s.dd.cnt].veloc = inp_qei_s.params.veloc; 

	inp_qei_s.dd.cnt++;
}
} // update_qei_state
/*****************************************************************************/
static void update_speed( // Update speed estimate from time period. (Angular_speed in RPM) 
	QEI_DATA_TYP &inp_qei_s // Reference to structure containing QEI parameters for one motor
)
{
	unsigned abs_inc = abs(inp_qei_s.ang_inc); // Absolute angle increment
	unsigned ticks = inp_qei_s.diff_time; // ticks per QEI position (in Reference Frequency Cycles)
	int const_val;
	const_val = (TICKS_PER_MIN_PER_QEI * abs_inc) + inp_qei_s.speed_err; // Add diffusion error to constant;


	inp_qei_s.ang_speed = (const_val + (ticks >> 1)) / ticks;  // Evaluate speed
	inp_qei_s.speed_err = const_val - (inp_qei_s.ang_speed * ticks); // Evaluate new remainder value 

	// Update previous values
	inp_qei_s.prev_inc =  abs_inc;
	inp_qei_s.prev_diff = inp_qei_s.diff_time; 

}	// update_speed
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
static void update_phase_state( // Update phase state
	QEI_DATA_TYP &inp_qei_s, // Reference to structure containing QEI parameters for one motor
	unsigned cur_phases // Current set of phase values
)
{
	// Get new time stamp
	inp_qei_s.diff_time = (int)((unsigned)(inp_qei_s.curr_time - inp_qei_s.prev_time));

	// Check for sensible time
	if (THR_TICKS_PER_QEI < inp_qei_s.diff_time)
	{ // Sensible time!
		// Update previous down-scaled time differences ...

		// Check for first pass
		if (0 < inp_qei_s.t_dif_cur)
		{
			inp_qei_s.t_dif_old = inp_qei_s.t_dif_cur;
			inp_qei_s.t_dif_cur = inp_qei_s.t_dif_new;
		} // if (0 < inp_qei_s.t_dif_cur)
		else
		{ // 1st pass
			inp_qei_s.prev_inc = 1;
			inp_qei_s.prev_diff = inp_qei_s.diff_time;
			inp_qei_s.t_dif_cur = inp_qei_s.diff_time;
			inp_qei_s.t_dif_old = inp_qei_s.diff_time;
		} // if (0 < inp_qei_s.t_dif_cur)

		inp_qei_s.t_dif_new = inp_qei_s.diff_time; // Store new down-scaled time difference

		// Determine new QEI state from new pin data
		update_qei_state( inp_qei_s ,cur_phases );

		// Check for end of start-up phase
		if (START_UP_CHANGES <= inp_qei_s.pin_changes)
		{
			// Check if we have good data
			if (QEI_HI_CLOCK == abs(inp_qei_s.curr_state))
			{
				update_speed( inp_qei_s ); // Update speed value with new time difference
			} // if (QEI_BIT_ERR != inp_qei_s.curr_state)
		} // if (START_UP_CHANGES <= inp_qei_s.pin_changes)
		else
		{
			inp_qei_s.pin_changes++; // Update number of input pin changes
			inp_qei_s.ang_speed = 1;	// Default speed value
		} // if (START_UP_CHANGES <= inp_qei_s.pin_changes)
	
		inp_qei_s.prev_time = inp_qei_s.curr_time; // Store time stamp
		inp_qei_s.prev_phases = cur_phases; // Store phase value
	} // if (THR_TICKS_PER_QEI < inp_qei_s.diff_time)

	return;
} // update_phase_state
/*****************************************************************************/
static void update_origin_state( // Update origin state
	QEI_DATA_TYP &inp_qei_s, // Reference to structure containing QEI parameters for one motor
	unsigned orig_flg // Flag set when motor at origin position 
)
{
	if (orig_flg)
	{ // Reset position ( 'orig_flg' transition  0 --> 1 )

		// Update origin counter
		if (0 < inp_qei_s.ang_inc)
		{
			inp_qei_s.params.rev_cnt++; // Increment
		} // if (0 < inp_qei_s.ang_inc)
		else
		{
			inp_qei_s.params.rev_cnt--; // Decrement
		} // else !(0 < inp_qei_s.ang_inc)

		inp_qei_s.ang_cnt = 0; // Reset position value
	} // if (orig_flg)

	return;
} // update_origin_state
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
static void service_input_pins( // Service detected change on input pins
	QEI_DATA_TYP &inp_qei_s, // Reference to structure containing QEI parameters for one motor
	QEI_RAW_TYP inp_pins // Set of raw data values on input port pins
)
{
	unsigned cur_phases; // Current set of phase values
	unsigned orig_flg; // Flag set when motor at origin position 
	unsigned err_flg; // Flag set when Error condition detected


	// MB~ ToDo Insert noise filter here for each pin.
 
	// NB Due to noise corrupting bit-values, flags may change, even though phase does NOT appear to have changed
	cur_phases = inp_pins & QEI_PHASE_MASK; // Extract Phase bits from input pins
	orig_flg = inp_pins & QEI_ORIG_MASK; 		// Extract origin flag 
	err_flg = !(inp_pins & QEI_NERR_MASK); 	// NB Bit_3=0, and err_flg=1, if error detected, 

{ // For Debug
	// Check for active B-bit (Printed 1st)
	if (cur_phases & 2)
	{
		inp_qei_s.dbg_str[0] = '1';
	} // if (cur_phases & bit_mask)
	else
	{
		inp_qei_s.dbg_str[0] = '0';
	} // if (cur_phases & bit_mask)

	// Check for active A-bit (Printed 2nd)
	if (cur_phases & 1)
	{
		inp_qei_s.dbg_str[1] = '1';
	} // if (cur_phases & bit_mask)
	else
	{
		inp_qei_s.dbg_str[1] = '0';
	} // if (cur_phases & bit_mask)
} // For Debug

	// Check if phases have changed
	if (cur_phases != inp_qei_s.prev_phases) 
	{
		update_phase_state( inp_qei_s ,cur_phases ); // update phase state

		inp_qei_s.prev_phases = cur_phases; // Store phase value
	}	// if (cur_phases != inp_qei_s.prev_phases)

	// Check for change in origin state
	if (orig_flg != inp_qei_s.prev_orig)
	{
		update_origin_state( inp_qei_s ,orig_flg ); // update origin state
	
		check_for_missed_origin( inp_qei_s ); // NB May update inp_qei_s.ang_cnt & inp_qei_s.params.rev_cnt

		inp_qei_s.prev_orig = orig_flg; // Store origin flag value
	} // if (orig_flg != inp_qei_s.prev_orig)

	// Check for change in error state
	if (err_flg != inp_qei_s.params.err)
	{
		// Update estimate of error status using new error flag
		estimate_error_status( inp_qei_s ,err_flg ); // NB Updates inp_qei_s.params.err
	} // if (err_flg != inp_qei_s.params.err)

 	// The theta value returned to the client should be in the range:  0 <= ang_cnt < 360 degrees
	inp_qei_s.params.theta = (inp_qei_s.ang_cnt & QEI_REV_MASK); // force into range [0..QEI_REV_MASK]

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
	int meas_veloc; // Angular_velocity of motor in RPM


	// Evaluate (signed) angular_velocity of motor in RPM
	if (0 < inp_qei_s.ang_inc)
	{
		meas_veloc = inp_qei_s.ang_speed; // +ve
	} // if (0 < inp_qei_s.ang_inc)
	else
	{
		meas_veloc = -inp_qei_s.ang_speed; // -ve
	} // else !(0 < inp_qei_s.ang_inc)

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
	printstr(" AI="); printint( dbg_smp_s.new_ang_inc ); 
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
/*****************************************************************************/
#pragma unsafe arrays
void foc_qei_do_multiple( // Get QEI data from motor and send to client
	streaming chanend c_qei[], // Array of data channel to client (carries processed QEI data)
	buffered port:4 in pb4_QEI[] // Array of buffered 4-bit input ports (carries raw QEI motor data)
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
			case (int motor_id=0; motor_id<NUMBER_OF_MOTORS; motor_id++) pb4_QEI[motor_id] when pinsneq(inp_pins[motor_id]) :> inp_pins[motor_id] :
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

	print_all_dbg( all_qei_s[0] ); // MB~ Dbg

	acquire_lock(); 
	printstrln("");
	printstrln("                                             QEI Server Ends");
	release_lock();

	return;
} // foc_qei_do_multiple
/*****************************************************************************/
