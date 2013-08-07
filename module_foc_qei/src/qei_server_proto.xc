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


	inp_qei_s.inv_phase = clkwise; // Assign table converting QEI Phase value to circular index

	inp_pins = 0xFF; // Set buffer for reading input pins to impossible value

	inp_qei_s.params.theta = 0; // Reset angular position returned to client
	inp_qei_s.params.rev_cnt = 0; // Reset revolution counter  returned to client
	inp_qei_s.params.veloc = 0; // Clear velocity returned to client
	inp_qei_s.params.err = QEI_ERR_OFF; // Clear error status flag returned to client

	inp_qei_s.diff_time = 0; // NB Initially this is used to count input-pin changes
	inp_qei_s.id = inp_id; // Clear Previous phase values
	inp_qei_s.ang_cnt = 0; // Reset counter indicating angular position of motor (from origin)
	inp_qei_s.ang_inc = 0; // Reset angular position increment
	inp_qei_s.prev_ang = 0; // MB~
	inp_qei_s.spin_sign = 0; // Clear Sign of spin direction
	inp_qei_s.prev_state = QEI_STALL; // Initialise previous QEI state
	inp_qei_s.state_errs = 0; // Initialise counter for invalid QEI state transitions
	inp_qei_s.status_errs = 0; // Initialise counter for QEI status errors
	inp_qei_s.confid = 0; // Initialise spin-direction confidence value (zero: No confidence)

	inp_qei_s.interval = 0; // Clear expected interval between QEI phase changes
	inp_qei_s.prev_time = 0;
	inp_qei_s.prev_orig = 0;
	inp_qei_s.prev_phases = 0;

	inp_qei_s.filt_val = 0; // filtered value
	inp_qei_s.coef_err = 0; // Coefficient diffusion error
	inp_qei_s.scale_err = 0; // Scaling diffusion error 
	inp_qei_s.speed_err = 0; // Speed diffusion error 

	inp_qei_s.t_dif_old = 0; // Initialise to unassigned time difference
	inp_qei_s.t_dif_cur = 0; // Initialise to unassigned time difference
	inp_qei_s.prev_squ = 0; // Initialise to unassigned time difference

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

	inp_qei_s.scale_bits = BITS_IN_BYTE;
	inp_qei_s.half_scale = (unsigned)(1 << (inp_qei_s.scale_bits - 1)); // Used in  rounding

	inp_qei_s.max_thr = ((U64_T)1 << (U64_T)(INT32_BITS - 1)); // 2^31

	return;
} // init_qei_data
/*****************************************************************************/
static ANG_INC_TYP estimate_angularincrement_new( // Estimate angular position increment from time increments
	QEI_DATA_TYP &inp_qei_s, // Reference to structure containing QEI parameters for one motor
	unsigned cur_phases // Current set of phase values
) // Returns output angular increment value
/* When used in the test-harness with added noise, this algorithm works up until the change from 'Accelerating'
 * to 'Fast steady-state'. This is a step-change in acceleration, 
 * which would probably never happen with the data from a real-motor.
 */
{
//MB~	int phase_index = inp_qei_s.inv_phase.vals[cur_phases]; // Converts [BA] phase value into circular index [0, 1, 2, 3]
	ANG_INC_TYP out_a_inc = 0; // Clear Output angular increment value
	unsigned new_est = 0; // New time difference estimate
	unsigned numr_32; // 32-bit Numerator for integer division
	int divr_32; // 32-bit Divisor for integer division
	U64_T squ_diff; // Square of current time difference
	U64_T numr_64;// Current 64-bit Numerator
	S64_T divr_64; // Current 64-bit Divisor
	S64_T tst_64;// test value for nearest estimate comparison 
	U64_T tmp_numr; // Temporary numerator
	S64_T tmp_divr; // Temporary divisor


/* This section estimates the next time-difference as numr/divr, assuming 2nd order angular motion. 
 * (i.e. Constant angular acceleration. Te = To^2.Tc/(To^2 + To.Tc - Tc^2)
 */ 
	squ_diff = (U64_T)inp_qei_s.t_dif_cur * (U64_T)inp_qei_s.t_dif_cur; // Square of time-difference estimate

	divr_64 = (U64_T)inp_qei_s.t_dif_old * (U64_T)inp_qei_s.t_dif_cur + inp_qei_s.prev_squ - squ_diff;
	assert(0 < divr_64); // ERROR: this should never happen
	numr_64 = inp_qei_s.prev_squ * (U64_T)inp_qei_s.t_dif_cur; // Calculate 64-bit numerator
	assert(numr_64 > divr_64); // ERROR: this should never happen

/* The naive test would be (numr/divr < new_diff), but we eliminate a division
   by using (numr < tst_val), where tst_val = (divr * new_diff)
*/
	tst_64 = (S64_T)inp_qei_s.t_dif_new * (S64_T)divr_64; // Compute test-value
/*
acquire_lock(); 
printstrln("");
printstr("TD[OCN]="); printuint(inp_qei_s.t_dif_old); printchar(':'); printuint(inp_qei_s.t_dif_cur); printchar(':'); printuintln(inp_qei_s.t_dif_new);
printstr("T="); printullong(tst_64); 
printstr(" PS="); printullong(inp_qei_s.prev_squ); 
printstr(" SD="); printullongln(squ_diff);
printstr("N="); printullong(numr_64); printstr(" D="); printllongln(divr_64);
release_lock(); // MB~
*/
	// Loop until (N x Estimated time-diff) is larger than New time-diff
	while (numr_64 < tst_64)
	{ // Not Larger. Subtract estimate again

		// Check if etimate has been evaluated
		if (0 == new_est)
		{
			// Calculate estimated time-diff from 64-bit arguments. Using 32-bit division to save cycles ...
	
			tmp_numr = numr_64;
			tmp_divr = divr_64;

			/* NB numr/divr can be greated than 2^16, 
       * so we have to reduce precision by increments, to prevent tmp_divr becoming zero.
			 */
			while (inp_qei_s.max_thr <= tmp_numr)
			{ // Down-scale (by 8-bits)
				tmp_divr = (tmp_divr + (S64_T)inp_qei_s.half_scale) >> (S64_T)inp_qei_s.scale_bits;
				tmp_numr = (tmp_numr + (U64_T)inp_qei_s.half_scale) >> (U64_T)inp_qei_s.scale_bits;
			} // while (inp_qei_s.max_thr <= tmp_numr)
	
			divr_32 = (int)tmp_divr;
			assert(0 < divr_32);  // ERROR: this should never happen
			numr_32 = (unsigned)tmp_numr;
			assert(numr_32 > divr_32); // ERROR: this should never happen

			new_est = (numr_32 + (divr_32 >> 1)) / divr_32; // Calculate new estimate
/*
acquire_lock(); 
printstr("NE="); printuint(new_est); 
printstr(" N32="); printuint(numr_32); printstr(" D32="); printintln(divr_32);
release_lock(); // MB~
*/
		} // if (0 == new_est)

		// We definitely have a angular increment, so update previous angle and time-diffs
		out_a_inc++; // increment change in angular position

		inp_qei_s.t_dif_old = inp_qei_s.t_dif_cur;
		inp_qei_s.t_dif_cur = new_est;

		inp_qei_s.t_dif_new -= (int)new_est; // Subtract estimate from measured time-difference

		inp_qei_s.prev_squ = squ_diff; // Store previous square of time-differences

		squ_diff = (U64_T)inp_qei_s.t_dif_cur * (U64_T)inp_qei_s.t_dif_cur;

		tst_64 = (S64_T)inp_qei_s.t_dif_new * (S64_T)divr_64; // re-compute test-value
/*
acquire_lock(); 
printstr("td[ODN]="); printint(inp_qei_s.t_dif_old); printstr(":"); printint(inp_qei_s.t_dif_cur); printstr(":"); printint(inp_qei_s.t_dif_new); printstr("  NE="); printintln(new_est);
printstr("T="); printullong(tst_64);
printstr(" PS="); printullong(inp_qei_s.prev_squ); 
printstr(" SD="); printullongln(squ_diff); 
release_lock(); // MB~
*/
	} // while (numr_64 < tst_64))

	/* We now have 2 time-diff estimates either side of the measured value. So choose the closest.
	 * Again we 'go through hoops' to eliminate a division.
	 * The naive test is:  new_diff < (numr/divr - new_diff)
	 * Eliminating the division, the test becomes:  2.new_diff.divr < numr
	 * or 2.tst_val < numr 
	 */
	tst_64 <<= 1;

	// If we have 2 estimates choose the closest	
	if ((tst_64 < numr_64) && (0 < out_a_inc))
	{ // previous estimate was closest. So unwind
		inp_qei_s.t_dif_new += inp_qei_s.t_dif_cur;
		inp_qei_s.t_dif_cur = inp_qei_s.t_dif_old;

/*
acquire_lock();
printstrln(""); printstr("T="); printullong(tst_64);
printstr(" PS="); printullongln(inp_qei_s.prev_squ); 
printstr("-Exit "); printuint(inp_qei_s.t_dif_old); printchar(':'); printuint(inp_qei_s.t_dif_cur); printchar(':'); printuint(inp_qei_s.t_dif_new);
printstrln(""); release_lock(); // MB~
*/

	} // if (tst_64 < numr_64)
	else
	{ // new estimate was closest.
		out_a_inc++; // increment change in angular position
		inp_qei_s.prev_squ = squ_diff; // Store square of time-diff for next iteration

/*
acquire_lock(); 
printstrln(""); printstr("T="); printullong(tst_64);
printstr(" PS="); printullongln(inp_qei_s.prev_squ); 
printstr(" Exit "); printuint(inp_qei_s.t_dif_old); printchar(':'); printuint(inp_qei_s.t_dif_cur); printchar(':'); printuint(inp_qei_s.t_dif_new);
printstrln(""); release_lock(); // MB~
*/

	} // else !(tst_64 < numr_64)

	return out_a_inc; // Return output increment value
} // estimate_angularincrement_new
/*****************************************************************************/
static ANG_INC_TYP estimate_angular_increment( // Estimate angular position increment from QEI states
	QEI_DATA_TYP &inp_qei_s, // Reference to structure containing QEI parameters for one motor
	unsigned cur_phases // Current set of phase values
) // Returns output angular increment value (one of [-2, -1, 0, 1, 2])
{
/* get_spin is a table for converting pins values to spin values
 *	[BA} order is 00 -> 10 -> 11 -> 01  Clockwise direction
 *	[BA] order is 00 -> 01 -> 11 -> 10  Anti-Clockwise direction
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
static const ANG_INC_TYP get_spin_state[NUM_QEI_PHASES][NUM_QEI_PHASES] = {
		{ QEI_STALL,	QEI_ANTI,		QEI_CLOCK,	QEI_JUMP	},
		{ QEI_CLOCK,	QEI_STALL,	QEI_JUMP,		QEI_ANTI	},
		{ QEI_ANTI,		QEI_JUMP,		QEI_STALL,	QEI_CLOCK	},
		{ QEI_JUMP,		QEI_CLOCK,	QEI_ANTI,		QEI_STALL	}
};

	QEI_ENUM_TYP curr_state = get_spin_state[cur_phases][inp_qei_s.prev_phases]; // Estimated new spin-state fom phase info.
	QEI_ENUM_TYP prev_state = inp_qei_s.prev_state; // local copy of previous QEI-state
	ANG_INC_TYP out_ang_inc; // Output angular increment value
	int new_confid; // new unclamped spin-direction confidence level


	// Update spin Finite-State-Machine ...

	// Check bit_0 of previous state
	if (prev_state & 0x1)
	{ // Valid previous spin state (CLOCK or ANTI)

		// Check bit_0 of current state
		if (curr_state & 0x1)
		{ // Valid current spin state (CLOCK or ANTI)
			// Check for change of spin-direction (CLOCK <--> ANTI)
			if (0 > (curr_state * inp_qei_s.confid))
			{
				assert(MAX_QEI_STATE_ERR > inp_qei_s.state_errs); // Too Many QEI errors

				inp_qei_s.state_errs++; // Increment invalid state cnt

				// NB This could be a genuine change of direction, so do confidence again to speed-up change-over
				new_confid = inp_qei_s.confid + curr_state; // new (unclamped) confidence level
				if (MAX_CONFID >= abs(new_confid)) inp_qei_s.confid = new_confid; // Clamp confidence level
			} // if (0 > (curr_state * inp_qei_s.confid))
			else
			{ // Consistent spin-direction
				inp_qei_s.state_errs = 0; // Reset error count
			} // else !(0 > (curr_state * inp_qei_s.confid))

			new_confid = inp_qei_s.confid + curr_state; // new (unclamped) confidence level
			if (MAX_CONFID >= abs(new_confid)) inp_qei_s.confid = new_confid; // Clamp confidence level

			out_ang_inc = 1; // Single position increment/decrement
		} // if (curr_state & 0x1)
		else
		{ // Invalid current spin state (STALL or JUMP)
			// Check for missed time slot
			out_ang_inc = 2; // Double position increment/decrement as phase jump detected
		} // else !(curr_state & 0x1)
	} // if (prev_state & 0x1)
	else
	{ // Invalid previous spin state

		// Check bit_0 of current state
		if (curr_state & 0x1)
		{ // Valid current spin state (CLOCK or ANTI)
			new_confid = inp_qei_s.confid + curr_state; // new (unclamped) confidence level
			if (MAX_CONFID >= abs(new_confid)) inp_qei_s.confid = new_confid; // Clamp confidence level

			out_ang_inc = 1; // Single position increment/decrement
		} // if (curr_state & 0x1)
		else
		{ // Invalid current spin state (STALL or JUMP)
			assert(MAX_QEI_STATE_ERR > inp_qei_s.state_errs); // Too Many QEI errors

			inp_qei_s.state_errs++; // Increment invalid state cnt

			// Consistent jumps so reduce confidence
			if (0 < inp_qei_s.confid)
			{ // Probably Clock-wise
				inp_qei_s.confid--;
			} // if (0 < inp_qei_s.confid)
			else
			{
				if (0 > inp_qei_s.confid) inp_qei_s.confid++; // Probably Anti-clockwise
			} // else !(0 < inp_qei_s.confid)

			// Check for missed time slot
			out_ang_inc = 2; // Double position increment/decrement as phase jump detected
		} // else !(curr_state & 0x1)
	} // else !(prev_state & 0x1)

	// Check spin-direction confidence
	if (1 > inp_qei_s.confid)
	{ // NON Clock-wise
		if (0 > inp_qei_s.confid)
		{ // Anti-clockwise
			out_ang_inc -= out_ang_inc ; // Decrement for Anti-clockwise spin
		} // if (0 > inp_qei_s.confid)
		else
		{ // Indeterminate spin direction
			out_ang_inc = 0; // Clear position increment/decrement as no confidence
		} // else !(0 > inp_qei_s.confid)
	} // if (1 >  inp_qei_s.confid)

	inp_qei_s.prev_state = curr_state; // Store old QEI state value

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
	// Update estimate of error status using new error flag
	estimate_error_status( inp_qei_s ,err_flg ); // NB Updates inp_qei_s.params.err

	// Update estimate of angular increment value using new spin state
	inp_qei_s.ang_inc = estimate_angular_increment( inp_qei_s ,cur_phases ); 

{ //MB~
	ANG_INC_TYP tmp = estimate_angularincrement_new( inp_qei_s ,cur_phases ); // Output angular increment value
	acquire_lock(); printstr("N="); printint(tmp); printstr(" O="); printint(inp_qei_s.ang_inc); printstr(" ND="); printintln(inp_qei_s.t_dif_new); release_lock(); // MB~
} //MB~
	inp_qei_s.ang_cnt += inp_qei_s.ang_inc; // Increment/Decrement angular position

	// Update estimate of spin direction based on spin-direction confidence
	if (inp_qei_s.confid < 0)
	{
		inp_qei_s.spin_sign = -1; // -ve spin direction
	} // if (inp_qei_s.ang_inc < 0)
	else
	{
		inp_qei_s.spin_sign = 1;  // +ve spin direction
	} // else !(inp_qei_s.ang_inc < 0)

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
	int const_val = (TICKS_PER_MIN_PER_QEI * inp_qei_s.ang_inc) + inp_qei_s.speed_err; // Add diffusion error to constant;


	inp_qei_s.ang_speed = (const_val + (ticks >> 1)) / ticks;  // Evaluate speed
	inp_qei_s.speed_err = const_val - (inp_qei_s.ang_speed * ticks); // Evaluate new remainder value 

	inp_qei_s.interval = ticks; // ticks per QEI position (in Reference Frequency Cycles)

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
		// Get new time stamp
		test_diff = (int)(inp_qei_s.curr_time - inp_qei_s.prev_time);

		// Check for sensible time
		if (THR_TICKS_PER_QEI < test_diff)
		{ // Sensible time!
			orig_flg = inp_pins & QEI_ORIG_MASK; 		// Extract origin flag 
			err_flg = !(inp_pins & QEI_NERR_MASK); 	// NB Bit_3=0, and err_flg=1, if error detected, 

			// Update previous down-scaled time differences ...

			// Check for first pass
			if (0 < inp_qei_s.t_dif_cur)
			{
				inp_qei_s.t_dif_old = inp_qei_s.t_dif_cur;
				inp_qei_s.t_dif_cur = inp_qei_s.t_dif_new;
			} // if (0 < inp_qei_s.t_dif_cur)
			else
			{ // 1st pass
				inp_qei_s.t_dif_cur = test_diff;
				inp_qei_s.t_dif_old = test_diff;
				inp_qei_s.prev_squ = (U64_T)test_diff * (U64_T)test_diff;
			} // if (0 < inp_qei_s.t_dif_cur)

			inp_qei_s.t_dif_new = test_diff; // Store new down-scaled time difference
acquire_lock(); printstr("TD="); printintln(test_diff); release_lock(); // MB~
	
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
static void service_client_request( // Send processed QEI data to client
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

// if (inp_qei_s.id) xscope_probe_data( 2 ,inp_qei_s.params.rev_cnt );

	c_qei <: inp_qei_s.params; // Transmit QEI parameters to Client

	return;
} // service_client_request
/*****************************************************************************/
#pragma unsafe arrays
void foc_qei_do_multiple( // Get QEI data from motor and send to client
	streaming chanend c_qei[], // Array of data channel to client (carries processed QEI data)
	port in pQEI[] 						 // Array of input port (carries raw QEI motor data)
)
#define STAT_BITS 12
#define NUM_STATS (1 << STAT_BITS)
#define HALF_STATS (NUM_STATS >> 1)
{
	QEI_DATA_TYP all_qei_s[NUMBER_OF_MOTORS]; // Array of structures containing QEI parameters for all motor
	QEI_RAW_TYP inp_pins[NUMBER_OF_MOTORS]; // Set of raw data values on input port pins
	timer chronometer; // H/W timer


	for (int motor_id=0; motor_id<NUMBER_OF_MOTORS; ++motor_id) 
	{
		init_qei_data( all_qei_s[motor_id] ,inp_pins[motor_id] ,motor_id ); // Initialise QEI data for current motor
	} // for motor_id

	while (1) {
#pragma xta endpoint "qei_main_loop"
#pragma ordered // If multiple cases fire at same time, service top-most first
		select {
			// Service any change on input port pins
			case (int motor_id=0; motor_id<NUMBER_OF_MOTORS; motor_id++) pQEI[motor_id] when pinsneq(inp_pins[motor_id]) :> inp_pins[motor_id] :
			{
				chronometer :> all_qei_s[motor_id].curr_time;	// Get new time stamp as soon as possible
				service_input_pins( all_qei_s[motor_id] ,inp_pins[motor_id] );

#if (USE_XSCOPE)
		xscope_int( 0 ,inp_pins[motor_id] );
		xscope_int( 1 ,all_qei_s[motor_id].prev_state );
		xscope_int( 2 ,all_qei_s[motor_id].confid );
		xscope_int( 3 ,all_qei_s[motor_id].params.rev_cnt );
		xscope_int( 4 ,all_qei_s[motor_id].params.theta );
		xscope_int( 5 ,(all_qei_s[motor_id].params.veloc >> 7) );
		xscope_int( 6 ,all_qei_s[motor_id].params.err );
#endif // (USE_XSCOPE)
			} // case
			break;

			// Service any client request for data
			case (int motor_id=0; motor_id<NUMBER_OF_MOTORS; motor_id++) c_qei[motor_id] :> int :
			{
				service_client_request( all_qei_s[motor_id] ,c_qei[motor_id] );
			} // case
			break;
		} // select
	}	// while (1)

	return;
} // foc_qei_do_multiple
/*****************************************************************************/
