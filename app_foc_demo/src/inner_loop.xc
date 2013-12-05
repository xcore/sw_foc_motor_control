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

#include "inner_loop.h"

/*****************************************************************************/
static void init_error_data( // Initialise Error-handling data
	ERR_DATA_TYP &err_data_s // Reference to structure containing data for error-handling
)
{
	int err_cnt; // phase counter


	err_data_s.err_flgs = 0; 	// Clear fault detection flags

	// Initialise error strings
	for (err_cnt=0; err_cnt<NUM_ERR_TYPS; err_cnt++)
	{
		err_data_s.line[err_cnt] = -1; 	// Initialised to unassigned line number
		safestrcpy( err_data_s.err_strs[err_cnt].str ,"No Message! Please add in function init_motor()" );
	} // for err_cnt

	safestrcpy( err_data_s.err_strs[OVERCURRENT_ERR].str ,"Over-Current Detected" );
	safestrcpy( err_data_s.err_strs[UNDERVOLTAGE_ERR].str ,"Under-Voltage Detected" );
	safestrcpy( err_data_s.err_strs[STALLED_ERR].str ,"Motor Stalled Persistently" );
	safestrcpy( err_data_s.err_strs[DIRECTION_ERR].str ,"Motor Spinning In Wrong Direction!" );
	safestrcpy( err_data_s.err_strs[SPEED_ERR].str ,"Motor Exceeded Maximim Speed" );

} // init_error_data 
/*****************************************************************************/
static unsigned calc_bit_resolution( // Calculate No of bits required to represent (unsigned) input value
	unsigned inp_val // Input value
) // Returns No. of resolution bits
{
	unsigned out_bits = 0; // No. of resolution bits to return


	while (inp_val > 0)
	{
		inp_val >>= 1;
		out_bits++;
	} // while (inp_val > 0)

	return out_bits; // Return No. of resolution bits
} // calc_bit_resolution
/*****************************************************************************/
static void init_gamma_data( // Initialise Gamma data for leading angle
	MOTOR_DATA_TYP &motor_s // Reference to structure containing motor data
)
{
	int speed_rng = (MAX_SPEC_RPM - MIN_STALL_RPM); // Speed range
	int gamma_rng = (MAX_GAMMA - MIN_GAMMA); // Gamma range
	int tmp_val; // Temporary manipulation variable;


	tmp_val = (gamma_rng << GAMMA_BITS); // Up-scale Gamma range

	// Calculate up-scaled Gamma gradient
	motor_s.gamma_grad = (tmp_val + (speed_rng >> 1)) / speed_rng; 

	tmp_val = (MIN_GAMMA << GAMMA_BITS); // Up-scale Minimum Gamma

	// Calculate up-scaled Gamma offset
//MB~ 	motor_s.gamma_off = tmp_val - MIN_STALL_RPM * motor_s.gamma_grad;
	motor_s.gamma_off = 24; //MB~ 24

} // init_gamma_data
/*****************************************************************************/
static void init_blend_data( // Initialise blending data for 'TRANSIT state'
	MOTOR_DATA_TYP &motor_s // Reference to structure containing motor data
)
/* Calculate time in TRANSIT state based on Requested open-loop Vq ...
 * Using at least one electrical cycle per  1000 Vq units
 * Then compute blending function data, so bit-shift division can be used.
 */
{
	unsigned diff_theta; // blending period (measured as QEI angle positions)
	unsigned numer_bits; // No of bits used to represent numerator of blending function
	int Vd_diff; // Vd difference
	int Vq_diff; // Vq difference
	int max_diff; // Max Voltage difference


	// Evaluate state termination conditions
	motor_s.search_theta = QEI_PER_PAIR; // theta value at end of 'SEARCH state'

	// This section uses at least one electrical cycle per 1024 Vq values ..

	motor_s.vect_data[D_ROTA].diff_V = motor_s.vect_data[D_ROTA].end_open_V - motor_s.vect_data[D_ROTA].start_open_V; // Vd difference
	motor_s.vect_data[Q_ROTA].diff_V = motor_s.vect_data[Q_ROTA].end_open_V - motor_s.vect_data[Q_ROTA].start_open_V; // Vq difference

	Vd_diff = abs(motor_s.vect_data[D_ROTA].diff_V); // Vd difference
	Vq_diff = abs(motor_s.vect_data[Q_ROTA].diff_V); // Vq difference

	max_diff = Vq_diff; // Preset to Vq difference

	// Check if Vd difference is larger
	if (max_diff < Vd_diff) max_diff = Vd_diff;

	motor_s.trans_cycles = (max_diff + VOLT_DIFF_MASK) >> VOLT_DIFF_BITS;
	if (motor_s.trans_cycles < 1) motor_s.trans_cycles = 1; // Ensure at least one cycle

	diff_theta = QEI_PER_PAIR * motor_s.trans_cycles; // No of QEI positions (theta)
	motor_s.trans_theta = motor_s.search_theta + diff_theta; // theta at end of 'TRANSIT state'

	// Calculate upscale value based on No. of electrical cycles
	motor_s.blend_up = (BLEND_UP + (motor_s.trans_cycles >> 1)) / motor_s.trans_cycles;

	// Check bit precision ...

	numer_bits = calc_bit_resolution( diff_theta ); // Preset with period bits

	numer_bits = (2 * numer_bits) + 1; // Update to account for maximum variable size

	numer_bits += calc_bit_resolution( motor_s.blend_up ); // Add in bits for up-scale factor

	assert(32 > numer_bits); // ERROR: Failed numeric overflow test

	// Preset blend divisor-bits with bit resolution of QEI_PER_PAIR ...
	motor_s.blend_bits = calc_bit_resolution(QEI_PER_PAIR - 1);

	motor_s.blend_bits += BLEND_BITS; // Add bits for up-scaling factor

	motor_s.half_blend = (1 << (motor_s.blend_bits -1)); // Used in rounding
} // init_blend_data
/*****************************************************************************/
static void init_pid_data( // Initialise PID data
	MOTOR_DATA_TYP &motor_s // reference to structure containing motor data
)
{
	int pid_cnt; // PID counter


	/* This section initialise the PID constants [ K_p ,K_i ,Kd ]
	 * NB The K values are used as fixed point integers e.g. K_p = 580000 / 2^PID_CONST_RES
	 * Default PID_CONST_RES = 15
	 * The Error Sum is held at a reduced precision:  Sum_Err /  2^PID_CONST_RES
	 * Therefore in the init_pid_consts interface K_i is upscaled by 2^PID_CONST_RES to compensate
	 */
//MB~			init_pid_consts( motor_s.pid_consts[TRANSFORM][ID_PID] ,0 ,0 ,0 );
//MB~			init_pid_consts( motor_s.pid_consts[TRANSFORM][IQ_PID]		,580000 ,384 ,0 );
//MB~			init_pid_consts( motor_s.pid_consts[TRANSFORM][IQ_PID]		,19000 ,4000	,0 );
//MB~			init_pid_consts( motor_s.pid_consts[TRANSFORM][ID_PID]		,(1 << (PID_CONST_RES - 1)) ,4000 ,0 ); // NB Kp assumes target Id=0
//MB~			init_pid_consts( motor_s.pid_consts[TRANSFORM][IQ_PID]		,19000 ,1000	,0 );
//MB~			init_pid_consts( motor_s.pid_consts[TRANSFORM][IQ_PID] ,472000 ,100	,0 );
//MB~			init_pid_consts( motor_s.pid_consts[TRANSFORM][IQ_PID] ,193000 ,100	,0 );
//MB~ 		init_pid_consts( motor_s.pid_consts[TRANSFORM][ID_PID] ,(1 << (PID_CONST_RES - 1) ,100 ,0 ); // NB Kp assumes target Id=0

//MB~	init_pid_consts( motor_s.pid_consts[TRANSFORM][ID_PID] ,1600000 ,4000 ,0 ); // NB Kp assumes target Id=0
//MB~	init_pid_consts( motor_s.pid_consts[TRANSFORM][IQ_PID] ,1100000 ,2000	,0 );
//MB~	init_pid_consts( motor_s.pid_consts[TRANSFORM][SPEED_PID]	,21900 ,6 ,0 ); // NB Tuned to give correct Velocity -> Iq conversion

	init_pid_consts( motor_s.pid_consts[TRANSFORM][ID_PID] ,1600000 ,1000 ,0 ); // NB Kp assumes target Id=0
	init_pid_consts( motor_s.pid_consts[TRANSFORM][IQ_PID] ,1100000 ,2000	,0 );
	init_pid_consts( motor_s.pid_consts[TRANSFORM][SPEED_PID]	,21900 ,6 ,0 ); // NB Tuned to give correct Velocity -> Iq conversion

// MB~ EXTREMA and VELOCITY need a re-tune
	init_pid_consts( motor_s.pid_consts[EXTREMA][ID_PID] ,0 ,0 ,0 );
	init_pid_consts( motor_s.pid_consts[EXTREMA][IQ_PID]		,400000 ,256 ,0 );
	init_pid_consts( motor_s.pid_consts[EXTREMA][SPEED_PID]	,20000	,3 ,0 );

	init_pid_consts( motor_s.pid_consts[VELOCITY][ID_PID] ,0 ,0 ,0 );
	init_pid_consts( motor_s.pid_consts[VELOCITY][IQ_PID]		,12000 ,8 ,0 );
	init_pid_consts( motor_s.pid_consts[VELOCITY][SPEED_PID]	,11200 ,8 ,0 );

	// Initialise PID regulators
	for (pid_cnt = 0; pid_cnt < NUM_PIDS; pid_cnt++)
	{ 
		initialise_pid( motor_s.pid_regs[pid_cnt] );
	} // for pid_cnt
	
	motor_s.pid_Id = 0;	// Output from radial current PID
	motor_s.pid_Iq = 0;	// Output from tangential current PID
	motor_s.pid_veloc = 0;	// Output from velocity PID

} // init_pid_data
/*****************************************************************************/
static void init_rotation_component( // Initialise data for one component of rotation vector
	MOTOR_DATA_TYP &motor_s, // Reference to structure containing motor data
	ROTA_VECT_ENUM comp_id, // Indentifier for component to initialise
	int start_V_openloop, // Component voltage at start of open-loop state
	int end_V_openloop, //  Component voltage at end of open-loop state
	int req_V_closedloop // Initial component voltage for closed-loop state
)
{
	motor_s.vect_data[comp_id].start_open_V = start_V_openloop;  // Voltage during open-loop mode (start-up) 
	motor_s.vect_data[comp_id].trans_V = start_V_openloop;  // Starting voltage for TRANSIT state
	motor_s.vect_data[comp_id].set_V = start_V_openloop; // Initialise driving voltage for closed-loop mode 
	motor_s.vect_data[comp_id].end_open_V = end_V_openloop; // Initialise requested voltage for open-loop mode 
	motor_s.vect_data[comp_id].req_closed_V = req_V_closedloop; // Initialise requested voltage for closed-loop mode 
	motor_s.vect_data[comp_id].prev_V = start_V_openloop;  // Preset previously used demand voltage value
	motor_s.vect_data[comp_id].diff_V = 0; // Difference between Start and Requested Voltage
	motor_s.vect_data[comp_id].rem_V = 0; // Voltage remainder, used in error diffusion

	motor_s.vect_data[comp_id].inp_I = 0; // Initialise input estimated current
	motor_s.vect_data[comp_id].est_I = 0; // Initialise (possibly filtered) output estimated current
	motor_s.vect_data[comp_id].prev_I = 0; // Initialise previous estimated current
	motor_s.vect_data[comp_id].rem_I = 0; // Initialise remainder used in error diffusion
} // init_rotation_component
/*****************************************************************************/
static void init_motor( // initialise data structure for one motor
	MOTOR_DATA_TYP &motor_s, // reference to structure containing motor data
	unsigned motor_id // Unique Motor identifier e.g. 0 or 1
)
{
	int phase_cnt; // phase counter
	int tmp_val; // temporary manipulation value
	int start_D; // Vd value at start of open-loop state
	int start_Q; // Vq value at start of open-loop state
	int end_D; // Vd value at end of open-loop state
	int end_Q; // Vq value at end of open-loop state
	int req_D; // Initial requested closed-loop Vd value
	int req_Q; // Initial requested closed-loop Vq value
	int start_gamma; // Gamma value at start
	int end_gamma; // Gamma value end of open-loop state
	int req_gamma; // Initial requested Gamma value for closed-loop state


	init_error_data( motor_s.err_data );

	init_gamma_data( motor_s );

	init_pid_data( motor_s );

	for (phase_cnt = 0; phase_cnt < NUM_PWM_PHASES; phase_cnt++)
	{ 
		motor_s.adc_params.vals[phase_cnt] = -1;
	} // for phase_cnt

	motor_s.id = motor_id; // Unique Motor identifier e.g. 0 or 1
	motor_s.iters = 0;
	motor_s.cnts[ALIGN] = 0;
	motor_s.cnts[SEARCH] = 0;
	motor_s.state = ALIGN;
	motor_s.hall_params.hall_val = HALL_NERR_MASK; // Initialise to 'No Errors'
	motor_s.prev_hall = (!HALL_NERR_MASK); // Arbitary value different to above

	motor_s.Iq_alg = TRANSFORM; // [TRANSFORM VELOCITY EXTREMA] Assign algorithm used to estimate coil current Iq (and Id)
	motor_s.first_pid = 1; // Set flag until first set of PID's completed
	motor_s.hall_offset = 0;	// Phase difference between the Hall sensor origin and PWM theta origin
	motor_s.qei_offset = 0;	// Phase difference between the QEI origin and PWM theta origin
	motor_s.hall_found = 0;	// Set flag to Hall origin NOT found
	motor_s.qei_found = 0;	// Set flag to QEI origin NOT found
	motor_s.xscope = 1; 	// Switch on xscope print flag
	motor_s.prev_pwm_time = 0; 	// previous open-loop time stamp
	motor_s.coef_err = 0; // Clear Extrema Coef. diffusion error
	motor_s.scale_err = 0; // Clear Extrema Scaling diffusion error
	motor_s.Iq_err = 0; // Clear Error diffusion value for measured Iq
	motor_s.prev_Id = 0;	// Initial target 'radial' current value

	motor_s.tot_ang = 0;	// Total angle traversed (NB accounts for multiple revolutions)
	motor_s.set_theta = 0; // PWM theta value
	motor_s.open_theta = 0; // Open-loop theta value
	motor_s.foc_theta = 0; // FOC theta value

	motor_s.trans_cycles = 1; // Default number of electrical cycles spent in TRANSIT state
	motor_s.trans_cnt = 0; // Counts trans_theta updates

	// Check consistency of pre-defined QEI values
	tmp_val = (1 << QEI_RES_BITS); // Build No. of QEI points from resolution bits
	assert( QEI_PER_REV == tmp_val );

	motor_s.half_qei = (QEI_PER_REV >> 1); // Half No. of QEI points per revolution
	motor_s.pwm_period = OPEN_LOOP_PERIOD; // Time between updates to PWM theta

	motor_s.filt_adc = START_VOLT_OPENLOOP; // Preset filtered value to something sensible

	motor_s.req_veloc = REQ_VELOCITY;
	motor_s.half_veloc = (motor_s.req_veloc >> 1);
	motor_s.prev_veloc = 0; // Previous measured velocity

	// NB Display will require following variables, before we have measured them! ...
	motor_s.qei_params.veloc = motor_s.req_veloc;
	motor_s.meas_speed = abs(motor_s.req_veloc);


	// Initialise angle variables dependant on spin direction
	if (0 > motor_s.req_veloc)
	{ // Negative spin direction
		motor_s.gamma_off = -motor_s.gamma_off;
		motor_s.gamma_grad = -motor_s.gamma_grad;

		start_gamma = -START_GAMMA_OPENLOOP; 
		end_gamma = -END_GAMMA_OPENLOOP; 
		req_gamma = -REQ_GAMMA_CLOSEDLOOP; 

		motor_s.end_hall = NEGA_LAST_HALL_STATE; // Choose last Hall state of 6-state cycle
	} // if (0 > motor_s.req_veloc)
	else
	{ // Positive spin direction
		// Use Park Transform to convert Absolute-Voltage & Angle, to equivalent 2-D vector components (Vd, Vq)
		start_gamma = START_GAMMA_OPENLOOP; 
		end_gamma = END_GAMMA_OPENLOOP; 
		req_gamma = REQ_GAMMA_CLOSEDLOOP; 
	
		motor_s.end_hall = POSI_LAST_HALL_STATE; // Choose last Hall state of 6-state cycle
	} // else !(0 > motor_s.req_veloc)

	// Use Park Transform to convert Absolute-Voltage & Angle, to equivalent 2-D vector components (Vd, Vq)
	park_transform( start_D ,start_Q ,0 ,START_VOLT_OPENLOOP ,start_gamma );
	park_transform( end_D ,end_Q ,0 ,END_VOLT_OPENLOOP ,end_gamma );
	park_transform( req_D ,req_Q ,0 ,REQ_VOLT_CLOSEDLOOP ,req_gamma );
	
	// Initialise each component of rotating vector
	init_rotation_component( motor_s ,D_ROTA ,start_D ,end_D ,req_D );
	init_rotation_component( motor_s ,Q_ROTA ,start_Q ,end_Q ,req_Q );
	init_blend_data( motor_s );	// Initialise blending data for 'TRANSIT state'

	motor_s.tmp = 0; // MB~ Dbg
	motor_s.temp = 0; // MB~ Dbg

	motor_s.dbg_sum = 0; // MB~
	motor_s.dbg_err = 0; // MB~
	motor_s.dbg_prev = QEI_REV_MASK; // MB~
	motor_s.dbg_diff = 1; // MB~

} // init_motor
/*****************************************************************************/
static void init_pwm( // Initialise PWM parameters for one motor
	PWM_COMMS_TYP &pwm_comms_s, // reference to structure containing PWM data
	chanend c_pwm, // PWM channel connecting Client & Server
	unsigned motor_id // Unique Motor identifier e.g. 0 or 1
)
{
	int phase_cnt; // phase counter


	pwm_comms_s.buf = 0; // Current double-buffer in use at shared memory address
	pwm_comms_s.params.id = motor_id; // Unique Motor identifier e.g. 0 or 1

	// initialise arrays
	for (phase_cnt = 0; phase_cnt < NUM_PWM_PHASES; phase_cnt++)
	{ 
		pwm_comms_s.params.widths[phase_cnt] = 0;
	} // for phase_cnt

	// Receive the address of PWM data structure from the PWM server, in case shared memory is used
	c_pwm :> pwm_comms_s.mem_addr; // Receive shared memory address from PWM server

	return;
} // init_pwm 
/*****************************************************************************/
static void error_pwm_values( // Set PWM values to error condition
	unsigned pwm_vals[]	// Array of PWM widths
)
{
	int phase_cnt; // phase counter


	// loop through all phases
	for (phase_cnt = 0; phase_cnt < NUM_PWM_PHASES; phase_cnt++)
	{ 
		pwm_vals[phase_cnt] = -1;
	} // for phase_cnt
} // error_pwm_values
/*****************************************************************************/
static int filter_adc_extrema( 		// Smooths adc extrema values using low-pass filter
	MOTOR_DATA_TYP &motor_s,	// reference to structure containing motor data
	ADC_TYP extreme_val						// Either a minimum or maximum ADC value
) // Returns filtered output value
/* This is a 1st order IIR filter, it is configured as a low-pass filter, 
 * The input value is up-scaled, to allow integer arithmetic to be used.
 * The output mean value is down-scaled by the same amount.
 * Error diffusion is used to keep control of systematic quantisation errors.
 */
{
	int scaled_inp = (extreme_val << XTR_SCALE_BITS); // Upscaled QEI input value
	int diff_val; // Difference between input and filtered output
	int increment; // new increment to filtered output value
	ADC_TYP out_val; // filtered output value


	// Form difference with previous filter output
	diff_val = scaled_inp - motor_s.filt_adc;

	// Multiply difference by filter coefficient (alpha)
	diff_val += motor_s.coef_err; // Add in diffusion error;
	increment = (diff_val + XTR_HALF_COEF) >> XTR_COEF_BITS ; // Multiply by filter coef (with rounding)
	motor_s.coef_err = diff_val - (increment << XTR_COEF_BITS); // Evaluate new quantisation error value 

	motor_s.filt_adc += increment; // Update (up-scaled) filtered output value

	// Update mean value by down-scaling filtered output value
	motor_s.filt_adc += motor_s.scale_err; // Add in diffusion error;
	out_val = (motor_s.filt_adc + XTR_HALF_SCALE) >> XTR_SCALE_BITS; // Down-scale
	motor_s.scale_err = motor_s.filt_adc - (out_val << XTR_SCALE_BITS); // Evaluate new remainder value 

	return out_val; // return filtered output value
} // filter_adc_extrema
/*****************************************************************************/
static ADC_TYP smooth_adc_maxima( // Smooths maximum ADC values
	MOTOR_DATA_TYP &motor_s // reference to structure containing motor data
)
{
	ADC_TYP max_val = motor_s.adc_params.vals[0]; // Initialise maximum to first phase
	ADC_TYP out_val; // filtered output value
	int phase_cnt; // phase counter


	for (phase_cnt = 1; phase_cnt < NUM_ADC_PHASES; phase_cnt++)
	{ 
		if (max_val < motor_s.adc_params.vals[phase_cnt]) max_val = motor_s.adc_params.vals[phase_cnt]; // Update maximum
	} // for phase_cnt

	out_val = filter_adc_extrema( motor_s ,max_val );

	return out_val;
} // smooth_adc_maxima
/*****************************************************************************/
static ADC_TYP smooth_adc_minima( // Smooths minimum ADC values
	MOTOR_DATA_TYP &motor_s // reference to structure containing motor data
)
{
	ADC_TYP min_val = motor_s.adc_params.vals[0]; // Initialise minimum to first phase
	ADC_TYP out_val; // filtered output value
	int phase_cnt; // phase counter


	for (phase_cnt = 1; phase_cnt < NUM_ADC_PHASES; phase_cnt++)
	{ 
		if (min_val > motor_s.adc_params.vals[phase_cnt]) min_val = motor_s.adc_params.vals[phase_cnt]; // Update minimum
	} // for phase_cnt

	out_val = filter_adc_extrema( motor_s ,min_val );

	return out_val;
} // smooth_adc_minima
/*****************************************************************************/
static void estimate_Iq_from_ADC_extrema( // Estimate Iq value from ADC signals. NB Assumes requested Id is Zero
	MOTOR_DATA_TYP &motor_s // reference to structure containing motor data
)
{
	int out_val; // Measured Iq output value


	if (0 > motor_s.req_veloc)
	{ // Iq is negative for negative velocity
		out_val = smooth_adc_minima( motor_s );
	} // if (0 > motor_s.req_veloc)
	else
	{ // Iq is positive for positive velocity
		out_val = smooth_adc_maxima( motor_s );
	} // if (0 > motor_s.req_veloc)

	motor_s.vect_data[Q_ROTA].est_I = out_val;
	motor_s.vect_data[D_ROTA].est_I = 0;
} // estimate_Iq_from_ADC_extrema
/*****************************************************************************/
static void estimate_Iq_from_velocity( // Estimates tangential coil current from measured velocity
	MOTOR_DATA_TYP &motor_s // reference to structure containing motor data
)
/* This function uses the following relationship
 * est_Iq = GRAD * sqrt( meas_veloc )   where GRAD was found by experiment.
 * WARNING: GRAD will be different for different motors.
 */
{
	int scaled_vel = VEL_GRAD * motor_s.qei_params.veloc; // scaled angular velocity


	if (0 > motor_s.qei_params.veloc)
	{
		motor_s.vect_data[Q_ROTA].est_I = -sqrtuint( -scaled_vel );
	} // if (0 > motor_s.qei_params.veloc)
	else
	{
		motor_s.vect_data[Q_ROTA].est_I = sqrtuint( scaled_vel );
	} // if (0 > motor_s.qei_params.veloc)
} // estimate_Iq_from_velocity
/*****************************************************************************/
static void filter_current_component( // Filters estimated current component
	ROTA_DATA_TYP &vect_data_s // Reference to structure containing data for one rotational vector component
)
{
	int corr_val; // Correction to previous value
	int filt_val; // Filtered correction value


	// Do Low-pass filter ...

	corr_val = (vect_data_s.inp_I - vect_data_s.est_I); // compute correction to previous filtered value
	corr_val += vect_data_s.rem_I; // Add in error diffusion remainder

	filt_val = (corr_val + ROTA_HALF_FILT) >> ROTA_FILT_BITS ; // 1st order filter (uncalibrated value)
	vect_data_s.rem_I = corr_val - (filt_val << ROTA_FILT_BITS); // Update remainder

	vect_data_s.est_I += filt_val; // Add filtered difference to previous value
} // init_rotation_component
/*****************************************************************************/
static void estimate_Iq_using_transforms( // Calculate Id & Iq currents using transforms. NB Required if requested Id is NON-zero
	MOTOR_DATA_TYP &motor_s // reference to structure containing motor data
)
{
	ROTA_VECT_ENUM comp_cnt; // Counts vector components
	int alpha_meas = 0; // Measured currents once transformed to a 2D vector
	int beta_meas = 0;	// Measured currents once transformed to a 2D vector

#pragma xta label "foc_loop_clarke"

	clarke_transform( motor_s.adc_params.vals[ADC_PHASE_A], motor_s.adc_params.vals[ADC_PHASE_B], motor_s.adc_params.vals[ADC_PHASE_C], alpha_meas, beta_meas );
// if (motor_s.xscope) xscope_int( 3 ,(-alpha_meas * 17) );
// if (motor_s.xscope) xscope_int( 4 ,(-beta_meas * 17) );

#pragma xta label "foc_loop_park"

	// Estimate coil currents (Id & Iq) using park transform
	// NB Invert alpha & beta here, as Back_EMF is in opposite direction to applied (PWM) voltage

	park_transform( motor_s.vect_data[D_ROTA].inp_I ,motor_s.vect_data[Q_ROTA].inp_I ,-alpha_meas ,-beta_meas ,motor_s.set_theta );

	// Filter current estimate
	for (comp_cnt=0; comp_cnt<NUM_ROTA_COMPS; comp_cnt++)
	{ 
		filter_current_component( motor_s.vect_data[comp_cnt] );
	} // for comp_cnt

// if (motor_s.xscope) xscope_int( 2 ,motor_s.vect_data[D_ROTA].est_I );
// if (motor_s.xscope) xscope_int( 3 ,motor_s.vect_data[Q_ROTA].est_I );

{ // MB~
	int d_abs = abs(motor_s.vect_data[D_ROTA].est_I);
	int q_abs = abs(motor_s.vect_data[Q_ROTA].est_I);
	int h_abs;
	int eficny;


	if (d_abs > q_abs)
	{
		h_abs = d_abs + (q_abs >> 1);
	} // if (d_abs > q_abs)
	else
	{
		h_abs = q_abs + (d_abs >> 1);
	} // else (d_abs > q_abs)

//	if (motor_s.xscope) xscope_int( 5 ,h_abs );
	if (h_abs > 0)
	{
		eficny = (motor_s.qei_params.veloc + (h_abs >> 1)) / h_abs;  
	} // if (h_abs > 0)
	else
	{
		eficny = 0;
	} // if (h_abs > 0)
// if (motor_s.xscope) xscope_int( 4 ,eficny );
} // MB~

} // estimate_Iq_using_transforms
/*****************************************************************************/
static unsigned convert_volts_to_pwm_width( // Converted voltages to PWM pulse-widths
	int inp_V  // Input voltage
)
{
	unsigned out_pwm; // output PWM pulse-width


	out_pwm = (inp_V + VOLT_OFFSET) >> VOLT_TO_PWM_BITS; // Convert voltage to PWM value. NB Always +ve

	// Clip PWM value into allowed range
	if (out_pwm > PWM_MAX_LIMIT)
	{
		out_pwm = PWM_MAX_LIMIT;
	} // if (out_pwm > PWM_MAX_LIMIT)
	else
	{
		if (out_pwm < PWM_MIN_LIMIT)
		{
			out_pwm = PWM_MIN_LIMIT;
		} // if (out_pwm < PWM_MIN_LIMIT)
	} // else !(out_pwm > PWM_MAX_LIMIT)

	return out_pwm; // return clipped PWM value
} // convert_volts_to_pwm_width
/*****************************************************************************/
static int smooth_demand_voltage( // Limits large changes in demand voltage
	ROTA_DATA_TYP &vect_data_s // Reference to structure containing data for one rotational vector component
) // Returns smoothed voltage
{
	int set_V = vect_data_s.set_V;  // Input demand voltage (changed for output)


	if (set_V > (vect_data_s.prev_V + HALF_SMOOTH_VOLT))
	{
		set_V = vect_data_s.prev_V + SMOOTH_VOLT_INC; // Allow small increase
	} // if (set_V > vect_data_s.prev_V)
	else
	{
		if (set_V < (vect_data_s.prev_V - HALF_SMOOTH_VOLT))
		{
			set_V = vect_data_s.prev_V - SMOOTH_VOLT_INC; // Allow small decrease
		} // if 
	} // else !(set_V > vect_data_s.prev_V)

	vect_data_s.prev_V = set_V; // Store smoothed demand voltage

	return set_V; // Return smoothed value
} // smooth_demand_voltage
/*****************************************************************************/
static void dq_to_pwm ( // Convert Id & Iq input values to 3 PWM output values 
	MOTOR_DATA_TYP &motor_s // Reference to structure containing motor data
)
{
	int volts[NUM_PWM_PHASES];	// array of intermediate demand voltages for each phase
	unsigned inp_theta = motor_s.set_theta; // Input demand theta
	int alpha_set = 0, beta_set = 0; // New intermediate demand voltages as a 2D vector
	int phase_cnt; // phase counter


	motor_s.vect_data[D_ROTA].set_V = smooth_demand_voltage( motor_s.vect_data[D_ROTA] );
	motor_s.vect_data[Q_ROTA].set_V = smooth_demand_voltage( motor_s.vect_data[Q_ROTA] );

// if (motor_s.xscope) xscope_int( 5 ,motor_s.vect_data[D_ROTA].set_V ); //MB~
// if (motor_s.xscope) xscope_int( 4 ,motor_s.vect_data[Q_ROTA].set_V ); //MB~

// if (motor_s.xscope) xscope_int( (2+motor_s.id) ,inp_theta ); //MB~
 
	// Inverse park  [d, q, theta] --> [alpha, beta]
	inverse_park_transform( alpha_set, beta_set, motor_s.vect_data[D_ROTA].set_V, motor_s.vect_data[Q_ROTA].set_V, inp_theta );

	// Final voltages applied: 
	inverse_clarke_transform( volts[PWM_PHASE_A] ,volts[PWM_PHASE_B] ,volts[PWM_PHASE_C] ,alpha_set ,beta_set ); // Correct order

	/* Scale to 12bit unsigned for PWM output */
	for (phase_cnt = 0; phase_cnt < NUM_PWM_PHASES; phase_cnt++)
	{
		motor_s.pwm_comms.params.widths[phase_cnt] = convert_volts_to_pwm_width( volts[phase_cnt] );
	} // for phase_cnt
} // dq_to_pwm
/*****************************************************************************/
static void calc_open_loop_pwm ( // Calculate open-loop PWM output values to spins magnetic field around (regardless of the encoder)
	MOTOR_DATA_TYP &motor_s // reference to structure containing motor data
)
{
	unsigned cur_time; // current time value
	int diff_time; // time since last theta update


	motor_s.tymer :> cur_time;
	diff_time = (int)(cur_time - motor_s.prev_pwm_time); 

	// Check if theta needs incrementing
	if (motor_s.pwm_period < diff_time)
	{
		if (motor_s.req_veloc < 0)
		{
			motor_s.open_theta--; // Decrement demand theta value
		} // if (motor_s.req_veloc < 0)
		else
		{
			motor_s.open_theta++; // Increment demand theta value
		} // else !(motor_s.req_veloc < 0)

		motor_s.prev_pwm_time = cur_time; // Update previous time
	} // if (motor_s.pwm_period < diff_time)

	// NB QEI_REV_MASK correctly maps -ve values into +ve range 0 <= theta < QEI_PER_REV;
	motor_s.set_theta = motor_s.open_theta & QEI_REV_MASK; // Convert to base-range [0..QEI_REV_MASK]

} // calc_open_loop_pwm
/*****************************************************************************/
static int calc_foc_angle( // Calculate PWM angle for FOC
	MOTOR_DATA_TYP &motor_s, // reference to structure containing motor data
	int qei_theta // QEI angle
) // Returns FOC angle
{
	int foc_angle; 	// FOC angle

	foc_angle = qei_theta + motor_s.qei_offset; // Calculate FOC angle

	return foc_angle; // Return FOC angle
} // calc_foc_angle 
/*****************************************************************************/
static int increment_voltage_component( // Returns incremented voltage for one vector component
	ROTA_DATA_TYP &vect_data_s // Reference to structure containing data for one rotating vector component
)
{
	int upscaled_inc_V = vect_data_s.diff_V; // Up-scaled Voltage increment
	int new_inc_V; // New Voltage increment
	int out_V; // New Output Voltage


	upscaled_inc_V += vect_data_s.rem_V; // Add in remainder for error diffusion
 	new_inc_V = (upscaled_inc_V + HALF_BLEND) >> BLEND_BITS; // Calculate new Voltage increment
	vect_data_s.rem_V = upscaled_inc_V - (new_inc_V << BLEND_BITS);  // Update remainder for error diffusion

	out_V = vect_data_s.trans_V + new_inc_V; 

	return out_V; // Return incremented demand voltage
} // increment_voltage_component
/*****************************************************************************/
static void calc_transit_pwm( // Calculate FOC PWM output values
	MOTOR_DATA_TYP &motor_s // reference to structure containing motor data
)
{
	unsigned cur_time; // current time value
	int diff_time; // time since last theta update
	int weight;	// Weighting for FOC theta


#pragma xta label "trans_loop_speed_pid"


	motor_s.tymer :> cur_time;
	diff_time = (int)(cur_time - motor_s.prev_pwm_time); 

	// Check if theta needs incrementing
	if (motor_s.pwm_period < diff_time)
	{
		motor_s.prev_pwm_time = cur_time; // Update previous time

		if (motor_s.req_veloc < 0)
		{
			motor_s.open_theta--; // Decrement demand theta value
		} // if (motor_s.req_veloc < 0)
		else
		{
			motor_s.open_theta++; // Increment demand theta value
		} // else !(motor_s.req_veloc < 0)

		motor_s.trans_cnt++; // Increment trans_theta update counter

		// Check if time for a Demand voltage increment
		if (motor_s.trans_cycles == motor_s.trans_cnt)
		{
			motor_s.trans_cnt = 0; // Reset trans_theta update counter
		
			// Smoothly change Demand voltage from open-loop to closed-loop values
			motor_s.vect_data[D_ROTA].trans_V = increment_voltage_component( motor_s.vect_data[D_ROTA] );
			motor_s.vect_data[Q_ROTA].trans_V = increment_voltage_component( motor_s.vect_data[Q_ROTA] );
		} // if (motor_s.trans_cycles == motor_s.trans_cnt)
	} // if (motor_s.pwm_period < diff_time)

	// WARNING: Do NOT allow  motor_s.foc_theta or motor_s.open_theta to wrap otherwise weighting will NOT work

	motor_s.foc_theta = calc_foc_angle( motor_s ,motor_s.tot_ang ); // Calculate FOC-angle, from QEI-angle

	// Calculate weighting for FOC.
	weight = motor_s.blend_up * (abs(motor_s.open_theta) - motor_s.search_theta);

	// NB Round and down-scale
	motor_s.set_theta = (weight * (motor_s.foc_theta - motor_s.open_theta) + motor_s.half_blend) >> motor_s.blend_bits;

	motor_s.set_theta += motor_s.open_theta;

	motor_s.set_theta &= QEI_REV_MASK; // Convert to base-range [0..QEI_REV_MASK]

	motor_s.vect_data[D_ROTA].set_V = motor_s.vect_data[D_ROTA].trans_V; 
	motor_s.vect_data[Q_ROTA].set_V = motor_s.vect_data[Q_ROTA].trans_V; 
} // calc_transit_pwm
/*****************************************************************************/
static void calc_foc_pwm( // Calculate FOC PWM output values
	MOTOR_DATA_TYP &motor_s // reference to structure containing motor data
)
/* The estimated tangential coil current (Iq), is much less than the requested value (Iq)
 * (The ratio is between 25..42 for the LDO motors)
 * The estimated and requested values fed into the Iq PID must be simliar to ensure correct operation
 * Therefore, the requested value is scaled down by a factor of 32.
 */
{
	int targ_Id;	// target 'radial' current value
	int targ_Iq;	// target 'tangential' current value
	int corr_Id;	// Correction to radial current value
	int corr_Iq;	// Correction to tangential current value
	int corr_veloc;	// Correction to angular velocity


#pragma xta label "foc_loop_speed_pid"

	// Applying Speed PID.

	// Check if PID's need presetting
	if (motor_s.first_pid)
	{
		preset_pid( motor_s.id ,motor_s.pid_regs[SPEED_PID] ,motor_s.pid_consts[motor_s.Iq_alg][SPEED_PID] ,motor_s.qei_params.veloc ,motor_s.req_veloc ,motor_s.qei_params.veloc );
	}; // if (motor_s.first_pid)

#ifdef MB
	if (motor_s.iters > 50000)
	{ // Track est_Iq value
		motor_s.temp++; 
	
		if (100 == motor_s.temp)
		{
			motor_s.temp = 0; 
			if (4000 > motor_s.req_veloc) motor_s.req_veloc++;
		} // if (1024 == motor_s.temp)
	} //if (motor_s.iters > 25000)
#endif //MB~

// if (motor_s.xscope) xscope_int( 3 ,motor_s.req_veloc);
	corr_veloc = get_pid_regulator_correction( motor_s.id ,motor_s.pid_regs[SPEED_PID] ,motor_s.pid_consts[motor_s.Iq_alg][SPEED_PID] ,motor_s.req_veloc ,motor_s.qei_params.veloc );
// if (motor_s.xscope) xscope_int( 6 ,corr_veloc );

	// Calculate velocity PID output
	if (PROPORTIONAL)
	{ // Proportional update
		motor_s.pid_veloc = corr_veloc;
	} // if (PROPORTIONAL)
	else
	{ // Offset update
		motor_s.pid_veloc = motor_s.vect_data[Q_ROTA].req_closed_V + corr_veloc;
	} // else !(PROPORTIONAL)

// if (motor_s.xscope) xscope_int( 9 ,motor_s.pid_regs[SPEED_PID].sum_err  );
// if (motor_s.xscope) xscope_int( 7 ,motor_s.pid_veloc );
	if (VELOC_CLOSED)
	{ // Evaluate requested IQ from velocity PID
		motor_s.vect_data[Q_ROTA].req_closed_V = 415 + (abs(motor_s.pid_veloc) >> 1);
	} // if (VELOC_CLOSED)

#ifdef MB
	if (motor_s.iters > 50000)
	{ // Track est_Iq value
		motor_s.temp++; 
	
		if (256 == motor_s.temp)
		{
			motor_s.temp = 0; 
			if (4000 > motor_s.qei_params.veloc) motor_s.vect_data[Q_ROTA].req_closed_V--;
		} // if (1024 == motor_s.temp)
	} //if (motor_s.iters > 25000)
#endif //MB~

#pragma xta label "foc_loop_id_iq_pid"

	// Select algorithm for estimateing coil current Iq (and Id)
	// WARNING: Changing algorithm will require re-tuning of PID values
	switch ( motor_s.Iq_alg )
	{
		case TRANSFORM : // Use Park/Clarke transforms
			estimate_Iq_using_transforms( motor_s );

			// WARNING: Dividing by a larger amount, may switch the sign of the PID error, and cause instability
			targ_Iq = (motor_s.vect_data[Q_ROTA].req_closed_V + 4) >> 3; // Scale requested value to be of same order as estimated value
		break; // case TRANSFORM
	
		case EXTREMA : // Use Extrema of measured ADC values
			estimate_Iq_from_ADC_extrema( motor_s );
			targ_Iq = (motor_s.vect_data[Q_ROTA].req_closed_V + 16) >> 5; // Scale requested value to be of same order as estimated value
		break; // case EXTREMA 
	
		case VELOCITY : // Use measured velocity
			estimate_Iq_from_velocity( motor_s );
			targ_Iq = motor_s.vect_data[Q_ROTA].req_closed_V; // NB No scaling required
		break; // case VELOCITY 
	
    default: // Unsupported
			printstr("ERROR: Unsupported Iq Estimation algorithm > "); 
			printintln( motor_s.Iq_alg );
			assert(0 == 1);
    break;
	} // switch (motor_s.Iq_alg)

	/* Here we would like to set targ_Id = 0, for max. efficiency.
	 * However, some motors can NOT reach 4000 RPM, no matter how much Vq is increased.
   * Therefore need to adjust targ_Id slowly towards targ_Id = sign(req_veloc) * est_Iq;
	 * NB Future work:- Add new motor-state where targ_Id = sign(req_veloc) * est_Iq;
	 * req_V is much lower for this state, ~920. Therefore need smooth tranasition
	 * Also, each time targ_Id is changed, Iq takes about 1 second to stabilise 
   */

// if (motor_s.xscope) xscope_int( 7 ,motor_s.vect_data[D_ROTA].req_closed_V ); //MB~
// if (motor_s.xscope) xscope_int( 8 ,motor_s.vect_data[Q_ROTA].req_closed_V ); //MB~

	targ_Iq = ((V2I_MUX * motor_s.vect_data[Q_ROTA].req_closed_V + HALF_V2I) >> V2I_BITS) + V2I_OFF;

	// Check if target Iq is too large
	if (targ_Iq > IQ_LIM)
	{ // Apply field weakening
		int abs_Id = (targ_Iq - IQ_LIM + IQ_ID_HALF) >> IQ_ID_BITS; // Calculate new absolute Id value, NB divide by IQ_ID_RATIO 

		targ_Id = abs(motor_s.prev_Id); // Preset to magnitude of previous value
		
		// Add a bit of hysterisis. MB~ This can probably be removed when Id resolution improved
		if (targ_Id > (abs_Id + 1))
		{ 
			targ_Id = abs_Id + 1; 
		} // if (targ_Id > (abs_Id + 1))
		else
		{
			if (targ_Id < (abs_Id - 1)) targ_Id = abs_Id - 1; 
		} // else !(targ_Id > (abs_Id + 1))

		if (0 > motor_s.req_veloc)
		{ // Negative spin direction
			targ_Id = -targ_Id; // targ_Id must be -ve for -ve spin
		} // if (0 > motor_s.req_veloc)

		targ_Iq = IQ_LIM; // Limit target Iq value
	} // if ((targ_Iq > IQ_LIM)
	else
	{ // NO field-weakening
		targ_Id = 0;
	} // if ((targ_Iq > IQ_LIM)

	motor_s.prev_Id = targ_Id; // Update previous target Id value

	// Apply PID control to Iq and Id

	// Check if PID's need presetting
	if (motor_s.first_pid)
	{
		preset_pid( motor_s.id ,motor_s.pid_regs[ID_PID] ,motor_s.pid_consts[motor_s.Iq_alg][ID_PID] ,motor_s.vect_data[D_ROTA].end_open_V ,targ_Id ,motor_s.vect_data[D_ROTA].est_I );
		preset_pid( motor_s.id ,motor_s.pid_regs[IQ_PID] ,motor_s.pid_consts[motor_s.Iq_alg][IQ_PID] ,motor_s.vect_data[Q_ROTA].end_open_V ,targ_Iq ,motor_s.vect_data[Q_ROTA].est_I );
	}; // if (motor_s.first_pid)

// if (motor_s.xscope) xscope_int( 6 ,targ_Id );
// if (motor_s.xscope) xscope_int( 9 ,targ_Iq );
	corr_Id = get_pid_regulator_correction( motor_s.id ,motor_s.pid_regs[ID_PID] ,motor_s.pid_consts[motor_s.Iq_alg][ID_PID] ,targ_Id ,motor_s.vect_data[D_ROTA].est_I );
	corr_Iq = get_pid_regulator_correction( motor_s.id ,motor_s.pid_regs[IQ_PID] ,motor_s.pid_consts[motor_s.Iq_alg][IQ_PID] ,targ_Iq ,motor_s.vect_data[Q_ROTA].est_I );

// if (motor_s.xscope) xscope_int( 5 ,motor_s.pid_regs[ID_PID].sum_err  );

	if (PROPORTIONAL)
	{ // Proportional update
		motor_s.pid_Id = corr_Id;
		motor_s.pid_Iq = corr_Iq;
	} // if (PROPORTIONAL)
	else
	{ // Offset update
		motor_s.pid_Id = targ_Id + corr_Id;
		motor_s.pid_Iq = targ_Iq + corr_Iq;
	} // else !(PROPORTIONAL)

	if (IQ_ID_CLOSED)
	{ // Update set DQ values
		motor_s.vect_data[D_ROTA].set_V  = motor_s.pid_Id;
		motor_s.vect_data[Q_ROTA].set_V = motor_s.pid_Iq;

// motor_s.vect_data[D_ROTA].set_V = motor_s.vect_data[D_ROTA].req_closed_V; // MB~
// motor_s.vect_data[Q_ROTA].set_V = motor_s.vect_data[Q_ROTA].req_closed_V; // MB~
	} // if (IQ_ID_CLOSED)
	else
	{ // Open-loop
		motor_s.vect_data[D_ROTA].set_V  = motor_s.vect_data[D_ROTA].start_open_V ; // Ideal voltage producing radial magnetic field (NB never update as no radial force is required)
		motor_s.vect_data[Q_ROTA].set_V = motor_s.vect_data[Q_ROTA].start_open_V; // Ideal voltage producing tangential magnetic field. (NB Updated based on the speed error)

		calc_open_loop_pwm( motor_s );
	} // else !(IQ_ID_CLOSED)

	// Update 'demand' theta value for next dq_to_pwm iteration from QEI-angle
	motor_s.set_theta = calc_foc_angle( motor_s ,motor_s.qei_params.theta );

	motor_s.set_theta &= QEI_REV_MASK; // Convert to base-range [0..QEI_REV_MASK]

	motor_s.first_pid = 0; // Clear 'first PID' flag

#if  (1 == GAMMA_SWEEP)
{ //MB~
	int tmp_theta;


	motor_s.temp++; 

	if (1024 == motor_s.temp)
	{
		motor_s.temp = 0; 
		motor_s.tmp++;
	} // if (1024 == motor_s.temp)

	tmp_theta = motor_s.tmp & (QEI_PER_PAIR - 1); // Mask into electrical cycle range

	park_transform( motor_s.vect_data[D_ROTA].set_V ,motor_s.vect_data[Q_ROTA].set_V ,0 ,END_VOLT_OPENLOOP ,tmp_theta ); //MB~

// if (motor_s.xscope) xscope_int( 6 ,tmp_theta );
} //MB~
#endif //(1 == GAMMA_SWEEP)


} // calc_foc_pwm
/*****************************************S************************************/
static void update_motor_state( // Update state of motor based on motor sensor data
	MOTOR_DATA_TYP &motor_s // reference to structure containing motor data
)
/* This routine is inplemented as a Finite-State-Machine (FSM) with the following 5 states:-
 *	SEARCH: Warm-up state where the motor is turned until the FOC start condition is found
 *	FOC: 		Normal FOC state
 *	STALL:	Motor has stalled, 
 *	STOP:		Error state: Destination state if error conditions are detected
 *
 * During the SEARCH state, the motor runs in open loop, monitoring both Hall sensor and QEI responses,
 * then when synchronisation has been achieved the motor switches to the FOC state, which uses the main FOC algorithm.
 * If too long a time is spent in the STALL state, this becomes an error and the motor is stopped.
 */
{
	MOTOR_STATE_ENUM motor_state; // local motor state


	// Update motor state based on new sensor data
	switch( motor_s.state )
	{
		case ALIGN: // Align Motor coils opposite matgnets
		{
			unsigned ts1;	// timestamp
			unsigned diff_time;	// time difference

			motor_s.tymer :> ts1;
			diff_time = ts1 - motor_s.prev_pwm_time;

			if (ALIGN_PERIOD < diff_time)
			{
				motor_s.prev_pwm_time = ts1; // Store time-stamp
				motor_state = SEARCH; 
				motor_s.state = motor_state; // NB Required due to XC compiler rules
			} // if 
		} break; // case ALIGN
	
		case SEARCH : // Turn motor using theta steps (open-loop), and update motor state
			// Check if end of SEACRH state
			if (motor_s.search_theta == abs(motor_s.open_theta))
			{ /* Calculate QEI offset
				 * In FOC state, theta will be based on angle returned from QEI sensor.
         * So recalibrate, to create smooth transition between SEARCH and FOC states.  
				 */
				motor_s.qei_offset = motor_s.open_theta - motor_s.tot_ang; // Difference betweene Open-loop and QEI angle
acquire_lock(); printstrln("TRANSIT");release_lock(); //MB~
				motor_state = TRANSIT; 
				motor_s.state = motor_state; // NB Required due to XC compiler rules
			} // if (QEI_PER_PAIR == abs(motor_s.open_theta))
		break; // case SEARCH 
	
		case TRANSIT : // Transit between open-loop and FOC, and update motor state
			// Check if end of SEACRH state
			if (motor_s.trans_theta == abs(motor_s.open_theta))
			{
				motor_s.vect_data[D_ROTA].start_open_V = motor_s.vect_data[D_ROTA].end_open_V; // NB Correct for any rounding inaccuracy from TRANSIT state
				motor_s.vect_data[Q_ROTA].start_open_V = motor_s.vect_data[Q_ROTA].end_open_V; // NB Correct for any rounding inaccuracy from TRANSIT state
acquire_lock(); printstrln("FOC");release_lock(); //MB~
				motor_state = FOC; 
				motor_s.state = motor_state; // NB Required due to XC compiler rules
			} // if ((QEI_PER_PAIR << 1) == abs(motor_s.open_theta))
		break; // case TRANSIT
	
		case FOC : // Normal FOC state
			// Check for a stall
#ifdef MB
			// check for correct spin direction
      if (0 > motor_s.half_veloc)
			{
				if (motor_s.qei_params.veloc > -motor_s.half_veloc)
				{	// Spinning in wrong direction
					motor_s.err_data.err_flgs |= (1 << DIRECTION_ERR);
					motor_s.err_data.line[DIRECTION_ERR] = __LINE__;
					motor_s.state = STOP; // Switch to stop state
					motor_s.cnts[STOP] = 0; // Initialise stop-state counter 
				} // if (motor_s.qei_params.veloc > -motor_s.half_veloc)
      } // if (0 > motor_s.half_veloc)
			else
			{
				if (motor_s.qei_params.veloc < -motor_s.half_veloc)
				{	// Spinning in wrong direction
					motor_s.err_data.err_flgs |= (1 << DIRECTION_ERR);
					motor_s.err_data.line[DIRECTION_ERR] = __LINE__;
					motor_s.state = STOP; // Switch to stop state
					motor_s.cnts[STOP] = 0; // Initialise stop-state counter 
				} // if (motor_s.qei_params.veloc < -motor_s.half_veloc)
if (motor_s.state == STOP)
{
acquire_lock(); printstr("BAD VEL="); printintln(motor_s.qei_params.veloc); release_lock(); //MB~
}
      } // if (0 > motor_s.half_veloc)
#endif //MB~

			if (motor_s.meas_speed < STALL_SPEED) 
			{
				motor_s.state = STALL; // Switch to stall state
				motor_s.cnts[STALL] = 0; // Initialise stall-state counter 
			} // if (motor_s.meas_speed < STALL_SPEED)
		break; // case FOC
	
		case STALL : // state where motor stalled
			// Check if still stalled
#ifdef MB
			if (motor_s.meas_speed < STALL_SPEED) 
			{
				// Check if too many stalled states
				if (motor_s.cnts[STALL] > STALL_TRIP_COUNT) 
				{
					motor_s.err_data.err_flgs |= (1 << STALL);
					motor_s.err_data.line[STALL] = __LINE__;
					motor_s.state = STOP; // Switch to stop state
					motor_s.cnts[STOP] = 0; // Initialise stop-state counter 
				} // if (motor_s.cnts[STALL] > STALL_TRIP_COUNT) 
			} // if (motor_s.meas_speed < STALL_SPEED) 
			else
#endif //MB~
			{ // No longer stalled
				motor_s.state = FOC; // Switch to main FOC state
				motor_s.cnts[FOC] = 0; // Initialise FOC-state counter 
			} // else !(motor_s.meas_speed < STALL_SPEED) 
		break; // case STALL
	
		case STOP : // Error state where motor stopped
			// Absorbing state. Nothing to do
		break; // case STOP
	
    default: // Unsupported
			assert(0 == 1); // Motor state not supported
    break;
	} // switch( motor_s.state )

	motor_s.cnts[motor_s.state]++; // Update counter for new motor state 

	// Select correct method of calculating DQ values
#pragma fallthrough
	switch( motor_s.state )
	{
		case ALIGN : // Turn motor until Aligned
			// Nothing to do
		break; // case ALIGN
	
		case SEARCH : // Turn motor until FOC start condition found
 			calc_open_loop_pwm( motor_s );
		break; // case SEARCH 
	
		case TRANSIT : // Smoothly transit from open-loop to FOC
 			calc_transit_pwm( motor_s );
		break; // case SEARCH 
	
		case FOC : // Normal FOC state
			calc_foc_pwm( motor_s );
		break; // case FOC

		case STALL : // state where motor stalled
			calc_foc_pwm( motor_s );
		break; // case STALL

		case STOP : // Error state where motor stopped
			// Nothing to do
		break; // case STOP
	
    default: // Unsupported
			assert(0 == 1); // Motor state not supported
    break;
	} // switch( motor_s.state )

	return;
} // update_motor_state
/*****************************************************************************/
void wait_for_servers_to_start( // Wait for other servers to initialise
	MOTOR_DATA_TYP &motor_s, // Structure containing motor data
	chanend c_wd, // Channel for communication with WatchDog server
	chanend c_pwm, // Channel for communication with PWM server
	streaming chanend c_hall, // Channel for communication with Hall server 
	streaming chanend c_qei,  // Channel for communication with QEI server
	streaming chanend c_adc_cntrl // Channel for communication with ADC server
)
#define NUM_INIT_SERVERS 4 // WARNING: edit this line to indicate No of servers to wait on
{
	unsigned ts1;	// timestamp
	int wait_cnt = NUM_INIT_SERVERS; // Set number of un-initialised servers
	unsigned cmd; // Command from Server


	c_wd <: WD_CMD_INIT;	// Signal WatchDog to Initialise ...

	// Loop until all servers have started
	while(0 < wait_cnt)
	{
		select {
			case c_hall :> cmd : // Hall server
			{
				assert(HALL_CMD_ACK == cmd); // ERROR: Hall server did NOT send acknowledge signal
				wait_cnt--; // Decrement number of un-initialised servers
			} // case	c_hall
			break;

			case c_qei :> cmd : // QEI server
			{
				assert(QEI_CMD_ACK == cmd); // ERROR: QEI server did NOT send acknowledge signal
				wait_cnt--; // Decrement number of un-initialised servers
			} // case	c_qei
			break;

			case c_adc_cntrl :> cmd : // ADC server
			{
				assert(ADC_CMD_ACK == cmd); // ERROR: ADC server did NOT send acknowledge signal
				wait_cnt--; // Decrement number of un-initialised servers
			} // case	c_adc_cntrl
			break;

			case c_wd :> cmd : // WatchDog server
			{
				assert(WD_CMD_ACK == cmd); // ERROR: WatchDog server did NOT send acknowledge signal
				wait_cnt--; // Decrement number of un-initialised servers
			} // case	c_pwm
			break;

			default :
				acquire_lock(); printint(motor_s.id);	release_lock();
				motor_s.tymer :> ts1;
				motor_s.tymer when timerafter(ts1 + MILLI_SEC) :> void;
			break; // default
		} // select
	} // while(0 < run_cnt)

	acquire_lock(); printstrln("");	release_lock();

} // wait_for_servers_to_start
/*****************************************************************************/
static void find_hall_origin( // Test Hall-state for origin
	MOTOR_DATA_TYP &motor_s // Reference to structure containing motor data
) // Returns new motor-state
/* The input pins from the Hall port hold the following data
 * Bit_3: Over-current flag (NB Value zero is over-current)
 * Bit_2: Hall Sensor Phase_C
 * Bit_1: Hall Sensor Phase_B
 * Bit_0: Hall Sensor Phase_A
 *
 * The Sensor bits are toggled every 180 degrees. 
 * Each phase is separated by 120 degrees.
 * WARNING: By Convention Phase_A is the MS-bit. 
 * However on the XMOS Motor boards, Phase_A is the LS-bit
 * This gives the following valid bit patterns for [CBA]
 * 
 *      <-------- Positive Spin <--------
 * (011)  001  101  100  110  010  011  (001)   [CBA]
 *      --------> Negative Spin -------->
 * 
 * WARNING: By Convention Phase_A leads Phase B for a positive spin.
 * However, each motor manufacturer may use their own definition for spin direction.
 * So key Hall-states are implemented as defines e.g. FIRST_HALL and LAST_HALL
 *
 * For the purposes of this algorithm, the angular position origin is defined as
 * the transition from the last-state to the first-state.
 */
{
	unsigned hall_inp = (motor_s.hall_params.hall_val & HALL_PHASE_MASK); // Mask out 3 Hall Sensor Phase Bits


	// Check for change in Hall state
	if (motor_s.prev_hall != hall_inp)
	{
		// Check for 1st Hall state, as we only do this check once a revolution
		if (hall_inp == FIRST_HALL_STATE) 
		{
			// Check for correct spin direction
			if (motor_s.prev_hall == motor_s.end_hall)
			{ // Spinning in correct direction

				// Check if the angular origin has been found, AND, we have done more than one revolution
				if (1 < abs(motor_s.qei_params.rev_cnt))
				{
					/* Calculate the offset between arbitary PWM set_theta and actual measured theta,
					 * NB There are multiple values of set_theta that can be used for each meas_theta, 
           * depending on the number of pole pairs. E.g. [0, 256, 512, 768] are equivalent.
					 */
					motor_s.hall_offset = motor_s.set_theta;
					motor_s.hall_found = 1; // Set flag indicating Hall origin located
				} // if (0 < motor_s.qei_params.rev_cnt)
			} // if (motor_s.prev_hall == motor_s.end_hall)
		} // if (hall_inp == FIRST_HALL_STATE)

		motor_s.prev_hall = hall_inp; // Store hall state for next iteration
	} // if (motor_s.prev_hall != hall_inp)

} // find_hall_origin
/*****************************************************************************/
static void find_qei_origin( // Test QEI state for origin
	MOTOR_DATA_TYP &motor_s // Reference to structure containing motor data
) // Returns new motor-state
{
	if (motor_s.qei_params.calib) // Check if angular position origin found
	{
 		// Check if we finished SEARCH state
		if (SEARCH != motor_s.state)
		{ // Re-calculate QEI offset, now QEI theta has done origin reset
			motor_s.qei_offset += (motor_s.qei_params.old_ang - motor_s.tot_ang); // Add back old QEI, & subtract new
		} // if (FOC == motor_s.state)

		motor_s.qei_found = 1; // Set flag indicating QEI origin located
	} // if (motor_s.qei_params.calib)

} // find_qei_origin
/*****************************************************************************/
#pragma unsafe arrays
static void collect_sensor_data( // Collect sensor data and update motor state if necessary
	MOTOR_DATA_TYP &motor_s, // reference to structure containing motor data
	chanend c_pwm, 
	streaming chanend c_hall, 
	streaming chanend c_qei, 
	streaming chanend c_adc_cntrl
)
{
	foc_hall_get_parameters( motor_s.hall_params ,c_hall ); // Get new hall state

	// Check error status
	if (motor_s.hall_params.err)
	{
		motor_s.err_data.err_flgs |= (1 << OVERCURRENT_ERR);
		motor_s.err_data.line[OVERCURRENT_ERR] = __LINE__;
		motor_s.state = STOP; // Switch to stop state
		motor_s.cnts[STOP] = 0; // Initialise stop-state counter 

		return;
	} // if (motor_s.hall_params.err)

	if (0 == motor_s.hall_found)
	{
		find_hall_origin( motor_s );
	} // if (0 == motor_s.hall_found)

	/* Get the position from encoder module. NB returns rev_cnt=0 at start-up  */
	foc_qei_get_parameters( motor_s.qei_params ,c_qei );
// if (motor_s.xscope) xscope_int( (4+motor_s.id) ,motor_s.qei_params.veloc ); // MB~
// if (motor_s.xscope) xscope_int( (6+motor_s.id) ,motor_s.qei_params.theta ); //MB~

#if (1 == VELOC_FILT) 
	// Filter velocity. MB~ Need to investigate why velocity spikes occur

	if (motor_s.qei_params.veloc < (motor_s.prev_veloc - MAX_VELOC_INC))
	{
		motor_s.qei_params.veloc = (motor_s.prev_veloc - MAX_VELOC_INC);
	} // if (motor_s.qei_params.veloc < (motor_s.prev_veloc - MAX_VELOC_INC))
	else
	{
		if (motor_s.qei_params.veloc > (motor_s.prev_veloc + MAX_VELOC_INC))
		{
			motor_s.qei_params.veloc = (motor_s.prev_veloc + MAX_VELOC_INC);
		} // if (motor_s.qei_params.veloc < (motor_s.prev_veloc + MAX_VELOC_INC))
	} // else !(motor_s.qei_params.veloc < (motor_s.prev_veloc -MAX_VELOC_INC ))

	motor_s.prev_veloc = motor_s.qei_params.veloc; // Update previous velocity
#endif // VELOC_FILT

	motor_s.tot_ang = motor_s.qei_params.rev_cnt * QEI_PER_REV + motor_s.qei_params.theta;
#if (LDO_MOTOR_SPIN)
{	// NB The QEI sensor on the LDO motor is inverted with respect to the other sensors
motor_s.qei_params.old_ang = -motor_s.qei_params.old_ang; // Invert velocity
motor_s.qei_params.veloc = -motor_s.qei_params.veloc; // Invert old angle
motor_s.tot_ang = -motor_s.tot_ang; // Invert total angle

// Re-calculate rev_cnt and theta
motor_s.qei_params.rev_cnt = (motor_s.tot_ang + motor_s.half_qei) >> QEI_RES_BITS; 
motor_s.qei_params.theta = motor_s.tot_ang - (motor_s.qei_params.rev_cnt << QEI_RES_BITS); 
}
#endif // (LDO_MOTOR_SPIN)

if (motor_s.xscope) xscope_int( (4+motor_s.id) ,motor_s.qei_params.veloc ); // MB~
if (motor_s.xscope) xscope_int( (6+motor_s.id) ,motor_s.qei_params.theta ); // MB~

	motor_s.meas_speed = abs( motor_s.qei_params.veloc ); // NB Used to spot stalling behaviour

	if (SAFE_MAX_SPEED < motor_s.meas_speed) // Safety
	{
		printstr("AngVel:"); printintln( motor_s.qei_params.veloc );

		motor_s.err_data.err_flgs |= (1 << SPEED_ERR);
		motor_s.err_data.line[SPEED_ERR] = __LINE__;
		motor_s.state = STOP; // Switch to stop state
		motor_s.cnts[STOP] = 0; // Initialise stop-state counter 

		return;
	} // if (4100 < motor_s.qei_params.veloc)

	if (0 == motor_s.qei_found)
	{
		find_qei_origin( motor_s );
	} // if (0 == motor_s.qei_found)


motor_s.dbg_tmr :> motor_s.dbg_strt; //MB~
	update_motor_state( motor_s );
motor_s.dbg_tmr :> motor_s.dbg_end; //MB~

	// Convert new set DQ values to PWM values
// if (motor_s.xscope) xscope_int( 6 ,motor_s.vect_data[D_ROTA].set_V  );
// if (motor_s.xscope) xscope_int( 4 ,motor_s.vect_data[Q_ROTA].set_V );
	dq_to_pwm( motor_s ); // Convert Output DQ values to PWM values

	foc_pwm_put_parameters( motor_s.pwm_comms ,c_pwm ); // Update the PWM values

	// Get ADC sensor data here, in gap between PWM trigger and ADC capture
	foc_adc_get_parameters( motor_s.adc_params ,c_adc_cntrl );

#ifdef MB
	unsigned dbg_diff = (unsigned)(motor_s.dbg_end - motor_s.dbg_strt);
	int dbg_inc = (int)dbg_diff - (int)motor_s.dbg_sum + motor_s.dbg_err;

	int dbg_filt = (dbg_inc + 128) >> 8;
	motor_s.dbg_err = dbg_inc - (dbg_filt << 8);

	motor_s.dbg_sum += dbg_filt;

//	if (motor_s.id) xscope_int( (8+motor_s.id) ,motor_s.dbg_sum ); //MB~
#endif //MB~

} // collect_sensor_data
/*****************************************************************************/
#pragma unsafe arrays
static void use_motor ( // Start motor, and run step through different motor states
	MOTOR_DATA_TYP &motor_s, // reference to structure containing motor data
	chanend c_pwm, 
	streaming chanend c_hall, 
	streaming chanend c_qei, 
	streaming chanend c_adc_cntrl, 
	chanend c_speed, 
	chanend c_commands,
	chanend c_wd
)
{
	unsigned command;	// Command received from the control interface


	motor_s.tymer :> motor_s.prev_pwm_time; // Store time-stamp

motor_s.dbg_tmr :> motor_s.dbg_orig; // MB~

	/* Main loop */
	while (STOP != motor_s.state)
	{
#pragma xta endpoint "foc_loop"
		select
		{
		case c_speed :> command:		/* This case responds to speed control through shared I/O */
#pragma xta label "foc_loop_speed_comms"
			switch(command)
			{
				case IO_CMD_GET_IQ :
					c_speed <: motor_s.qei_params.veloc;
					c_speed <: motor_s.req_veloc;
				break; // case IO_CMD_GET_IQ
	
				case IO_CMD_SET_SPEED :
					c_speed :> motor_s.req_veloc;
					motor_s.half_veloc = (motor_s.req_veloc >> 1);
				break; // case IO_CMD_SET_SPEED 
	
				case IO_CMD_GET_FAULT :
					c_speed <: motor_s.err_data.err_flgs;
				break; // case IO_CMD_GET_FAULT 
	
		    default: // Unsupported
					assert(0 == 1); // command NOT supported
		    break; // default
			} // switch(command)

		break; // case c_speed :> command:

		case c_commands :> command:		//This case responds to CAN or ETHERNET commands
#pragma xta label "foc_loop_shared_comms"
			if(command == IO_CMD_GET_VALS)
			{
				c_commands <: motor_s.qei_params.veloc;
				c_commands <: motor_s.adc_params.vals[ADC_PHASE_A];
				c_commands <: motor_s.adc_params.vals[ADC_PHASE_B];
			}
			else if(command == IO_CMD_GET_VALS2)
			{
				c_commands <: motor_s.adc_params.vals[ADC_PHASE_C];
				c_commands <: motor_s.pid_veloc;
				c_commands <: motor_s.pid_Id;
				c_commands <: motor_s.pid_Iq;
			}
			else if (command == IO_CMD_SET_SPEED)
			{
				c_commands :> motor_s.req_veloc;
				motor_s.half_veloc = (motor_s.req_veloc >> 1);
			}
			else if (command == IO_CMD_GET_FAULT)
			{
				c_commands <: motor_s.err_data.err_flgs;
			}

		break; // case c_commands :> command:

		default:	// This case updates the motor state
// xscope_int( (8+motor_s.id) ,(8+motor_s.id) );
			motor_s.iters++; // Increment No. of iterations 

			// NB There is not enough band-width to probe all xscope data
			if (motor_s.iters & 7) // probe every 8th value
			{
				motor_s.xscope = 0; // Switch OFF xscope probe
			} // if ((motor_s.id) & !(motor_s.iters & 7))
			else
			{
				motor_s.xscope = 1; // Switch ON xscope probe
			} // if ((motor_s.id) & !(motor_s.iters & 7))
// motor_s.xscope = 1; // MB~ Crude Switch

			collect_sensor_data( motor_s ,c_pwm ,c_hall ,c_qei ,c_adc_cntrl );

			// Check if it is time to stop demo
			if (motor_s.iters > DEMO_LIMIT)
			{
				motor_s.state = STOP; // Switch to stop state
				motor_s.cnts[STOP] = 0; // Initialise stop-state counter 
			} // if (motor_s.iters > DEMO_LIMIT)
		break; // default:

		}	// select

		c_wd <: WD_CMD_TICK; // Keep WatchDog alive
	}	// while (STOP != motor_s.state)

	// Set PWM values to stop motor
	error_pwm_values( motor_s.pwm_comms.params.widths );

	foc_pwm_put_parameters( motor_s.pwm_comms ,c_pwm ); // Update the PWM with 'stop values'
} // use_motor
/*****************************************************************************/
void signal_servers_to_stop( // Signal then wait for other servers to terminate
	MOTOR_DATA_TYP &motor_s, // Structure containing motor data
	chanend c_wd, // Channel for communication with WatchDog server
	chanend c_pwm, // Channel for communication with PWM server
	streaming chanend c_hall, // Channel for communication with Hall server 
	streaming chanend c_qei,  // Channel for communication with QEI server
	streaming chanend c_adc_cntrl // Channel for communication with ADC server
)
#define NUM_STOP_SERVERS 4 // WARNING: edit this line to indicate No of servers to close 
{
	unsigned cmd; // Command from Server
	unsigned ts1;	// timestamp
	int run_cnt = NUM_STOP_SERVERS; // Initialise number of running servers


	// Signal other processes to stop ...

	c_pwm <: PWM_CMD_LOOP_STOP; // Stop PWM server
	c_adc_cntrl <: ADC_CMD_LOOP_STOP; // Stop ADC server
	c_qei <: QEI_CMD_LOOP_STOP; // Stop QEI server
	c_hall <: HALL_CMD_LOOP_STOP; // Stop Hall server

	// Loop until no more servers still running
	while(0 < run_cnt)
	{
		select {
			case c_qei :> cmd : // QEI server
			{
				// If Shutdown acknowledged, decrement No. of running servers
				if (QEI_CMD_ACK == cmd) run_cnt--;
			} // case	c_qei
			break;

			case c_hall :> cmd : // Hall server
			{
				// If Shutdown acknowledged, decrement No. of running servers
				if (HALL_CMD_ACK == cmd) run_cnt--;
			} // case	c_hall
			break;

			case c_adc_cntrl :> cmd : // ADC server
			{
				// If Shutdown acknowledged, decrement No. of running servers
				if (ADC_CMD_ACK == cmd) run_cnt--;
			} // case	c_adc_cntrl
			break;

			case c_pwm :> cmd : // PWM server
			{
				// If Shutdown acknowledged, decrement No. of running servers
				if (PWM_CMD_ACK == cmd) run_cnt--;
			} // case	c_pwm
			break;

			default :
				acquire_lock(); printint(motor_s.id);	release_lock();
				motor_s.tymer :> ts1;
				motor_s.tymer when timerafter(ts1 + MILLI_SEC) :> void;
			break; // default
		} // select
	} // while(0 < run_cnt)

	acquire_lock(); printstrln("");	release_lock();
} // signal_servers_to_stop
/*****************************************************************************/
static void error_handling( // Prints out error messages
	ERR_DATA_TYP &err_data_s // Reference to structure containing data for error-handling
)
{
	int err_cnt; // counter for different error types 
	unsigned cur_flgs = err_data_s.err_flgs; // local copy of error flags


	// Loop through error types
	for (err_cnt=0; err_cnt<NUM_ERR_TYPS; err_cnt++)
	{
		// Test LS-bit for active flag
		if (cur_flgs & 1)
		{
			printstr( "Line " );
			printint( err_data_s.line[err_cnt] );
			printstr( ": " );
			printstrln( err_data_s.err_strs[err_cnt].str );
		} // if (cur_flgs & 1)

		cur_flgs >>= 1; // Discard flag
	} // for err_cnt

} // error_handling
/*****************************************************************************/
#pragma unsafe arrays
void run_motor ( 
	unsigned motor_id,
	chanend c_wd,
	chanend c_pwm,
	streaming chanend c_hall, 
	streaming chanend c_qei, 
	streaming chanend c_adc_cntrl, 
	chanend c_speed, 
	chanend c_commands 
)
{
	MOTOR_DATA_TYP motor_s; // Structure containing motor data
	unsigned ts1;	/* timestamp */


	acquire_lock(); 
	printstr("                      Motor_");
	printint(motor_id);
	printstrln(" Starts");
	release_lock();

	// Allow the rest of the system to settle
	motor_s.tymer :> ts1;
	motor_s.tymer when timerafter(ts1 + (MILLI_400_SECS << 1)) :> ts1;

	init_motor( motor_s ,motor_id );	// Initialise motor data

	init_pwm( motor_s.pwm_comms ,c_pwm ,motor_id );	// Initialise PWM communication data

	// Wait for other processes to start ...
	wait_for_servers_to_start( motor_s ,c_wd ,c_pwm ,c_hall ,c_qei ,c_adc_cntrl );

	acquire_lock(); 
	printstr("                      Motor_");
	printint(motor_id);
	printstrln(" Starts");
	release_lock();

	// Stagger the start of each motor 
motor_s.tymer when timerafter(ts1 + ((MICRO_SEC << 4) * motor_id)) :> ts1;
	motor_s.prev_pwm_time = ts1; // Store time_stamp

	// start-and-run motor
	use_motor( motor_s ,c_pwm ,c_hall ,c_qei ,c_adc_cntrl ,c_speed ,c_commands ,c_wd );

	// Closedown other processes ...
	signal_servers_to_stop( motor_s ,c_wd ,c_pwm ,c_hall ,c_qei ,c_adc_cntrl );

	// NB At present only Motor_1 works
	if (motor_id)
	{
		acquire_lock(); 
		if (motor_s.err_data.err_flgs)
		{
			printstr( "Demo Ended Due to Following Errors on Motor " );
			printintln(motor_s.id);
			error_handling( motor_s.err_data );
		} // if (motor_s.err_data.err_flgs)
		else
		{
			printstrln( "Demo Ended Normally" );
		} // else !(motor_s.err_data.err_flgs)
		release_lock();

		_Exit(1); // Exit without flushing buffers
	} // if (motor_id)

} // run_motor
/*****************************************************************************/
// inner_loop.xc
