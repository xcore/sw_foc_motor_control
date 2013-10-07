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
static void init_motor( // initialise data structure for one motor
	MOTOR_DATA_TYP &motor_s, // reference to structure containing motor data
	unsigned motor_id // Unique Motor identifier e.g. 0 or 1
)
{
	int phase_cnt; // phase counter
	int pid_cnt; // PID counter
	int tmp_val; // temporary manipulation value


	init_error_data( motor_s.err_data );


	// Initialise PID constants [ K_p ,K_i ,Kd, ,resolution ] ...

	init_pid_consts( motor_s.pid_consts[TRANSFORM][I_D] ,0 ,0 ,0 ,PID_RESOLUTION );
	init_pid_consts( motor_s.pid_consts[TRANSFORM][I_Q]		,580000 ,(384 << PID_RESOLUTION) ,0 ,PID_RESOLUTION );
	init_pid_consts( motor_s.pid_consts[TRANSFORM][SPEED]	,20000	,(4 << PID_RESOLUTION) ,0 ,PID_RESOLUTION );

	init_pid_consts( motor_s.pid_consts[EXTREMA][I_D] ,0 ,0 ,0 ,PID_RESOLUTION );
	init_pid_consts( motor_s.pid_consts[EXTREMA][I_Q]		,400000 ,(256 << PID_RESOLUTION)	,0 ,PID_RESOLUTION );
	init_pid_consts( motor_s.pid_consts[EXTREMA][SPEED]	,20000	,(3 << PID_RESOLUTION)		,0 ,PID_RESOLUTION );

	init_pid_consts( motor_s.pid_consts[VELOCITY][I_D] ,0 ,0 ,0 ,PID_RESOLUTION );
	init_pid_consts( motor_s.pid_consts[VELOCITY][I_Q]		,12000 ,(8 << PID_RESOLUTION) ,0 ,PID_RESOLUTION );
	init_pid_consts( motor_s.pid_consts[VELOCITY][SPEED]	,11200 ,(8 << PID_RESOLUTION) ,0 ,PID_RESOLUTION );

	// Initialise PID regulators
	for (pid_cnt = 0; pid_cnt < NUM_PIDS; pid_cnt++)
	{ 
		initialise_pid( motor_s.pid_regs[pid_cnt] );
	} // for pid_cnt
	
	for (phase_cnt = 0; phase_cnt < NUM_PWM_PHASES; phase_cnt++)
	{ 
		motor_s.adc_params.vals[phase_cnt] = -1;
	} // for phase_cnt

	motor_s.id = motor_id; // Unique Motor identifier e.g. 0 or 1
	motor_s.iters = 0;
	motor_s.cnts[SEARCH] = 0;
	motor_s.state = SEARCH;
	motor_s.hall_params.hall_val = HALL_NERR_MASK; // Initialise to 'No Errors'
	motor_s.prev_hall = (!HALL_NERR_MASK); // Arbitary value different to above

	motor_s.Iq_alg = TRANSFORM; // [TRANSFORM VELOCITY EXTREMA] Assign algorithm used to estimate coil current Iq (and Id)
	motor_s.first_foc = 1; // Set flag until first FOC iteration completed
	motor_s.hall_offset = 0;	// Phase difference between the Hall sensor origin and PWM theta origin
	motor_s.qei_offset = 0;	// Phase difference between the QEI origin and PWM theta origin
	motor_s.hall_found = 0;	// Set flag to Hall origin NOT found
	motor_s.qei_found = 0;	// Set flag to QEI origin NOT found
	motor_s.phi_err = 0; // Erro diffusion value for phi estimate
	motor_s.pid_Id = 0;	// Output from radial current PID
	motor_s.pid_Iq = 0;	// Output from tangential current PID
	motor_s.pid_veloc = 0;	// Output from velocity PID
	motor_s.set_Vd = 0;	// Ideal voltage producing radial magnetic field (NB never update as no radial force is required)
	motor_s.set_Vq = 0;	// Ideal voltage producing tangential magnetic field. (NB Updated based on the speed error)
	motor_s.prev_Vq = 0;	// Previous voltage producing tangential magnetic field.
	motor_s.est_Iq = 0;	// Clear Iq value estimated from measured angular velocity
	motor_s.xscope = 1; 	// Swithc on xscope print flag
	motor_s.prev_time = 0; 	// previous open-loop time stamp
	motor_s.coef_err = 0; // Clear Extrema Coef. diffusion error
	motor_s.scale_err = 0; // Clear Extrema Scaling diffusion error
	motor_s.Iq_err = 0; // Clear Error diffusion value for measured Iq
	motor_s.gamma_off = 0;	// Gamma value offset
	motor_s.gamma_err = 0;	// Error diffusion value for Gamma value
	motor_s.gamma_ramp = 0;	// Initialise Gamma ramp value to zero
//MB~	motor_s.gamma_est = 0;	// Estimate of leading-angle, used to 'pull' pole towards coil.
	motor_s.gamma_inc = 0; //MB~ tuning
	motor_s.phi_ramp = 0;	// Initialise Phi (phase lag) ramp value to zero

	motor_s.set_theta = 0; // PWM theta value
	motor_s.open_theta = 0; // Open-loop theta value
	motor_s.foc_theta = 0; // FOC theta value

	// Check consistency of pre-defined QEI values
	tmp_val = (1 << QEI_RES_BITS); // Build No. of QEI points from resolution bits
	assert( QEI_PER_REV == tmp_val );

	motor_s.half_qei = (QEI_PER_REV >> 1); // Half No. of QEI points per revolution

	motor_s.Vd_openloop = START_VD_OPENLOOP;
	motor_s.Vq_openloop = START_VQ_OPENLOOP;
	motor_s.update_period = OPEN_LOOP_PERIOD; // Time between updates to PWM theta

	motor_s.req_Vd = motor_s.Vd_openloop; // Requested 'radial' voltage 
	motor_s.req_Vq = motor_s.Vq_openloop; // Requested 'tangential' voltage 
	motor_s.prev_Vq = motor_s.Vq_openloop; // Preset previously used set_Vq value
	motor_s.filt_val = motor_s.Vq_openloop; // Preset filtered value to something sensible

//MB~	motor_s.req_veloc = REQ_VELOCITY;
	motor_s.req_veloc = REQ_VELOCITY;
	motor_s.half_veloc = (motor_s.req_veloc >> 1);

	// NB Display will require following variables, before we have measured them! ...
	motor_s.qei_params.veloc = motor_s.req_veloc;
	motor_s.meas_speed = abs(motor_s.req_veloc);

	// Initialise angle variables dependant on spin direction
	if (0 > motor_s.req_veloc)
	{ // Negative spin direction
		motor_s.gamma_off = -GAMMA_INTERCEPT;
		motor_s.phi_off = -PHI_INTERCEPT;

		motor_s.end_hall = NEGA_LAST_HALL_STATE; // Choose last Hall state of 6-state cycle
/*
motor_s.gamma_est = -38;	// MB~ Motor_00
motor_s.gamma_est = 108;	// MB~ Motor_01
motor_s.gamma_est = -16;	// MB~ Motor_02
motor_s.gamma_est = -44;	// MB~ Motor_03
motor_s.gamma_est = 100;	// MB~ Motor_04
motor_s.gamma_est = 114;	// MB~ Motor_05
motor_s.gamma_est = 60;	// MB~ Motor_06
motor_s.gamma_est = -83;	// MB~ Motor_07
motor_s.gamma_est = -46;	// MB~ Motor_08
motor_s.gamma_est = -62;	// MB~ Motor_09
motor_s.gamma_est = -49;	// MB~ Motor_10
motor_s.gamma_est = -38;	// MB~ Motor_11
*/
motor_s.gamma_est = 123;	// MB~ Motor_09
motor_s.gamma_est = -40;	// MB~ Motor_08
	} // if (0 > motor_s.req_veloc)
	else
	{ // Positive spin direction
		motor_s.gamma_off = GAMMA_INTERCEPT;
		motor_s.phi_off = PHI_INTERCEPT;

		motor_s.end_hall = POSI_LAST_HALL_STATE; // Choose last Hall state of 6-state cycle
/*
motor_s.gamma_est = 45;	// MB~ Motor_00
motor_s.gamma_est = 90;	// MB~ Motor_01
motor_s.gamma_est = 46;	// MB~ Motor_02
motor_s.gamma_est = 42;	// MB~ Motor_03
motor_s.gamma_est = 47;	// MB~ Motor_04
motor_s.gamma_est = 55;	// MB~ Motor_05
motor_s.gamma_est = 41;	// MB~ Motor_06
motor_s.gamma_est = -22;	// MB~ Motor_07
motor_s.gamma_est = -55;	// MB~ Motor_08
motor_s.gamma_est = 86;	// MB~ Motor_09
motor_s.gamma_est = 44;	// MB~ Motor_10
motor_s.gamma_est = -121;	// MB~ Motor_11
*/
motor_s.gamma_est = 44;	// MB~ Motor_09
motor_s.gamma_est = 40;	// MB~ Motor_08
	} // else !(0 > motor_s.req_veloc)

	motor_s.tmp = 0; // MB~ Dbg
	motor_s.temp = 0; // MB~ Dbg

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
	diff_val = scaled_inp - motor_s.filt_val;

	// Multiply difference by filter coefficient (alpha)
	diff_val += motor_s.coef_err; // Add in diffusion error;
	increment = (diff_val + XTR_HALF_COEF) >> XTR_COEF_BITS ; // Multiply by filter coef (with rounding)
	motor_s.coef_err = diff_val - (increment << XTR_COEF_BITS); // Evaluate new quantisation error value 

	motor_s.filt_val += increment; // Update (up-scaled) filtered output value

	// Update mean value by down-scaling filtered output value
	motor_s.filt_val += motor_s.scale_err; // Add in diffusion error;
	out_val = (motor_s.filt_val + XTR_HALF_SCALE) >> XTR_SCALE_BITS; // Down-scale
	motor_s.scale_err = motor_s.filt_val - (out_val << XTR_SCALE_BITS); // Evaluate new remainder value 

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

	motor_s.est_Iq = out_val;
	motor_s.est_Id = 0;
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
		motor_s.est_Iq = -sqrtuint( -scaled_vel );
	} // if (0 > motor_s.qei_params.veloc)
	else
	{
		motor_s.est_Iq = sqrtuint( scaled_vel );
	} // if (0 > motor_s.qei_params.veloc)
} // estimate_Iq_from_velocity
/*****************************************************************************/
static void estimate_Iq_using_transforms( // Calculate Id & Iq currents using transforms. NB Required if requested Id is NON-zero
	MOTOR_DATA_TYP &motor_s // reference to structure containing motor data
)
{
	int alpha_meas = 0, beta_meas = 0;	// Measured currents once transformed to a 2D vector
	int theta_park;	// Estimated theta value to get Max. Iq value from Park transform
	int scaled_phi;	// Scaled Phi offset
	int phi_est;	// Estimated phase difference between PWM and ADC sinusoids
	int smooth_phi;	// Smoothed Phi value (Leading angle)

#pragma xta label "foc_loop_clarke"

	// To calculate alpha and beta currents from measured data
// if (motor_s.xscope) xscope_probe_data( 4 ,motor_s.adc_params.vals[ADC_PHASE_A] );
// if (motor_s.xscope) xscope_probe_data( 5 ,motor_s.adc_params.vals[ADC_PHASE_B] );
// if (motor_s.xscope) xscope_probe_data( 6 ,motor_s.adc_params.vals[ADC_PHASE_C] );
	clarke_transform( motor_s.adc_params.vals[ADC_PHASE_A], motor_s.adc_params.vals[ADC_PHASE_B], motor_s.adc_params.vals[ADC_PHASE_C], alpha_meas, beta_meas );
// if (motor_s.xscope) xscope_probe_data( 5 ,alpha_meas);
// if (motor_s.xscope) xscope_probe_data( 6 ,beta_meas );

	// Update Phi estimate ...
	scaled_phi = motor_s.qei_params.veloc * PHI_GRAD + motor_s.phi_off + motor_s.phi_err;
	phi_est = (scaled_phi + HALF_PHASE) >> PHASE_BITS;
	motor_s.phi_err = scaled_phi - (phi_est << PHASE_BITS);

//MB~	smooth_phi = motor_s.qei_offset + phi_est;
	smooth_phi = motor_s.qei_offset;

	// Smooth changes in Phi estimate ...

	if (0 > motor_s.req_veloc)
	{ // Negative spin direction

		// If necessary, force a smooth change
		if (motor_s.phi_ramp > phi_est)
		{
			motor_s.phi_ramp--; // Increment ramp value
			smooth_phi = motor_s.phi_ramp;
		} //if (motor_s.phi_ramp < phi_est)
	} // if (0 > motor_s.req_veloc)
	else
	{ // Positive spin direction

		// If necessary, force a smooth change
		if (motor_s.phi_ramp < phi_est)
		{
			motor_s.phi_ramp++; // Increment ramp value
			smooth_phi = motor_s.phi_ramp;
		} //if (motor_s.phi_ramp < phi_est)
	} // else !(0 > motor_s.req_veloc)

	// Calculate theta value for Park transform
	theta_park = motor_s.qei_params.theta + smooth_phi;
	theta_park &= QEI_REV_MASK; // Convert to base-range [0..QEI_REV_MASK]

#pragma xta label "foc_loop_park"

	// Estimate coil currents (Id & Iq) using park transform
	park_transform( motor_s.est_Id ,motor_s.est_Iq ,alpha_meas ,beta_meas ,theta_park );
// if (motor_s.xscope) xscope_probe_data( 5 ,motor_s.est_Id );
// if (motor_s.xscope) xscope_probe_data( 6 ,motor_s.est_Iq );

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
static void dq_to_pwm ( // Convert Id & Iq input values to 3 PWM output values 
	MOTOR_DATA_TYP &motor_s, // Reference to structure containing motor data
	int set_Vd, // Demand Radial voltage from the Voltage control PIDs
	int set_Vq, // Demand tangential voltage from the Voltage control PIDs
	unsigned inp_theta	// Input demand theta
)
{
	int volts[NUM_PWM_PHASES];	// array of intermediate demand voltages for each phase
	int alpha_set = 0, beta_set = 0; // New intermediate demand voltages as a 2D vector
	int phase_cnt; // phase counter


	// Smooth Vq values
	if (set_Vq > motor_s.prev_Vq)
	{
		set_Vq = motor_s.prev_Vq + 1; // Allow small increase
	} // if (set_Vq > motor_s.prev_Vq)
	else
	{
		if (set_Vq < motor_s.prev_Vq)
		{
			set_Vq = motor_s.prev_Vq - 1; // Allow small decrease
		} // if 
	} // else !(set_Vq > motor_s.prev_Vq)

	motor_s.prev_Vq = set_Vq; // Store smoothed Vq value

	// Inverse park  [d, q, theta] --> [alpha, beta]
// if (motor_s.xscope) xscope_probe_data( 4 ,set_Vq );
	inverse_park_transform( alpha_set, beta_set, set_Vd, set_Vq, inp_theta  );

	// Final voltages applied: 
	inverse_clarke_transform( volts[PWM_PHASE_A] ,volts[PWM_PHASE_B] ,volts[PWM_PHASE_C] ,alpha_set ,beta_set ); // Correct order

if (motor_s.xscope) xscope_probe_data( 5 ,volts[0] );
if (motor_s.xscope) xscope_probe_data( 6 ,inp_theta );

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
	timer chronometer; // timer
	unsigned cur_time; // current time value
	int diff_time; // time since last theta update


	motor_s.set_Vd = motor_s.Vd_openloop;
	motor_s.set_Vq = motor_s.Vq_openloop;

	chronometer :> cur_time;
	diff_time = (int)(cur_time - motor_s.prev_time); 

	// Check if theta needs incrementing
	if (motor_s.update_period < diff_time)
	{
		if (motor_s.req_veloc < 0)
		{
			motor_s.open_theta--; // Decrement demand theta value
		} // if (motor_s.req_veloc < 0)
		else
		{
			motor_s.open_theta++; // Increment demand theta value
		} // else !(motor_s.req_veloc < 0)

		motor_s.prev_time = cur_time; // Update previous time
	} // if (motor_s.update_period < diff_time)

	// NB QEI_REV_MASK correctly maps -ve values into +ve range 0 <= theta < QEI_PER_REV;
	motor_s.set_theta = motor_s.open_theta & QEI_REV_MASK; // Convert to base-range [0..QEI_REV_MASK]

	motor_s.set_theta &= QEI_REV_MASK; // Convert to base-range [0..QEI_REV_MASK]
} // calc_open_loop_pwm
/*****************************************************************************/
static void calc_transit_pwm( // Calculate FOC PWM output values
	MOTOR_DATA_TYP &motor_s // reference to structure containing motor data
)
{
	timer chronometer; // timer
	unsigned cur_time; // current time value
	int diff_time; // time since last theta update
	int weight;	// Weighting for FOC theta
	int tot_ang;	// Total angle traversed


#pragma xta label "trans_loop_speed_pid"

	chronometer :> cur_time;
	diff_time = (int)(cur_time - motor_s.prev_time); 

	// Check if theta needs incrementing
	if (motor_s.update_period < diff_time)
	{
		if (motor_s.req_veloc < 0)
		{
			motor_s.open_theta--; // Decrement demand theta value
		} // if (motor_s.req_veloc < 0)
		else
		{
			motor_s.open_theta++; // Increment demand theta value
		} // else !(motor_s.req_veloc < 0)

		motor_s.prev_time = cur_time; // Update previous time
	} // if (motor_s.update_period < diff_time)

	// WARNING: Do NOT allow  motor_s.foc_theta or motor_s.open_theta to wrap otherwise weighting will NOT work

	tot_ang = motor_s.qei_params.theta + (QEI_PER_REV * motor_s.qei_params.rev_cnt); // Total angle traversed
	motor_s.foc_theta = tot_ang + motor_s.qei_offset + motor_s.gamma_est;
if (motor_s.xscope) xscope_probe_data( 4 ,motor_s.foc_theta );

	weight = abs(motor_s.open_theta) - QEI_PER_PAIR; // Weighting for FOC. NB [0 --> QEI_PER_PAIR]

	// NB Round and divide by QEI_PER_PAIR)   
	motor_s.set_theta = (weight * (motor_s.foc_theta - motor_s.open_theta) + (QEI_PER_PAIR >> 1)) >> 8;

	motor_s.set_theta += motor_s.open_theta;

if (motor_s.temp)
{
acquire_lock();
printint(motor_s.temp); 
printstr(": Qtheta="); printint(motor_s.qei_params.theta );
printstr(" Qrevs="); printint(motor_s.qei_params.rev_cnt );
printstr(" Qang="); printintln(tot_ang);
printstr(" Qoff="); printint(motor_s.qei_offset );
printstr(" Gamma="); printint(motor_s.gamma_est );
printstr(" Open="); printint(motor_s.open_theta); 
printstr(" FOC="); printint(motor_s.foc_theta); 
printstr(" Set="); printint(motor_s.set_theta);
printstr(" Wrap="); printintln(motor_s.set_theta & QEI_REV_MASK); 
release_lock(); //MB~

motor_s.temp--;
}

	motor_s.set_theta &= QEI_REV_MASK; // Convert to base-range [0..QEI_REV_MASK]
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
	int targ_Iq;	// target measured Iq (scaled down requested Iq value)
	int corr_Id;	// Correction to radial current value
	int corr_Iq;	// Correction to tangential current value
	int corr_veloc;	// Correction to angular velocity
	int scaled_phase;	// Scaled Phase offset
	int smooth_gamma;	// Smoothed Gamma value (Leading angle)


#pragma xta label "foc_loop_speed_pid"

	// Applying Speed PID.

	// Check if PID's need presetting
	if (motor_s.first_foc)
	{
		preset_pid( motor_s.id ,motor_s.pid_regs[SPEED] ,motor_s.pid_consts[motor_s.Iq_alg][SPEED] ,motor_s.qei_params.veloc ,motor_s.req_veloc ,motor_s.qei_params.veloc );
	}; // if (motor_s.first_foc)

	corr_veloc = get_pid_regulator_correction( motor_s.id ,motor_s.pid_regs[SPEED] ,motor_s.pid_consts[motor_s.Iq_alg][SPEED] ,motor_s.req_veloc ,motor_s.qei_params.veloc );

	// Calculate velocity PID output
	if (PROPORTIONAL)
	{ // Proportional update
		motor_s.pid_veloc = corr_veloc;
	} // if (PROPORTIONAL)
	else
	{ // Offset update
		motor_s.pid_veloc = motor_s.req_Vq + corr_veloc;
	} // else !(PROPORTIONAL)

	if (VELOC_CLOSED)
	{ // Evaluate set IQ from velocity PID
		motor_s.req_Vq = motor_s.pid_veloc;
	} // if (VELOC_CLOSED)
	else
	{ 
		motor_s.req_Vq = motor_s.Vq_openloop;
	} // if (VELOC_CLOSED)

#pragma xta label "foc_loop_id_iq_pid"

	// Select algorithm for estimateing coil current Iq (and Id)
	// WARNING: Changing algorithm will require re-tuning of PID values
	switch ( motor_s.Iq_alg )
	{
		case TRANSFORM : // Use Park/Clarke transforms
			estimate_Iq_using_transforms( motor_s );
			targ_Iq = (motor_s.req_Vq + 16) >> 5; // Scale requested value to be of same order as estimated value
		break; // case TRANSFORM
	
		case EXTREMA : // Use Extrema of measured ADC values
			estimate_Iq_from_ADC_extrema( motor_s );
			targ_Iq = (motor_s.req_Vq + 16) >> 5; // Scale requested value to be of same order as estimated value
		break; // case EXTREMA 
	
		case VELOCITY : // Use measured velocity
			estimate_Iq_from_velocity( motor_s );
			targ_Iq = motor_s.req_Vq; // NB No scaling required
		break; // case VELOCITY 
	
    default: // Unsupported
			printstr("ERROR: Unsupported Iq Estimation algorithm > "); 
			printintln( motor_s.Iq_alg );
			assert(0 == 1);
    break;
	} // switch (motor_s.Iq_alg)

	// Apply PID control to Iq and Id

	// Check if PID's need presetting
	if (motor_s.first_foc)
	{
		preset_pid( motor_s.id ,motor_s.pid_regs[I_Q] ,motor_s.pid_consts[motor_s.Iq_alg][I_Q] ,motor_s.Vq_openloop ,targ_Iq ,motor_s.est_Iq );
		preset_pid( motor_s.id ,motor_s.pid_regs[I_D] ,motor_s.pid_consts[motor_s.Iq_alg][I_D] ,motor_s.Vd_openloop ,motor_s.req_Vd ,motor_s.est_Id );
	}; // if (motor_s.first_foc)

	corr_Iq = get_pid_regulator_correction( motor_s.id ,motor_s.pid_regs[I_Q] ,motor_s.pid_consts[motor_s.Iq_alg][I_Q] ,targ_Iq ,motor_s.est_Iq );
	corr_Id = get_pid_regulator_correction( motor_s.id ,motor_s.pid_regs[I_D] ,motor_s.pid_consts[motor_s.Iq_alg][I_D] ,motor_s.req_Vd ,motor_s.est_Id );

	if (PROPORTIONAL)
	{ // Proportional update
		motor_s.pid_Id = corr_Id;
		motor_s.pid_Iq = corr_Iq;
	} // if (PROPORTIONAL)
	else
	{ // Offset update
		motor_s.pid_Id = motor_s.set_Vd + corr_Id;
		motor_s.pid_Iq = motor_s.set_Vq + corr_Iq;
	} // else !(PROPORTIONAL)

	if (IQ_ID_CLOSED)
	{ // Update set DQ values
		motor_s.set_Vd = motor_s.pid_Id;
		motor_s.set_Vq = motor_s.pid_Iq;
	} // if (IQ_ID_CLOSED)
	else
	{ // Open-loop
		// If necessary, smoothly change open-loop Vq to requested value. NB REQ_VQ_OPENLOOP always +ve 
		if (motor_s.Vq_openloop > REQ_VQ_OPENLOOP )
		{
			motor_s.Vq_openloop -= 1; // Decrease slightly
		} // if (motor_s.Vq_openloop > REQ_VQ_OPENLOOP )
		else
		{
			if (motor_s.Vq_openloop < REQ_VQ_OPENLOOP )
			{
				motor_s.Vq_openloop += 1; // Increase slightly
			} // if (motor_s.Vq_openloop > REQ_VQ_OPENLOOP )
		} // else !(motor_s.Vq_openloop > REQ_VQ_OPENLOOP )

		calc_open_loop_pwm( motor_s );
	} // else !(IQ_ID_CLOSED)

	// Update Gamma estimate ...
#ifdef MB
//MB~	scaled_phase = motor_s.qei_params.veloc * GAMMA_GRAD + motor_s.gamma_off + motor_s.gamma_err;
	scaled_phase = 50 * GAMMA_GRAD + motor_s.gamma_off + motor_s.gamma_err;
	motor_s.gamma_est = (scaled_phase + HALF_PHASE) >> PHASE_BITS;
	motor_s.gamma_err = scaled_phase - (motor_s.gamma_est << PHASE_BITS);
#endif //MB~

motor_s.tmp++;
if ((1 << 10) == motor_s.tmp)
{
	motor_s.tmp = 0;
	motor_s.gamma_est++;
	if (QEI_PER_PAIR == motor_s.gamma_est)
	{
		motor_s.gamma_est = 0;
		motor_s.gamma_ramp -= QEI_PER_PAIR; 
	}  // if (QEI_PER_PAIR == motor_s.gamma_est)
} // if (1024 == motor_s.tmp)
// if (motor_s.xscope) xscope_probe_data( 2 ,motor_s.gamma_est);

	if (motor_s.gamma_ramp > motor_s.gamma_est)
	{
		motor_s.gamma_ramp--; // Decrement ramp value
	} // if
	else
	{
		if (motor_s.gamma_ramp < motor_s.gamma_est)
		{
			motor_s.gamma_ramp++; // Increment ramp value
		} // if
	} // else
	smooth_gamma = motor_s.gamma_ramp;

#ifdef MB
	// Smooth changes in Gamma estimate ...
	smooth_gamma = motor_s.gamma_est;

	if (0 > motor_s.req_veloc)
	{ // Negative spin direction

		// If necessary, force a smooth change
		if (motor_s.gamma_ramp > motor_s.gamma_est)
		{
			motor_s.gamma_ramp--; // Decrement ramp value
			smooth_gamma = motor_s.gamma_ramp;
		} //if (motor_s.gamma_ramp < motor_s.gamma_est)
 	} // if (0 > motor_s.req_veloc)
	else
	{ // Positive spin direction

		// If necessary, force a smooth change
		if (motor_s.gamma_ramp < motor_s.gamma_est)
		{
			motor_s.gamma_ramp++; // Increment ramp value
			smooth_gamma = motor_s.gamma_ramp;
		} //if (motor_s.gamma_ramp < motor_s.gamma_est)
	} // else !(0 > motor_s.req_veloc)
#endif //MB~

	// Update 'demand' theta value for next dq_to_pwm iteration
//MB~ if (motor_s.xscope) xscope_probe_data( 4 ,smooth_gamma);
	motor_s.set_theta = motor_s.qei_params.theta + motor_s.qei_offset + motor_s.gamma_est;

	motor_s.set_theta &= QEI_REV_MASK; // Convert to base-range [0..QEI_REV_MASK]

	motor_s.first_foc = 0; // Clear 'first FOC' flag
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
	int tot_ang;	// Total angle traversed


	// Update motor state based on new sensor data
	switch( motor_s.state )
	{
		case SEARCH : // Turn motor using theta steps (open-loop), and update motor state
//MB~ acquire_lock(); printstr("Open="); printintln(motor_s.open_theta); release_lock(); //MB~
			// Check if one full electrical cycle completed
			if (QEI_PER_PAIR == abs(motor_s.open_theta))
			{ /* Calculate QEI offset
				 * In FOC state, theta will be based on angle returned from QEI sensor.
         * So recalibrate, to create smooth transition between SEARCH and FOC states.  
				 */
				tot_ang = motor_s.qei_params.theta + (QEI_PER_REV * motor_s.qei_params.rev_cnt); // Total QEI qngle
				motor_s.qei_offset = motor_s.open_theta - tot_ang; // Difference betweene Open-loop and QEI angle
acquire_lock();
printstr("TRANS Otheta="); printint(motor_s.open_theta);
printstr(" Qtheta="); printint(tot_ang);
printstr(" Ptheta="); printint(motor_s.set_theta);
printstr(" Qoff="); printintln(motor_s.qei_offset); 
release_lock(); //MB~
motor_s.temp = 1;
				motor_state = TRANSIT; 
				motor_s.state = motor_state; // NB Required due to XC compiler rules
			} // if (QEI_PER_PAIR == abs(motor_s.open_theta))
		break; // case SEARCH 
	
		case TRANSIT : // Transit between open-loop and FOC, and update motor state
			// Check if two full electrical cycles completed
			if ((QEI_PER_PAIR << 1) == abs(motor_s.open_theta))
			{
acquire_lock(); printstrln("FOC"); release_lock(); //MB~

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

if (motor_s.xscope) xscope_probe_data( 2 ,motor_s.qei_offset );
	motor_s.cnts[motor_s.state]++; // Update counter for new motor state 

	// Select correct method of calculating DQ values
#pragma fallthrough
	switch( motor_s.state )
	{
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
	MOTOR_DATA_TYP motor_s, // Structure containing motor data
	chanend c_wd, // Channel for communication with WatchDog server
	chanend c_pwm, // Channel for communication with PWM server
	streaming chanend c_hall, // Channel for communication with Hall server 
	streaming chanend c_qei,  // Channel for communication with QEI server
	streaming chanend c_adc_cntrl // Channel for communication with ADC server
)
#define NUM_INIT_SERVERS 4 // WARNING: edit this line to indicate No of servers to wait on
{
	timer chronometer;	// Timer
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
				acquire_lock(); printchar('.');	release_lock();
				chronometer :> ts1;
				chronometer when timerafter(ts1 + MILLI_SEC) :> void;
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
	int tot_ang;	// Total angle traversed


	if (motor_s.qei_params.calib) // Check if angular position origin found
	{
acquire_lock();
 		// Check if we finished SEARCH state
		if (SEARCH != motor_s.state)
		{ // Re-calculate QEI offset, now QEI theta has done origin reset
			tot_ang = (motor_s.qei_params.rev_cnt * QEI_PER_REV) + motor_s.qei_params.theta; // Total QEI angle
			motor_s.qei_offset += (motor_s.qei_params.old_ang - tot_ang); // Add back old QEI, & subtract new

printstr(" Old_Ang="); printint(motor_s.qei_params.old_ang);
printstr(" Qrevs="); printint(motor_s.qei_params.rev_cnt );
printstr(" Qtheta="); printint(motor_s.qei_params.theta );
printstr(" TotAng="); printint(tot_ang);
printstr(" Otheta="); printint(motor_s.open_theta);
printstr(" Ptheta="); printint(motor_s.set_theta);
printstr(" Qoff="); printint(motor_s.qei_offset);

motor_s.temp = 2;
		} // if (FOC == motor_s.state)

printstrln(" ORIGIN"); release_lock(); //MB~
		motor_s.qei_found = 1; // Set flag indicating QEI origin located
	} // if (motor_s.qei_params.calib)

} // find_qei_origin
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
	int tot_ang;	// Total angle traversed


	/* Main loop */
	while (STOP != motor_s.state)
	{
#pragma xta endpoint "foc_loop"
//MB~ if (motor_s.xscope) xscope_probe_data( 3 ,motor_s.state );
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
			motor_s.iters++; // Increment No. of iterations 

			// NB There is not enough band-width to probe all xscope data
			if ((motor_s.id) & !(motor_s.iters & 15)) // probe every 16th value
			{
				motor_s.xscope = 1; // Switch ON xscope probe
			} // if ((motor_s.id) & !(motor_s.iters & 7))
			else
			{
				motor_s.xscope = 0; // Switch OFF xscope probe
			} // if ((motor_s.id) & !(motor_s.iters & 7))
// motor_s.xscope = 1; // MB~ Crude Switch

			if (STOP != motor_s.state)
			{
				// Check if it is time to stop demo
				if (motor_s.iters > DEMO_LIMIT)
				{
					motor_s.state = STOP; // Switch to stop state
					motor_s.cnts[STOP] = 0; // Initialise stop-state counter 
				} // if (motor_s.iters > DEMO_LIMIT)

				foc_hall_get_parameters( motor_s.hall_params ,c_hall ); // Get new hall state

				// Check error status
				if (motor_s.hall_params.err)
				{
					motor_s.err_data.err_flgs |= (1 << OVERCURRENT_ERR);
					motor_s.err_data.line[OVERCURRENT_ERR] = __LINE__;
					motor_s.state = STOP; // Switch to stop state
					motor_s.cnts[STOP] = 0; // Initialise stop-state counter 
				} // if (motor_s.hall_params.err)
				else
				{
					if (0 == motor_s.hall_found)
					{
						find_hall_origin( motor_s );
					} // if (0 == motor_s.hall_found)

					/* Get the position from encoder module. NB returns rev_cnt=0 at start-up  */
					foc_qei_get_parameters( motor_s.qei_params ,c_qei );

					tot_ang = motor_s.qei_params.rev_cnt * QEI_PER_REV + motor_s.qei_params.theta;
#if (LDO_MOTOR_SPIN)
{	// NB The QEI sensor on the LDO motor is inverted with respect to the other sensors
	motor_s.qei_params.old_ang = -motor_s.qei_params.old_ang; // Invert velocity
	motor_s.qei_params.veloc = -motor_s.qei_params.veloc; // Invert old angle
	tot_ang = -tot_ang; // Invert total angle

	// Re-calculate rev_cnt and theta
	motor_s.qei_params.rev_cnt = (tot_ang + motor_s.half_qei) >> QEI_RES_BITS; 
	motor_s.qei_params.theta = tot_ang - (motor_s.qei_params.rev_cnt << QEI_RES_BITS); 

	assert( tot_ang == (motor_s.qei_params.rev_cnt * QEI_PER_REV + motor_s.qei_params.theta)); // MB~ Check angle
}
#endif // (LDO_MOTOR_SPIN)

if (1 < abs(motor_s.qei_params.rev_cnt))
{
	if (motor_s.xscope) xscope_probe_data( 0 ,motor_s.qei_params.theta );
}
else
{
	if (motor_s.xscope) xscope_probe_data( 0 ,tot_ang );
}
if (motor_s.xscope) xscope_probe_data( 1 ,motor_s.qei_params.veloc );
if (motor_s.xscope) xscope_probe_data( 3 ,motor_s.qei_params.rev_cnt );

					if (0 == motor_s.qei_found)
					{
						find_qei_origin( motor_s );
					} // if (0 == motor_s.qei_found)

					motor_s.meas_speed = abs( motor_s.qei_params.veloc ); // NB Used to spot stalling behaviour

					if (4400 < motor_s.meas_speed) // Safety
					{
						printstr("AngVel:"); printintln( motor_s.qei_params.veloc );

						motor_s.err_data.err_flgs |= (1 << SPEED_ERR);
						motor_s.err_data.line[SPEED_ERR] = __LINE__;
						motor_s.state = STOP; // Switch to stop state
						motor_s.cnts[STOP] = 0; // Initialise stop-state counter 
					} // if (4100 < motor_s.qei_params.veloc)

					// Get ADC sensor data
					foc_adc_get_parameters( motor_s.adc_params ,c_adc_cntrl );
// if (motor_s.xscope) xscope_probe_data( 5 ,motor_s.adc_params.vals[ADC_PHASE_A] );
// if (motor_s.xscope) xscope_probe_data( 6 ,(motor_s.adc_params.vals[ADC_PHASE_B] >> 4) );

					update_motor_state( motor_s );
				} // else !(motor_s.hall_params.err)

				// Check if motor needs stopping
				if (STOP == motor_s.state)
				{
					// Set PWM values to stop motor
					error_pwm_values( motor_s.pwm_comms.params.widths );
				} // if (STOP == motor_s.state)
				else
				{
					// Convert new set DQ values to PWM values
					dq_to_pwm( motor_s ,motor_s.set_Vd ,motor_s.set_Vq ,motor_s.set_theta ); // Convert Output DQ values to PWM values

					foc_pwm_put_parameters( motor_s.pwm_comms ,c_pwm ); // Update the PWM values

#ifdef USE_XSCOPE
					if ((motor_s.cnts[FOC] & 0x1) == 0) // If even, (NB Forgotton why this works!-(
					{
						if (0 == motor_s.id) // Check if 1st Motor
						{
/*
							xscope_probe_data(0, motor_s.qei_params.veloc );
				  	  xscope_probe_data(1, motor_s.set_Vq );
							xscope_probe_data(4, motor_s.adc_params.vals[ADC_PHASE_A] );
							xscope_probe_data(5, motor_s.adc_params.vals[ADC_PHASE_B]);
*/
						} // if (0 == motor_s.id)
					} // if ((motor_s.cnts[FOC] & 0x1) == 0) 
#endif
				} // else !(STOP == motor_s.state)
			} // if (STOP != motor_s.state)
		break; // default:

		}	// select

		c_wd <: WD_CMD_TICK; // Keep WatchDog alive

	}	// while (STOP != motor_s.state)
} // use_motor
/*****************************************************************************/
void signal_servers_to_stop( // Signal then wait for other servers to terminate
	MOTOR_DATA_TYP motor_s, // Structure containing motor data
	chanend c_wd, // Channel for communication with WatchDog server
	chanend c_pwm, // Channel for communication with PWM server
	streaming chanend c_hall, // Channel for communication with Hall server 
	streaming chanend c_qei,  // Channel for communication with QEI server
	streaming chanend c_adc_cntrl // Channel for communication with ADC server
)
#define NUM_STOP_SERVERS 4 // WARNING: edit this line to indicate No of servers to close 
{
	unsigned cmd; // Command from Server
	timer chronometer;	// Timer
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
				acquire_lock(); printchar('X');	release_lock();
				chronometer :> ts1;
				chronometer when timerafter(ts1 + MILLI_SEC) :> void;
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
	timer chronometer;	/* Timer */
	unsigned ts1;	/* timestamp */


	acquire_lock(); 
	printstr("                      Motor_");
	printint(motor_id);
	printstrln(" Starts");
	release_lock();

	// Allow the rest of the system to settle
	chronometer :> ts1;
	chronometer when timerafter(ts1 + (MILLI_400_SECS << 1)) :> ts1;

	// Stagger the start of each motor 
	chronometer when timerafter(ts1 + ((MICRO_SEC << 4) * motor_id)) :> ts1;

	init_motor( motor_s ,motor_id );	// Initialise motor data

	init_pwm( motor_s.pwm_comms ,c_pwm ,motor_id );	// Initialise PWM communication data

	// Wait for other processes to start ...
	wait_for_servers_to_start( motor_s ,c_wd ,c_pwm ,c_hall ,c_qei ,c_adc_cntrl );

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
