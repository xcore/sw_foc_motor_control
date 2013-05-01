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
 *
 *****************************************************************************
 *
 * During normal operation, the inner_loop() function loops through the following states:
 *  START: Initial entry state. 
 *		Move to SEARCH state.
 *
 *  SEARCH:	Motor runs open-loop, spinning the magnetic field around at a fixed torque,
 *		until the QEI reports that it has an accurate position measurement.
 *		Then the Hall sensors and QEI data are used to calculate the phase 
 *		difference between the motors coils and the QEI origin.
 *		Move to FOC state.
 *
 *  FOC:		Motor runs in closed-loop, the full Field Oriented Control (FOC)
 *		is used to commutate the rotor, until a 'stop' or 'stall' condition is detected.
 *		Move to STOP or STALL state.
 *
 *	STALL: Currently, error message is issued.
 *		Move to STOP state.
 *
 *	STOP: End loop
 *
 *	For each iteration of the FOC state, the following actions are performed:-
 *		Read the QEI and ADC data.
 *		Estimate the angular velocity from the QEI data. 
 *		Estimate the Iq value from either the ADC data or angular velocity.
 *		Optionally estimate the Id value from the ADC data
 *		Optionally use the demand and estimated velocity to produce a demand Iq value, using a PID
 *		Use the demand and estimated Iq to produce a requested Iq value, using a PID
 *  	Transform the requested Iq and Id values into voltages for the motor coils
 *  	Convert voltages into PWM duty cycles.
 *
 * This is a standard FOC algorithm, with the current and speed control loops combined.
 *
 * Notes:
 *
 *	The Leading_Angle is the angle between the position of maximum Iq 
 *	(tangential magnetic field), and the position of the magnetic pole.
 *	This allows the coil to be pulled towards the pole in an efficient manner.
 *
 *	The Phase_Lag, is the angular difference between the applied voltage (PWM),
 *	and the measured current (ADC).
 *
 *	Due to the effects of back EMF and inductance, both the Leading_Angle and 
 *	Phase_Lag vary with angular velocity.
 *
 **/                                   

#ifndef _INNER_LOOP_H_
#define _INNER_LOOP_H_

#include <stdlib.h> // Required for abs()

#include <xs1.h>
#include <print.h>
#include <assert.h>
#include <safestring.h>

#include "app_global.h"
#include "mathuint.h"
#include "hall_client.h"
#include "qei_client.h"
#include "adc_client.h"
#include "pwm_client.h"
#include "pid_regulator.h"
#include "clarke.h"
#include "park.h"
#include "watchdog.h"
#include "shared_io.h"

// Timing definitions
#define MILLI_400_SECS (400 * MILLI_SEC) // 400 ms. Start-up settling time
#define OPEN_LOOP_PERIOD (262 * MICRO_SEC) // 262us. Time between open-loop theta increments

#define PWM_MAX_LIMIT 3800
#define PWM_MIN_LIMIT 200
#define OFFSET_14 16383

#define STALL_SPEED 100
#define STALL_TRIP_COUNT 5000

#define LDO_MOTOR_SPIN 1 // Motor spins like an LDO Motor

#define FIRST_HALL_STATE 0b001 // 1st Hall state of 6-state cycle

#define INIT_THETA 0 // Initial start-up angle

#define REQ_VELOCITY 4000 // Initial start-up speed
#define START_VQ_OPENLOOP 2000 // Vq value for open-loop startup phase
#define START_VD_OPENLOOP 0		// Vd value for open-loop startup phase
#define REQ_VQ_OPENLOOP 6500 // Vq value for open-loop tuning
#define MIN_VQ 1600 // Motor will stall if abs(Vq) falls below this value

// Set-up defines for scaling ...
#define SHIFT_20 20
#define SHIFT_16 16
#define SHIFT_9   9

#define PHASE_BITS SHIFT_20 // No of bits in phase offset scaling factor 
#define PHASE_DENOM (1 << PHASE_BITS)
#define HALF_PHASE (PHASE_DENOM >> 1)

#define PHI_GRAD 11880 // 0.01133 as integer ratio PHI_GRAD/PHASE_DENOM
#define PHI_INTERCEPT 35693527 // 34.04 as integer ratio PHI_INTERCEPT/PHASE_DENOM

#define GAMMA_GRAD 7668 // 0.007313 as integer ratio GAMMA_GRAD/PHASE_DENOM
#define GAMMA_INTERCEPT 33019658 // 31.49 as integer ratio GAMMA_INTERCEPT/PHASE_DENOM

#define VEL_GRAD 10000 // (Estimated_Current)^2 = VEL_GRAD * Angular_Velocity

#define XTR_SCALE_BITS SHIFT_16 // Used to generate 2^n scaling factor
#define XTR_HALF_SCALE (1 << (XTR_SCALE_BITS - 1)) // Half Scaling factor (used in rounding)

#define XTR_COEF_BITS SHIFT_9 // Used to generate filter coef divisor. coef_div = 1/2^n
#define XTR_COEF_DIV (1 << XTR_COEF_BITS) // Coef divisor
#define XTR_HALF_COEF (XTR_COEF_DIV >> 1) // Half of Coef divisor

#define PROPORTIONAL 1 // Selects between 'proportional' and 'offset' error corrections
#define VELOC_CLOSED 1 // Selects fully closed loop (both velocity, Iq and Id)
#define IQ_ID_CLOSED 1 // Selcects Iq/Id closed-loop, velocity open-loop

#ifdef USE_XSCOPE
	#define DEMO_LIMIT 100000 // XSCOPE
#else // ifdef USE_XSCOPE
	#define DEMO_LIMIT 4000000
#endif // else !USE_XSCOPE

#define STR_LEN 80 // String Length

#pragma xta command "add exclusion foc_loop_motor_fault"
#pragma xta command "add exclusion foc_loop_speed_comms"
#pragma xta command "add exclusion foc_loop_shared_comms"
#pragma xta command "add exclusion foc_loop_startup"
#pragma xta command "analyze loop foc_loop"
#pragma xta command "set required - 40 us"

/** Different Estimation algorithms for coil currents Iq (and Id)*/
typedef enum IQ_EST_TAG
{
  TRANSFORM = 0,	// Uses Park/Clarke transforms on measured ADC coil currents
  EXTREMA,				// Uses Extrema of measured ADC coil currents
  VELOCITY,				// Uses measured velocity
  NUM_IQ_ESTIMATES    // Handy Value!-)
} IQ_EST_TYP;

/** Different Motor Phases */
typedef enum MOTOR_STATE_ETAG
{
  START = 0,	// Initial entry state
  SEARCH,		// Turn motor until FOC start condition found
  FOC,		  // Normal FOC state
	STALL,		// state where motor stalled
	STOP,			// Error state where motor stopped
  NUM_MOTOR_STATES	// Handy Value!-)
} MOTOR_STATE_ENUM;

// WARNING: If altering Error types. Also update error-message in init_error_data()
/** Different Motor Phases */
typedef enum ERROR_ETAG
{
	OVERCURRENT = 0,
	UNDERVOLTAGE,
	STALLED,
	DIRECTION,
  NUM_ERR_TYPS	// Handy Value!-)
} ERROR_ENUM;

typedef struct STRING_TAG // Structure containing string
{
	char str[STR_LEN]; // Array of characters
} STRING_TYP;

typedef struct ERR_DATA_TAG // Structure containing Error handling data
{
	STRING_TYP err_strs[NUM_ERR_TYPS]; // Array messages for each error type 
	int line[NUM_ERR_TYPS];	// Array of line number for NEWEST occurance of error type.
	unsigned err_flgs;	// Set of Fault detection flags for each error type
} ERR_DATA_TYP;

typedef struct MOTOR_DATA_TAG // Structure containing motor state data
{
	ADC_PARAM_TYP adc_params; // Structure containing measured data from ADC
	HALL_PARAM_TYP hall_params; // Structure containing measured data from Hall sensors
	PWM_COMMS_TYP pwm_comms; // Structure containing PWM communication data between Client/Server.
	QEI_PARAM_TYP qei_params; // Structure containing measured data from QEI sensors
	PID_CONST_TYP pid_consts[NUM_IQ_ESTIMATES][NUM_PIDS]; // array of PID const data for different IQ Estimate algorithms 
	PID_REGULATOR_TYP pid_regs[NUM_PIDS]; // array of pid regulators used for motor control
	ERR_DATA_TYP err_data; // Structure containing data for error-handling
	int cnts[NUM_MOTOR_STATES]; // array of counters for each motor state	
	MOTOR_STATE_ENUM state; // Current motor state
	int meas_speed;	// speed, i.e. magnitude of angular velocity
	int est_Id;	// Estimated radial current value
	int est_Iq;	// Estimated tangential current value
	int req_Vd;	// Requested voltage producing radial magnetic field.
	int req_Vq;	// Requested voltage producing tangential magnetic field
	int req_veloc;	// Requested (target) angular velocity set by the user/comms interface
	int half_veloc;	// Half requested angular velocity
	int Vd_openloop;	// Requested Id value when tuning open-loop
	int Vq_openloop;	// Requested Iq value when tuning open-loop
	int pid_veloc;	// Output of angular velocity PID
	int pid_Id;	// Output of 'radial' current PID
	int pid_Iq;	// Output of 'tangential' current PID
	int set_Vd;	// Demand 'radial' voltage set by control loop
	int set_Vq;	// Demand 'tangential' voltage set by control loop 
	int prev_Vq;	// Previous Demand 'tangential' voltage
	int set_theta;	// theta value
	int start_theta; // Theta start position during warm-up (START and SEARCH states)
	int first_foc; // Flag set until first FOC (closed-loop) iteration completed

	int iters; // Iterations of inner_loop
	unsigned id; // Unique Motor identifier e.g. 0 or 1
	unsigned prev_hall; // previous hall state value
	unsigned end_hall; // hall state at end of cycle. I.e. next value is first value of cycle (001)
	int Iq_alg;	// Algorithm used to estimate coil current Iq (and Id)
	unsigned xscope;	// Flag set when xscope output required

	int theta_offset;	// Phase difference between the QEI and the coils
	int phi_err;	// Error diffusion value for Phi value
	int phi_off;	// Phi value offset
	int gamma_est;	// Estimate of leading-angle, used to 'pull' pole towards coil.
	int gamma_off;	// Gamma value offset
	int gamma_err;	// Error diffusion value for Gamma value
	int gamma_ramp;	// Gamma ramp value used to smoothly change between different Gamma values
	int phi_ramp;	// Phi ramp value used to smoothly change between different Phi values (phase lag)
	int Iq_err;	// Error diffusion value for scaling of measured Iq
	int adc_err;	// Error diffusion value for ADC extrema filter
	int prev_angl; 	// previous angular position
	unsigned prev_time; 	// previous open-loop time stamp

	int filt_val; // filtered value
	int coef_err; // Coefficient diffusion error
	int scale_err; // Scaling diffusion error 

	int temp; // MB~ Dbg
} MOTOR_DATA_TYP;

/*****************************************************************************/
void run_motor ( // run the motor inner loop
	unsigned motor_id,
	chanend? c_wd,
	chanend c_pwm,
	streaming chanend c_hall,
	streaming chanend c_qei,
	streaming chanend c_adc,
	chanend c_speed,
	chanend c_can_eth_shared 
);
/*****************************************************************************/

#endif /* _INNER_LOOP_H_ */
