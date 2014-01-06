/**
 * Module:  module_dsc_blocks
 * Version: 1v0alpha1
 * Build:   c9e25ba4f74e9049d5da65cb5c829a3d932ed199
 * File:    pid_regulator.h
 * Modified by : Srikanth
 * Last Modified on : 04-May-2011
 *
 * The copyrights, all other intellectual and industrial 
 * property rights are retained by XMOS and/or its licensors. 
 * Terms and conditions covering the use of this code can
 * be found in the Xmos End User License Agreement.
 *
 * Copyright XMOS Ltd 2010
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the 
 * copyright notice above.
 *
 **/

#ifndef __PI_REGULATOR_H__
#define __PI_REGULATOR_H__

#include <xs1.h>
#include <assert.h>
#include <print.h>

#include "app_global.h"

/* The PID regulator maintains a sum-of-errors, to prevent overflow but maintain maximum precision, 
 * the sum-of-errors is dynamically scaled.
 */

#ifdef BLDC_FOC
#define PID_CONST_RES 13 // Bit resolution of PID constants
#endif

#ifdef BLDC_BASIC
#define PID_CONST_RES 15 // Bit resolution of PID constants
#endif

#ifndef PID_CONST_RES
#define PID_CONST_RES 17 // Bit resolution of PID constants
#endif

#define PID_HALF_SCALE ((1 << PID_CONST_RES) >> 1) // Half PID constant scaling-factor, NB used for rounding
#define MAX_ERR_SUM (1 << 30) // Max. value of error-sum before re-scaling occurs

/** Different PID Regulators */
typedef enum PID_ETAG
{
  ID_PID = 0,  // Radial Current (in rotor frame of reference)
  IQ_PID,      // Tangential (Torque) Current (in rotor frame of reference)
  SPEED_PID,	 // Speed 
  NUM_PIDS     // Handy Value!-)
} PID_ENUM;

// Structure of Constant definitions for PID regulator
typedef struct PID_CONST_TAG
{
	int K_p; // PID Previous-error mix-amount
	int K_i; // PID Integral-error mix-amount
	int K_d; // PID Derivative-error mix-amount
	int sum_res; // Resolution used for sum-of-errors
	int half_scale; // Half sum-of-errors scaling-factor, NB used for rounding
} PID_CONST_TYP;

typedef struct PID_REGULATOR_TAG 
{
	int prev_err; // Previous error
	int sum_err; // Sum of errors
	int rem; // Remainder
	int qnt_err; // Quantisation Error
} PID_REGULATOR_TYP;

#ifdef __XC__
// XC Version
/*****************************************************************************/
void init_all_pid_consts( // Initialise a set of floating point PID Constants
	PID_CONST_TYP &pid_const_p, // Reference to PID constants data structure
	float inp_K_p, // Input Proportional Error constant
	float inp_K_i, // Input Integral Error constant
	float inp_K_d // Input Differential Error constant
);
/*****************************************************************************/
void init_int_pid_consts( // Initialise a set of integer PID Constants
	PID_CONST_TYP &pid_const_p, // Reference to PID constants data structure
	int inp_K_p, // Input Proportional Error constant
	int inp_K_i, // Input Integral Error constant
	int inp_K_d // Input Differential Error constant
);
/*****************************************************************************/
void initialise_pid( // Initialise PID settings
	PID_REGULATOR_TYP &pid_regul_s // Reference to PID regulator data structure
);
/*****************************************************************************/
void preset_pid( // Computes new PID correction based on input error
	unsigned motor_id, // Unique Motor identifier e.g. 0 or 1
	PID_REGULATOR_TYP &pid_regul_s, // Reference to PID regulator data structure
	PID_CONST_TYP &pid_const_p, // Reference to PID constants data structure
	int open_val, // Open-loop requested value
	int closed_val, // Closed-loop requested value
	int meas_val // measured value
);
/*****************************************************************************/
int get_pid_regulator_correction( // Computes new PID correction based on input error
	unsigned motor_id, // Unique Motor identifier e.g. 0 or 1
	PID_REGULATOR_TYP &pid_regul_s, // Reference to PID regulator data structure
	PID_CONST_TYP &pid_const_p, // Reference to PID constants data structure
	int requ_val, // request value
	int meas_val // measured value
);
/*****************************************************************************/
#else // ifdef __XC__
// C Version
/*****************************************************************************/
void init_pid_consts( // Initialise a set of PID Constants
	PID_CONST_TYP * pid_const_p, // Pointer to PID constants data structure
	int inp_K_p, // Input Proportional Error constant
	int inp_K_i, // Input Integral Error constant
	int inp_K_d // Input Differential Error constant
);
/*****************************************************************************/
void inititialise_pid( // Initialise PID settings
	PID_REGULATOR_TYP * pid_regul_p // Pointer to PID regulator data structure
);
/*****************************************************************************/
void preset_pid( // Preset PID data ready for first iteration
	unsigned motor_id, // Unique Motor identifier e.g. 0 or 1
	PID_REGULATOR_TYP * pid_regul_p, // Pointer to PID regulator data structure
	PID_CONST_TYP * pid_const_p, // Pointer to PID constants data structure
	int open_val, // Open-loop requested value
	int closed_val, // Closed-loop requested value
	int meas_val // measured value
);
/*****************************************************************************/
int get_pid_regulator_correction( // Computes new PID correction based on input error
	unsigned motor_id, // Unique Motor identifier e.g. 0 or 1
	PID_REGULATOR_TYP * pid_regul_p, // Pointer to PID regulator data structure
	PID_CONST_TYP * pid_const_p, // Pointer to PID constants data structure
	int requ_val, // request value
	int meas_val // measured value
);
/*****************************************************************************/
#endif // else !__XC__

#endif // ifndef __PI_REGULATOR_H__
