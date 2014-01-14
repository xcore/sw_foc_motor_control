/**
 * Module:  module_dsc_blocks
 * Version: 1v0alpha1
 * Build:   c9e25ba4f74e9049d5da65cb5c829a3d932ed199
 * File:    pid_regulator.c
 * Modified by : Srikanth
 * Last Modified on : 26-May-2011
 *
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

#include "pid_regulator.h"

/*****************************************************************************/
int init_one_pid_const( // Convert one floating point PID Constant to fixed point
	float inp_Ke, // Input Error constant
	int scale_factor // Ratio of Output Fixed-Point value to Input Floating-Point value
) // Return fixed point (integer) value
{
	int out_int; // returned output integer value


	if (inp_Ke == 0 )
	{
		return 0; // Return zero
	} // if (inp_Ke < 0 )

	if (inp_Ke < 0 )
	{
		assert(1 == 0); // ERROR: Negative values NOT supported
	} // if (inp_Ke < 0 )

	out_int = (int)(inp_Ke * (float)scale_factor + (float)0.5); // Scale-up floating-point to fixed-point

	assert( out_int > 16 ); // WARNING: Loss of precision

	return out_int; // Return rounded value
} // init_one_pid_const
/*****************************************************************************/
void init_all_pid_consts( // Initialise a set of floating point PID Constants
	PID_CONST_TYP * pid_const_p, // Pointer to PID constants data structure
	float inp_K_p, // Input Proportional Error constant
	float inp_K_i, // Input Integral Error constant
	float inp_K_d // Input Differential Error constant
)
{
	pid_const_p->K_p = init_one_pid_const( inp_K_p ,(1 << PID_CONST_LO_RES) );
	pid_const_p->K_d = init_one_pid_const( inp_K_d ,(1 << PID_CONST_HI_RES) );
	pid_const_p->K_i = init_one_pid_const( inp_K_i ,(1 << PID_CONST_HI_RES) );

	// Preset resolution for sum-of-errors to NO down-scaling
	pid_const_p->sum_res = 0;
	pid_const_p->half_scale = 0;
} // init_all_pid_consts
/*****************************************************************************/
void init_int_pid_consts( // Initialise a set of integer PID Constants
	PID_CONST_TYP * pid_const_p, // Pointer to PID constants data structure
	int inp_K_p, // Input Proportional Error constant
	int inp_K_i, // Input Integral Error constant
	int inp_K_d // Input Differential Error constant
)
{
	pid_const_p->K_p = inp_K_p;
	pid_const_p->K_d = inp_K_d;
	pid_const_p->K_i = inp_K_i;

	// Preset resolution for sum-of-errors to NO down-scaling
	pid_const_p->sum_res = 0;
	pid_const_p->half_scale = 0;
} // init_int_pid_consts
/*****************************************************************************/
void initialise_pid( // Initialise PID regulator 
	PID_REGULATOR_TYP * pid_regul_p // Pointer to PID regulator data structure
)
{
	// Initialise variables
	pid_regul_p->sum_err = 0;
	pid_regul_p->prev_err = 0;
	pid_regul_p->xtra_err = 0;
	pid_regul_p->low_err = 0;
	pid_regul_p->rem = 0;
} // initialise_pid
/*****************************************************************************/
void preset_pid( // Preset PID ready for first iteration
	unsigned motor_id, // Unique Motor identifier e.g. 0 or 1
	PID_REGULATOR_TYP * pid_regul_p, // Pointer to PID regulator data structure
	PID_CONST_TYP * pid_const_p, // Local pointer to PID constants data structure
	int open_val, // Open-loop requested value
	int closed_val, // Closed-loop requested value
	int meas_val // measured value
)
{
	S64_T tmp_64; // temporary 64-bit precision velue


	// Prevent divide by zero (by unused I_D PID)
	if (pid_const_p->K_i)
	{
		tmp_64 = ((S64_T)pid_const_p->K_p * (S64_T)(closed_val - meas_val)); // Lo-Res Up-scaled proportional error
		tmp_64 = (tmp_64 << (S64_T)PID_CONST_XTRA_RES); // Hi-Res Up-scaled proportional error
		tmp_64 = ((S64_T)open_val << (S64_T)PID_CONST_HI_RES) - tmp_64; // Subtract from Hi-Res Up-scaled Requested open-loop value

		// Integer division (with rounding) by Hi-Res up-scaled Integral constant
		pid_regul_p->sum_err = (int)((tmp_64 + ((S64_T)pid_const_p->K_i >> 1)) / (S64_T)pid_const_p->K_i);
	} //if (pid_const_p->K_i)
} // preset_pid 
/*****************************************************************************/
int get_pid_regulator_correction( // Computes new PID correction based on input error
	unsigned motor_id, // Unique Motor identifier e.g. 0 or 1
	PID_REGULATOR_TYP * pid_regul_p, // Pointer to PID regulator data structure
	PID_CONST_TYP * pid_const_p, // Local pointer to PID constants data structure
	int requ_val, // request value
	int meas_val,  // measured value
	int num_vals // Number of updates to perform
)
{
	int inp_err = (requ_val - meas_val); // Compute input error
	int diff_err; // Compute difference error
	int corr_err; // corrected error, by adding in diffusion remainder
	int down_err; // down-scaled error
	S64_T res_l_64 = 0; // Partial result at Low resolution (Kp) at 64-bit precision
	S64_T res_h_64 = 0; // Partial result at High resolution (Ki & Kd) at 64-bit precision
	int res_32; // Result at 32-bit precision


	// Build 64-bit result
	res_l_64 = (S64_T)pid_const_p->K_p * (S64_T)inp_err;

	// Check if Integral Error used
	if (pid_const_p->K_i)
	{
		corr_err = (num_vals * inp_err) + pid_regul_p->rem; // Add-in previous remainder
		down_err = (int)((corr_err + (S64_T)pid_const_p->half_scale) >> pid_const_p->sum_res); // Down-scale error 

		// Check for overflow
		while (pid_regul_p->sum_err > (MAX_ERR_SUM - down_err))
		{ // Overflow condition detected. down-scale
printf("PID Re-scale\n"); //MB~

			// Save old scaling factor as new half-scaling-factor
			pid_const_p->half_scale = (1 << pid_const_p->sum_res); 
			pid_const_p->sum_res++; // Double down-scaling factor

			assert(16 > pid_const_p->sum_res); // Check for over-large scaling factor

 			pid_regul_p->sum_err >>= 1; // Halve error-sum
			pid_const_p->K_i <<= 1; // Double constant for error-sum
 
			// Recompute down-scaled error
			down_err = (int)((corr_err + (S64_T)pid_const_p->half_scale) >> pid_const_p->sum_res); // Down-scale error 
		} // while (pid_regul_p->sum_err > (MAX_ERR_SUM - down_err))

		pid_regul_p->sum_err += down_err; // Update Sum of (down-scaled) errors
		res_h_64 += (S64_T)pid_const_p->K_i * (S64_T)pid_regul_p->sum_err;

		pid_regul_p->rem = corr_err - (down_err << pid_const_p->sum_res); // Update remainder
	} // if (pid_const_p->K_d)
 
	// Check if Differential Error used
	if (pid_const_p->K_d)
	{
		diff_err = (inp_err - pid_regul_p->prev_err); // Compute difference error

		res_h_64 += (S64_T)pid_const_p->K_d * (S64_T)diff_err;

		pid_regul_p->prev_err = inp_err; // Update previous error
	} // if (pid_const_p->K_d)

pid_regul_p->prev_err = inp_err; // MB~ Dbg
 
	// Convert to High Resolution terms to Low resolution ...

	res_h_64 += (S64_T)pid_regul_p->xtra_err; // Add-in previous quantisation (diffusion) error
	res_l_64 += ((res_h_64 + (S64_T)PID_HALF_XTRA_SCALE) >> (S64_T)PID_CONST_XTRA_RES); // Add in down-scaled result
	pid_regul_p->xtra_err = (int)(res_h_64 - (res_l_64 << (S64_T)PID_CONST_XTRA_RES)); // Update diffusion error

	// Convert to 32-bit result ...

	res_l_64 += (S64_T)pid_regul_p->low_err; // Add-in previous quantisation (diffusion) error
	res_32 = (int)((res_l_64 + (S64_T)PID_HALF_LO_SCALE) >> (S64_T)PID_CONST_LO_RES); // Down-scale result
	pid_regul_p->low_err = (int)(res_l_64 - ((S64_T)res_32 << (S64_T)PID_CONST_LO_RES)); // Update diffusion error

	return res_32;
} // get_pid_regulator_correction 
/*****************************************************************************/
// pid_regulator.c
