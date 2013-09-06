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

#include "maths_functions.h"

/*****************************************************************************/
int get_sine_value( // Returns scaled sine value value
	int index, // Index into sample-set
	int wave_len, // Sine wavelength measured in samples
	int amplitude // Sine-wave amplitude
)
{
	double inp_rad = PIx2 * (double)index / (double)wave_len; // Input to sine in radians
	double out_val; // floating-point output value


	out_val = (double)amplitude * sin( inp_rad );

	// Check sign of result
	if (0 > out_val)
	{
		out_val -= 0.5;
	} // if (0 > out_val)
	else
	{
		out_val += 0.5;
	} // else !(0 > out_val)

	return (int)out_val;
} // get_sine_value
/*****************************************************************************/
