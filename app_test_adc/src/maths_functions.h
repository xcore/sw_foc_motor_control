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

#ifndef _MATHS_FUNCTIONS_H_
#define _MATHS_FUNCTIONS_H_

#include <math.h>

#define PI ((double)3.14159265)
#define PIx2 ((double)2 * PI)

/*****************************************************************************/
/** Returns scaled sine value value
 * \param index // Index into sample-set
 * \param wave_len // Sine wavelength measured in samples
 * \param amplitude // Sine-wave amplitude
 */
int get_sine_value( // Returns scaled sine value value
	int index, // Index into sample-set
	int wave_len, // Sine wavelength measured in samples
	int amplitude // Sine-wave amplitude
);
/*****************************************************************************/
#endif /* _MATHS_FUNCTIONS_H_ */
