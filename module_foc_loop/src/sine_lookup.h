/**
 * The copyrights, all other intellectual and industrial 
 * property rights are retained by XMOS and/or its licensors. 
 * Terms and conditions covering the use of this code can
 * be found in the Xmos End User License Agreement.
 *
 * Copyright XMOS Ltd 2011
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the 
 * copyright notice above.
 *
 **/                                   
#ifndef __SINE_LOOKUP_H__
#define __SINE_LOOKUP_H__

// WARNING: If the value of SINE_RES_BITS is changed sine_table[] needs to be regenerated
#define SINE_RES_BITS 14 // Number of bits used to scale Sine/Cosine functions
#define MAX_SINE (1 << SINE_RES_BITS) // Max. table value, i.e. sine( 90 degrees )
#define HALF_SINE (MAX_SINE >> 1) // Half of max. sine value ), NB used for rounding

extern short sine_table[];

/** \brief Look up the fixed point sine value
 *
 * This looks up the sine of a value. The value is the index into the
 * sine table, rather than a particular angular measurement. The sine
 * table has 256 entries, so each index is 1.4 degrees.
 *
 * A table of 256 entries is suitable for an encoder angle measured in
 * 1024 steps, attached to a 4 pole motor. Each encoder step is
 * 360/1024 = 0.35 physical degrees, but this is worth 4 times as many
 * electrical degrees, or 1.4 electrical degrees.
 *
 * The result is in fixed point 18.14 format.
 *
 * \param angle the index of the sine value to look up
 * \return the 18.14 fixed point sine value
 */
int sine( unsigned angle );

/** \brief Look up the fixed point cosine value
 *
 * This looks up the cosine of a value. The value is the index into the
 * sine table, rather than a particular angular measurement.
 *
 * \param angle the index of the cosine value to look up
 * \return the 18.14 fixed point cosine value
 */
int cosine( unsigned angle );

#endif /*__SINE_LOOKUP_H__*/
