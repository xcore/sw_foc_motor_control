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

#include "sine_cosine.h"
#include "sine_lookup.h"

#if (NUM_SIN_ANGS_IN_REV == 256)
	#include "sine_table_256.h"
	#define TABLE_FOUND 1
#elif (NUM_SIN_ANGS_IN_REV == 1024)
	#include "sine_table_1024.h"
	#define TABLE_FOUND 1
#elif (NUM_SIN_ANGS_IN_REV == 4096)
	#include "sine_table_4096.h"
	#define TABLE_FOUND 1
#endif // NUM_SIN_ANGS_IN_REV 

#if (TABLE_FOUND != 1)
#error ERROR. Sine-Table NOT loaded
#endif // (TABLE_FOUND != 1)

/*****************************************************************************/
int sine( 
	unsigned inp_angle 
)
{
#ifdef FAULHABER_MOTOR
	unsigned cur_angle = ((inp_angle + 2) >> 2)
#else
	unsigned cur_angle = inp_angle;
#endif
	unsigned quad_id; // Quadrant identifier (0..3)
	int out_val; // Ouput value

	cur_angle &= (NUM_SIN_ANGS_IN_REV-1); // Clip angle into 0..360 degree range
	quad_id = (cur_angle >> SINE_ANG_BITS); // Determine which angle quadrant

	// Determine which angle quadrant
	switch(quad_id)
	{
		case 0 :
			out_val = sine_table[cur_angle];
		break; // case IO_CMD_FLIP_SPIN
	
		case 1 :
			out_val = sine_table[NUM_SIN_ANGS_IN_Qx2 - cur_angle];
		break; // case IO_CMD_FLIP_SPIN
	
		case 2 :
			out_val = -sine_table[cur_angle - NUM_SIN_ANGS_IN_Qx2];
		break; // case IO_CMD_FLIP_SPIN
	
		case 3 :
			out_val = -sine_table[NUM_SIN_ANGS_IN_REV - cur_angle];
		break; // case IO_CMD_FLIP_SPIN
	
    default: // Unsupported Quadrant id
			assert(0 == 1); // quad_id NOT supported
    break; // default
	} // switch(quad_id)

	return out_val;
} // sine
/*****************************************************************************/
int cosine( 
	unsigned inp_angle 
)
{
#ifdef FAULHABER_MOTOR
	unsigned cur_angle = ((inp_angle + 2) >> 2);
#else
	unsigned cur_angle = inp_angle;
#endif
	unsigned quad_id; // Quadrant identifier (0..3)
	int out_val; // Ouput value


	cur_angle &= (NUM_SIN_ANGS_IN_REV-1); // Clip angle into 0..360 degree range
	quad_id = (cur_angle >> SINE_ANG_BITS); // Determine which angle quadrant

	// Determine which angle quadrant
	switch(quad_id)
	{
		case 0 :
			out_val = sine_table[NUM_SIN_ANGS_IN_QUAD - cur_angle];
		break; // case IO_CMD_FLIP_SPIN
	
		case 1 :
			out_val = -sine_table[cur_angle - NUM_SIN_ANGS_IN_QUAD];
		break; // case IO_CMD_FLIP_SPIN
	
		case 2 :
			out_val = -sine_table[NUM_SIN_ANGS_IN_Qx3 - cur_angle];
		break; // case IO_CMD_FLIP_SPIN
	
		case 3 :
			out_val = sine_table[cur_angle - NUM_SIN_ANGS_IN_Qx3];
		break; // case IO_CMD_FLIP_SPIN
	
    default: // Unsupported Quadrant id
			assert(0 == 1); // quad_id NOT supported
    break; // default
	} // switch(quad_id)

	return out_val;
} // cosine
/*****************************************************************************/
// sine_cosine.c
