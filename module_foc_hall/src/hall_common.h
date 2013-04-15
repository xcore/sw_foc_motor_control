/**
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
#ifndef _HALL_COMMON_H_
#define _HALL_COMMON_H_

/** Used to mask out 4-bits of Hall Sensor Data */
#define HALL_ALL_MASK (0b1111)	// Used to mask out all 4-bits of Hall Sensor Data

/** Used to mask out Hall error-bit */
#define HALL_ERR_MASK (0b1000) // Used to mask out Hall Error Bit(s)

/** Hall Request Data Command */
#define HALL_CMD_DATA_REQ 1 // Request new hall sensor data

#define HALL_PHASE_MASK (HALL_ALL_MASK & (~HALL_ERR_MASK)) // Used to mask out 3 Hall Sensor Phase Bits

#endif /* _HALL_COMMON_H_ */
