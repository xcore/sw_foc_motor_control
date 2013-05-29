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

#ifndef NUM_POLE_PAIRS 
	#error Define. NUM_POLE_PAIRS in app_global.h
#endif // NUM_POLE_PAIRS

/** Define the number of different HALL sensor positions per pole-pair */
#define NUM_HALL_PHASES 6 // Number of HALL Phases (per pole-pair)

/** Used to mask out Hall error-bit */
#define HALL_NERR_MASK (0b1000) // Used to mask out Hall Error Bit(s)

/** Used to mask out 3 Hall Sensor Phase Bits */
#define HALL_PHASE_MASK (0b0111) // Used to mask out 3 Hall Sensor Phase Bits

/** Hall Request Data Command */
#define HALL_CMD_DATA_REQ 1 // Request new hall sensor data

#define HALL_ALL_MASK (HALL_NERR_MASK | HALL_PHASE_MASK) // Used to mask out all 4-bits of Hall Sensor Data

#define HALL_PER_REV (NUM_HALL_PHASES * NUM_POLE_PAIRS) // No. of HALL positions per Revolution

/** Different Hall Error states */
typedef enum ERROR_HALL_ETAG
{
  HALL_ERR_OFF = 0,	// No Error
  HALL_ERR_ON,			// Error
  NUM_HALL_ERRS	// Handy Value!-)
} ERROR_HALL_ENUM;

/** Structure containing HALL parameters for one motor */
typedef struct HALL_PARAM_TAG // 
{
	unsigned hall_val; // Hall sensor value (4 LS bits)
	int err; // Error Status
} HALL_PARAM_TYP;

#endif /* _HALL_COMMON_H_ */
