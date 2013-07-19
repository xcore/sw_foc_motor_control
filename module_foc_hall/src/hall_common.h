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

#include "app_global.h"

#ifndef PLATFORM_REFERENCE_MHZ
	#error Define. PLATFORM_REFERENCE_MHZ in app_global.h
#endif

#ifndef SECS_PER_MIN
	#error Define. SECS_PER_MIN in app_global.h
#endif

#ifndef NUM_POLE_PAIRS 
	#error Define. NUM_POLE_PAIRS in app_global.h
#endif // NUM_POLE_PAIRS

#ifndef HALL_PER_REV 
	#error Define. HALL_PER_REV in app_global.h
#endif // HALL_PER_REV

#define HALL_BITS 4 // No of Active bits in QEI value
#define HALL_PHASE_MASK (0b0111) // Used to mask out 3 Hall Sensor Phase Bits
#define HALL_NERR_MASK (0b1000) // Used to mask out Hall Error Bit(s)

/** Hall Request Data Command */
#define HALL_CMD_DATA_REQ 1 // Request new hall sensor data

#define HALL_ALL_MASK (HALL_NERR_MASK | HALL_PHASE_MASK) // Used to mask out all 4-bits of Hall Sensor Data

/* Calculate speed definitions, preserving precision and preventing overflow !-)
 * 
 * The time difference between changes in HALL data is measured in 'ticks'.
 * For a Platform Reference frequency of 100 MHz, there will be 6,000,000,000 ticks/minute
 * If there are 24 different HALL points per revolution, then angular velocity (in RPM) is 
 * (60 * 100000000)/(24 * Tick_Diff) or (TICKS_PER_MIN_PER_HALL / Tick_Diff) 
 */
#define TICKS_PER_SEC_PER_HALL ((PLATFORM_REFERENCE_HZ + (HALL_PER_REV >> 1)) / HALL_PER_REV) // Ticks/sec/angular_position (rounded) // 14-bits
#define TICKS_PER_MIN_PER_HALL (SECS_PER_MIN * TICKS_PER_SEC_PER_HALL) // Ticks/min/angular_position // 30 bits

// QEI Command Codes (Client --> Server) 
#define QEI_CMD_DATA_REQ	1	// QEI Data Request

/** Raw Hall data type (on input pins) */
typedef unsigned long HALL_RAW_TYP;

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
	unsigned hall_val; // Hall sensor value (3 LS bits)
	int err; // Error Status
} HALL_PARAM_TYP;

#endif /* _HALL_COMMON_H_ */
