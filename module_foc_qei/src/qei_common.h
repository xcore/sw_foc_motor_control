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

#ifndef _QEI_COMMON_H_
#define _QEI_COMMON_H_

#include "use_locks.h"
#include "app_global.h"

#ifndef PLATFORM_REFERENCE_MHZ
	#error Define. PLATFORM_REFERENCE_MHZ in app_global.h
#endif

#ifndef SECS_PER_MIN
	#error Define. SECS_PER_MIN in app_global.h
#endif

#ifndef QEI_PER_REV 
	#error Define. QEI_PER_REV in app_global.h
#endif // QEI_PER_REV

#define QEI_REV_MASK (QEI_PER_REV - 1) // Mask used to force QEI count into base-range [0..QEI_REV_MASK] 

#define QEI_SAMP_BITS 4 // Size of QEI input port sample in bits
#define QEI_SAMP_SIZ (1 << QEI_SAMP_BITS) // Max. No. of possible QEI sample values
#define QEI_SAMP_MASK (QEI_SAMP_SIZ - 1) // Mask used to extract sample bits from buffer

#define QEI_PHASE_MASK (0b0011) // 2 LS-bits contain [A,B] phase info.
#define QEI_ORIG_MASK (0b0100) // Bit_2 contain origin info.
#define QEI_NERR_MASK (0b1000) // Bit_3 contains error status (1 == No Errors)

#define QEI_PERIOD_LEN (QEI_PHASE_MASK + 1) // Number of different Phase combinations before a repeat. e.g [00 10 11 01]

/* Calculate speed definitions, preserving precision and preventing overflow !-)
 * 
 * The time difference between changes in QEI data is measured in 'ticks'.
 * For a Platform Reference frequency of 100 MHz, there will be 6,000,000,000 ticks/minute
 * If there are 1024 different QEI points per revolution, then angular velocity (in RPM) is 
 * (60 * 100000000)/(1024 * Tick_Diff) or (TICKS_PER_MIN_PER_QEI / Tick_Diff) 
 */
#define TICKS_PER_SEC_PER_QEI ((PLATFORM_REFERENCE_HZ + (QEI_PER_REV >> 1)) / QEI_PER_REV) // Ticks/sec/angular_position (rounded) // 18-bits
#define TICKS_PER_MIN_PER_QEI (SECS_PER_MIN * TICKS_PER_SEC_PER_QEI) // Ticks/min/angular_position // 24 bits

/** Different QEI Commands (Client --> Server) */
typedef enum CMD_QEI_ETAG
{
	// NB Can't use non-negative integers, due to conflicts in test harness
  QEI_CMD_ACK = (-2),	// QEI Server Command Acknowledged (Control)
	QEI_CMD_LOOP_STOP = (-1), // Stop while-loop.
	QEI_CMD_DATA_REQ = 1,	// QEI Data Request
} CMD_QEI_ENUM;

/** Different QEI Error states */
typedef enum ERROR_QEI_ETAG
{
  QEI_ERR_OFF = 0,	// No Error
  QEI_ERR_ON,			// Error
  NUM_QEI_ERRS	// Handy Value!-)
} ERROR_QEI_ENUM;

/** Raw QEI data type (on input pins) */
typedef unsigned long QEI_RAW_TYP;

/** Structure containing QEI parameters for one motor */
typedef struct QEI_PARAM_TAG // 
{
	int tot_ang_this;	// Total angle traveresed since time=0, for this motor
	int tot_ang_master;	// Total angle traveresed since time=0, for master motor (NB Maybe same as this motor)
	unsigned phase_period; // time (in ticks) to traverse one QEI phase (angular position)
	int corr_ang;	// Angular correction (Old - New)
	int orig_corr; // Flag set if origin correction available
	ERROR_QEI_ENUM err;	// Flag set when Error condition detected

	// WARNING: If editing this structure, also edit parameter_compare() in check_qei_test.xc 
} QEI_PARAM_TYP;

#endif /* _QEI_COMMON_H_ */
