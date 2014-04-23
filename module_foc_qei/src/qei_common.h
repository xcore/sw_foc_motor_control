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
