/*
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
 */

#include <xs1.h>
#include <assert.h>
#include <print.h>

#include "app_global.h"
#include "qei_common.h"

#ifndef _QEI_CLIENT_H__
#define _QEI_CLIENT_H__

/* These defines are used to calculate offset between Hall-state origin, and QEI origin.
 * There are 6 Hall-states per revolution, 60 degrees each, half-way through each state is therefore 30 degrees.
 * There may be 1024 QEI counts/rev (QEI_PER_REV) (See app_global.h)
 */
#define QEI_PER_POLE (QEI_PER_REV / NUM_POLE_PAIRS) // e.g. 256 No. of QEI sensors per pole 
#define QEI_POLE_MASK (QEI_PER_POLE - 1) // Mask used to force QEI count into base-range [0..QEI_POLE_MASK] 

#define THETA_HALF_PHASE (QEI_PER_POLE / 12) // e.g. ~21 CoilSectorAngle/2 (6 sectors = 60 degrees per sector, 30 degrees per half sector)

/** Structure containing QEI parameters for one motor */
typedef struct QEI_PARAM_TAG // 
{
	int theta;		// Angular position
	int veloc;		// Angular velocity
	int rev_cnt;	// Revolution counter (No. of origin traversals)
} QEI_PARAM_S;

/*****************************************************************************/
void foc_qei_get_data( // Returns New QEI data (speed, position, etc)
	QEI_PARAM_S &qei_data_s,	// Reference to structure containing QEI data
	streaming chanend c_qei	// Channel connecting to QEI client & server
); // On returns qei_data_s is updated
/*****************************************************************************/

#endif /* _QEI_CLIENT_H__ */
