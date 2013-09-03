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

#ifndef _DISPLAY_GENERATOR_DATA_H_
#define _DISPLAY_GENERATOR_DATA_H_

#include <stdlib.h>

#include <xs1.h>
#include <assert.h>
#include <print.h>

#include "app_global.h"
#include "use_locks.h"
#include "qei_common.h"

#define DISP_BUF_BITS 4 // No. of bits used to represent buffer size
#define DISP_BUF_SIZ (1 << DISP_BUF_BITS) // buffer size
#define DISP_BUF_MASK (DISP_BUF_SIZ  - 1) // Used to wrap buffer offsets
 
/*****************************************************************************/
/** Displays Generated QEI test data
 * \param c_gen // Channel for receiving generated QEI values
 */
void disp_gen_data( // Displays Generated QEI test data
	streaming chanend c_gen // Channel for receiving generated QEI values
);
/*****************************************************************************/
#endif /* _DISPLAY_GENERATOR_DATA_H_ */
