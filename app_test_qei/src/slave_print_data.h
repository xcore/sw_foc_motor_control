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

#ifndef _SLAVE_PRINT_DATA_H_
#define _SLAVE_PRINT_DATA_H_

#include <stdlib.h>

#include <xs1.h>
#include <assert.h>
#include <print.h>

#include "app_global.h"
#include "use_locks.h"
#include "qei_common.h"
#include "test_qei_common.h"
#include "master_print_scheduler.h"

/** Type containing display data */
typedef struct DISP_DATA_TAG // Structure containing display data
{
	char qei_str[(QEI_BITS + 1)]; // display string for QEI value
	DISP_INP_ENUM_TYP inp_id; // display input identifier
	int print_cnt; // Print progress counter
} DISP_DATA_TYP;

/*****************************************************************************/
/** Displays data
 * \param comm_data_s	// Structure containing common test data
 * \param c_master,	// Channel for receiving data from master-print core
 * \param disp_id // Unique Display core identifier
 */
void slave_print( // Displays data
	const COMMON_TST_TYP &comm_data_s, // Structure containing common test data
	streaming chanend c_master,	// Channel for receiving data from master-print core
	int disp_id // Unique Display core identifier
);
/*****************************************************************************/
#endif /* _SLAVE_PRINT_DATA_H_ */
