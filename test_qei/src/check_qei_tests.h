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

#ifndef _CHECK_QEI_TESTS_H_
#define _CHECK_QEI_TESTS_H_

#include <xs1.h>
#include <assert.h>
#include <print.h>

#include "app_global.h"
#include "test_qei_common.h"
#include "qei_client.h"

/*****************************************************************************/
void disp_all_qei_client_data( // Display QEI results for all motors
	streaming chanend c_qei[] // Array of QEI channels between Client and Server
);
/*****************************************************************************/
#endif /* _CHECK_QEI_TESTS_H_ */
