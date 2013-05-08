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

#ifndef _CHECK_HALL_TESTS_H_
#define _CHECK_HALL_TESTS_H_

#include <stdlib.h>

#include <xs1.h>
#include <assert.h>
#include <print.h>

#include "app_global.h"
#include "use_locks.h"
#include "test_hall_common.h"
#include "hall_client.h"

/*****************************************************************************/
/** Display HALL results for all motors
 * \param c_hall[]	// Array of channels connecting HALL client & server
 */
void disp_all_hall_client_data( // Display HALL results for all motors
	streaming chanend c_hall[] // Array of HALL channels between Client and Server
);
/*****************************************************************************/
#endif /* _CHECK_HALL_TESTS_H_ */
