/*
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
 */                                   
#ifndef HALL_CLIENT_H_
#define HALL_CLIENT_H_

#include <xs1.h>
#include <assert.h>
#include <print.h>

#include "hall_commands.h"


/*****************************************************************************/
unsigned get_hall_data( // Returns (4-bit) Hall sensor data from channel
	streaming chanend c_hall // Streaming channel for Hall sensor data
);
/*****************************************************************************/
#endif /* HALL_CLIENT_H_ */
