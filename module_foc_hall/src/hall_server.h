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

#ifndef _HALL_SERVER_H_
#define _HALL_SERVER_H_

#include <xs1.h>
#include <print.h>
#include <assert.h>

#include "app_global.h"
#include "hall_common.h"

/** Structure containing HALL data for one motor */
typedef struct HALL_DATA_TAG // 
{
	unsigned inp_val; // Raw value on input port pins
	unsigned out_val; // Filtered output value
	int id; // Unique motor identifier
} HALL_DATA_TYP;

/*****************************************************************************/
/** Get Hall Sensor data from port (motor) and send to client
 * \param c_hall[]	// Array of data channels to client (carries processed Hall data)
 * \param p4_hall[]	// Array of input ports (carries raw Hall motor data)
 */
void foc_hall_do_multiple( // Get Hall Sensor data from port (motor) and send to client
	streaming chanend c_hall[],	// Array of data channels to client (carries processed Hall data)
	port in p4_hall[]						// Array of input ports (carries raw Hall motor data)
);
/*****************************************************************************/

#endif // _HALL_SERVER_H_
