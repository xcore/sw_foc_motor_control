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
 */                                   
#ifndef _HALL_CLIENT_H_
#define _HALL_CLIENT_H_

#include <xs1.h>
#include <assert.h>
#include <print.h>

#include "app_global.h"
#include "hall_common.h"

/** Structure containing HALL parameters for one motor */
typedef struct HALL_PARAM_TAG // 
{
	unsigned hall_val; // Hall sensor value (4 LS bits)
} HALL_PARAM_TYP;

/*****************************************************************************/
/** On return structure contains new Hall data (4-bit value)
 * \param qei_data_s	// Reference to structure containing QEI data
 * \param c_hall // data channel to server (carries processed Hall data)
 */
void foc_hall_get_data( // On return structure contains new Hall data (4-bit value)
	HALL_PARAM_TYP &hall_data_s,	// Reference to structure containing Hall parameters
	streaming chanend c_hall // Streaming channel for Hall sensor data
);
/*****************************************************************************/
#endif /* _HALL_CLIENT_H_ */
