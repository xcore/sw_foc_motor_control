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

#include "hall_client.h"

/*****************************************************************************/
void foc_hall_get_parameters( // Returns (4-bit) Hall sensor parameters from channel
	HALL_PARAM_TYP &hall_param_s,	// Reference to structure containing Hall parameters
	streaming chanend c_hall // Streaming channel for Hall sensor data
)
{
	c_hall <: HALL_CMD_DATA_REQ;	// Request new hall sensor data
	c_hall :> hall_param_s; // Read new hall sensor parameters

	return;
} // foc_hall_get_data
/*****************************************************************************/
