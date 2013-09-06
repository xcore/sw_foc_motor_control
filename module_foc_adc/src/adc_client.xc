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

#include "adc_client.h"

/*****************************************************************************/
void foc_adc_get_parameters( // Read 2 of 3 ADC values from the motor, and convert them into signed 32-bit integer
	ADC_PARAM_TYP &adc_param_s, // Reference to structure containing ADC parameters
	streaming chanend c_adc_cntrl // channel connecting to ADC client and server
)
{
	c_adc_cntrl <: ADC_CMD_DATA_REQ;	// Request ADC data
	c_adc_cntrl :> adc_param_s;	// Receive ADC parameters

	return;
} // foc_adc_get_data
/*****************************************************************************/
