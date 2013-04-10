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
void foc_adc_get_data( // Read 2 of 3 ADC values from the motor, and convert them into signed 32-bit integer
	ADC_PARAM_TYP &adc_data_s, // Reference to structure containing ADC data
	streaming chanend c_adc_cntrl // channel connecting to ADC client and server
)
{
	int phase_cnt; // ADC Phase counter
	int adc_sum = 0; // Accumulator for transmitted ADC Phases


	c_adc_cntrl <: ADC_CMD_REQ;	// Request ADC data */

	// Loop through used phases of ADC data
	for (phase_cnt=0; phase_cnt<USED_ADC_PHASES; ++phase_cnt) 
	{
		c_adc_cntrl :> adc_data_s.vals[phase_cnt];	// Receive One phase of ADC data

		adc_sum += adc_data_s.vals[phase_cnt]; // Add adc value to sum
	} // for phase_cnt

	// Calculate last ADC phase from previous phases (NB Sum of phases is zero)
	adc_data_s.vals[(NUM_ADC_PHASES - 1)] = -adc_sum;

	return;
} // foc_adc_get_data
/*****************************************************************************/
