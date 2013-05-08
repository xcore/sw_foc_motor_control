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

#ifndef _TEST_ADC_COMMON_H_
#define _TEST_ADC_COMMON_H_

/* The S/W and H/W may NOT use the same No. of bits to represent an ADC value.
 * The H/W interface must convert whatever data is supplied into the range defined below
 */
#define ADC_RESOLUTION 12 // 12: gives ADC values in the range -2048..2047
#define NUM_ADC_LVLS (1 << ADC_RESOLUTION) // No. Of different ADC amplitude levels
#define MAX_ADC_VAL ((NUM_ADC_LVLS >> 1) - 1) // Max. ADC amplitude
#define MIN_ADC_VAL (-(NUM_ADC_LVLS >> 1)) // Min. ADC amplitude

#define TEST_TIME (40 * MICRO_SEC) // Time between tests

#endif /* _TEST_ADC_COMMON_H_ */
