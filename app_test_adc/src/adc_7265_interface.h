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

#ifndef _ADC_7265_INTERFACE_H_
#define _ADC_7265_INTERFACE_H_

#include <xs1.h>
#include <assert.h>
#include <print.h>
#include <safestring.h>
#include <syscall.h>

#include "app_global.h"
#include "use_locks.h"
#include "adc_common.h"
#include "test_adc_common.h"
#include "sine_common.h"

/** bit-shift used to convert standardised ADC value (Currently 25-bit) to 
 *  format used by the ADC chip: Currently 14-bit, MSB transmitted 1st (bit-reversed)
 */
#define INT32_BITS (sizeof(int) * BITS_IN_BYTE) // No. of bits in 32-bit integer
#define ADC_GEN_BITS (GAIN_BITS + SIN_VAL_BITS + 1) //25 No. of bits in Standardised ADC generated value NB (1 is sign bit)
#define SHIFT_BITS (INT32_BITS - ADC_GEN_BITS - ADC_PRE_PAD_BITS) //5 Bit-shift used in int_32 -> ADC_TYP conversion

/*****************************************************************************/
/** Generate ADC test data for all motors
 * \param	c_pwm2adc_trig[] // Array of channels for sending PWM-to_ADC trigger pulse
 * \param	c_adc_sin, // Channel for communication with Sine_Generator cores
 * \param	pb32_tst_data[]	// Array of ADC data ports for transmitting raw ADC values (NB For Phase_A and Phase_B)
 * \param	p1_tst_ready // 1-bit port used for ready signal
 * \param	p1_tst_sclk // 1-bit port used for serial clock
 * \param	tst_xclk // Internal XMOS clock
 */
void adc_7265_interface( // Generate ADC Test data for all motors
	chanend c_pwm2adc_trig[], // Array of channels for sending PWM-to_ADC trigger pulse
	streaming chanend c_sin, // Channel for communication with Sine_Generator cores
	buffered out port:32 pb32_tst_data[],	// Array of ADC data ports for transmitting raw ADC values (NB For Phase_A and Phase_B)
	in port p1_tst_ready, 										// 1-bit port used for ready signal
	in port p1_tst_sclk, 										// 1-bit port used for serial clock
	clock tst_xclk // Internal XMOS clock
);
/*****************************************************************************/
#endif /* _ADC_7265_INTERFACE_H_ */
