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

#ifndef _CAPTURE_PWM_DATA_H_
#define _CAPTURE_PWM_DATA_H_

#include <stdlib.h>

#include <xs1.h>
#include <assert.h>
#include <print.h>
#include <safestring.h>

#include "app_global.h"
#include "use_locks.h"
#include "test_pwm_common.h"


/** Define Input poer buffer size in bits */
#define PORT_BUF_BITS 7 // 3 Input port buffer size in bits, NB Can probably use 2, but sailing cloase to the wind
#define NUM_PORT_BUFS (1 << PORT_BUF_BITS) // No. of input port buffers used for storing PWM widths, NB Can probably use 4, but sailing cloase to the wind
#define PORT_BUF_MASK (NUM_PORT_BUFS - 1) // Bit-mask used to wrap input port buffer offset

/** Define number of channels in bits */
#define CHAN_BITS 3 // No. of channels in bits, NB Can probably use 2, but sailing cloase to the wind
#define NUM_CHANS (1 << CHAN_BITS) // No. of channel 
#define CHAN_MASK (NUM_CHANS - 1) // Bit-mask used to wrap channel offset

/*****************************************************************************/
/** Display PWM results for all motors
 * \param p32_tst_hi, // array of PWM ports (High side)  
 * \param p32_tst_lo, // array of PWM ports (Low side)   
 * \param c_chk // Channel for sending PWM data to test checker
 * \param c_adc_trig // ADC trigger channel 
 */
void capture_pwm_client_data( // Captures PWM data from input pins for one motor
	buffered in port:32 p32_tst_hi[], // array of PWM ports (High side)  
	buffered in port:32 p32_tst_lo[], // array of PWM ports (Low side)   
	streaming chanend c_chk // Channel for transmitting PWM data to test checker
);
/*****************************************************************************/
#endif /* _CAPTURE_PWM_DATA_H_ */
