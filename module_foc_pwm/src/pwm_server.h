/*
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
 *
 **/                                   

#ifndef _PWM_SERVER_H_
#define _PWM_SERVER_H_

#include <xs1.h>
#include <assert.h>
#include <print.h>

#include "app_global.h"
#include "pwm_common.h"
#include "pwm_convert_width.h"

#ifndef LOCK_ADC_TO_PWM 
	#error Define. LOCK_ADC_TO_PWM in app_global.h
#endif // LOCK_ADC_TO_PWM

#define PWM_CLK_MHZ 250 // For historical reasons, PWM timings are based on a 250 MHz clock 

// Number of PWM time increments between ADC/PWM synchronisation points. NB Independent of Reference Frequency
#define INIT_SYNC_INCREMENT (PWM_MAX_VALUE)

/** Structure containing pwm server control data */
typedef struct PWM_SERV_TAG
{
	int id; // Motor Id
	unsigned ref_time; // Reference Time incremented every PWM period, all other times are measured relative to this value
	int data_ready; //Data ready flag
} PWM_SERV_TYP;

/*****************************************************************************/
/** \brief Implementation of the centre aligned inverted pair PWM server, with ADC synchronization
 *
 *  This server includes a port which triggers the ADC measurement
 *
 *  \param c_pwm PWM channel between Client and Server
 *  \param p32_pwm_hi the array of PWM ports (HI side)
 *  \param p32_pwm_lo the array of PWM ports (LO side)
 *  \param c_adc_trig the control channel for triggering the ADC
 *  \param p16_adc_sync a dummy port used for precise timing of the ADC trigger
 *  \param pwm_clk a clock for generating accurate PWM timing
 */
void foc_pwm_config( // Configure PWM ports
	chanend c_pwm[NUMBER_OF_MOTORS], // PWM channel between Client and Server
	buffered out port:32 p32_pwm_hi[NUMBER_OF_MOTORS][NUM_PWM_PHASES], // array of PWM ports (High side)  
	buffered out port:32 p32_pwm_lo[NUMBER_OF_MOTORS][NUM_PWM_PHASES], // array of PWM ports (Low side)   
	in port p16_adc_sync[NUMBER_OF_MOTORS], // Dummy port used with ADC trigger
	clock pwm_clk // clock for generating accurate PWM timing
);
/*****************************************************************************/
/** \brief Implementation of the centre aligned inverted pair PWM server, with ADC synchronization
 *
 *  This server includes a port which triggers the ADC measurement
 *
 *  \param motor_id  Motor identifier
 *  \param c_pwm PWM channel between Client and Server
 *  \param p32_pwm_hi the array of PWM ports (HI side)
 *  \param p32_pwm_lo the array of PWM ports (LO side)
 *  \param c_adc_trig the control channel for triggering the ADC
 *  \param p16_adc_sync a dummy port used for precise timing of the ADC trigger
 */
void foc_pwm_do_triggered( // Implementation of the Centre-aligned, High-Low pair, PWM server, with ADC synchronization
	unsigned motor_id, // Motor identifier
	chanend c_pwm, // PWM channel between Client and Server
	buffered out port:32 p32_pwm_hi[NUM_PWM_PHASES], // array of PWM ports (High side)  
	buffered out port:32 p32_pwm_lo[NUM_PWM_PHASES], // array of PWM ports (Low side)   
	chanend c_adc_trig, // ADC trigger channel 
	in port p16_adc_sync // Dummy port used with ADC trigger
);
/*****************************************************************************/

#endif // _PWM_SERVER_H_
