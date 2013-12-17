/*
 * \file adc_common.h
 *
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
 **/                                   
#ifndef _ADC_COMMON_H_
#define _ADC_COMMON_H_

#include "use_locks.h"

#define ADC_UPSCALE_BITS 7 // No of bits used when upsacaling ADC values (to improve precision)
#define ADC_HALF_UPSCALE (1 << (ADC_UPSCALE_BITS - 1)) // Half Up-scaling factor (Used for rounding)

/** Different ADC Phases */
typedef enum ADC_PHASE_ETAG
{
  ADC_PHASE_A = 0,
  ADC_PHASE_B,
  ADC_PHASE_C,
	NUM_ADC_PHASES // 3 ADC phases [A, B, C]
} ADC_PHASE_ENUM;

#define USED_ADC_PHASES (NUM_ADC_PHASES - 1) // NB 3rd Phase can be inferred as 3 phases sum to zero

/** Different ADC Commands */
typedef enum CMD_ADC_ETAG
{
  ADC_CMD_DATA_REQ = 0,  // Request ADC data
	ADC_CMD_LOOP_STOP, // Stop while-loop.
	ADC_CMD_ACK,	// Acknowledge Command from control loop
  NUM_ADC_CMDS    // Handy Value!-)
} CMD_ADC_ENUM;

typedef signed long ADC_TYP;

/** Structure containing ADC parameters for one motor */
typedef struct ADC_PARAM_TAG // Structure containing ADC parameters
{
	ADC_TYP vals[NUM_ADC_PHASES]; // Array of ADC values for each phase
} ADC_PARAM_TYP;

#endif /* _ADC_COMMON_H_ */
