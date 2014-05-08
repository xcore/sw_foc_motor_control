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

#ifndef _MASTER_PRINT_SCHEDULER_H_
#define _MASTER_PRINT_SCHEDULER_H_

#include <stdlib.h>

#include <xs1.h>
#include <assert.h>
#include <print.h>

#include "app_global.h"
#include "use_locks.h"
#include "qei_common.h"
#include "test_qei_common.h"

/** Define number of slave-print cores */
#define DISP_SLAVE_BITS 2 // No. of slaves in bits, NB Can probably use 1, but sailing close to the wind
#define NUM_DISP_SLAVES (1 << DISP_SLAVE_BITS) // No. of slaves
#define DISP_SLAVE_MASK (NUM_DISP_SLAVES - 1) // Bit-mask used to wrap slave offset

/*	WARNING: Bug Work-around.
 *	When fixed replace instances of MB_FIXME with NUM_DISP_INPUTS, and delete this define
 */
#define MB_FIXME 2

/** Print data classes */
typedef enum DISP_CLASS_ENUM_TAG
{
  DISP_CLASS_VECT = 0,	// Test Vectors
  DISP_CLASS_PROG,			// Progress Indicator
  DISP_CLASS_CHECK,			// Client Parameters to Check
  DISP_CLASS_GENR8,			// Generated Raw data values
	NUM_DISP_CLASSES			// Handy value!-)
} DISP_CLASS_ENUM_TYP;

/*****************************************************************************/
/** Collects and schedules display data
 * \param comm_data_s	// Structure containing common test data
 * \param c_slave[]	// Array of output channels for sending data to Display cores
 * \param c_inputs[]	// Array of input channel for receiving display data
 */
void master_print( // Collects and schedules display data
	const COMMON_TST_TYP &comm_data_s, // Reference to structure containing common test data
	streaming chanend c_slave[NUM_DISP_SLAVES], // Array of channels for sending data to Display cores
	streaming chanend c_inputs[MB_FIXME]	// Array of channel for receiving display data
);
/*****************************************************************************/
#endif /* _MASTER_PRINT_SCHEDULER_H_ */
